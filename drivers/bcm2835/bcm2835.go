// Package bcm2835 bit-bangs SWD through the Raspberry Pi GPIO registers mapped
// directly from /dev/gpiomem.
//
// # Which boards
//
// The BCM2835 register layout, which BCM2836, BCM2837 and BCM2711 all kept:
//
//	Pi 1, Pi 2, Pi 3, Pi 4, Pi Zero, Pi Zero 2 W
//	CM1, CM3, CM3+, CM4
//
// It is NOT the Pi 5, Pi 500 or CM5. Those put the GPIO on an RP1 southbridge
// across PCIe, with a different register layout and no /dev/gpiomem -- they
// expose /dev/gpiomem0..4 instead. Opening this driver there fails with a
// message saying so rather than writing to the wrong addresses. Use
// [github.com/kvth/swd-to-go/drivers/linuxgpiod] on those boards.
//
// Register offsets and the memory-barrier discipline are taken from this repo's
// own C++ gateway (gpio_regs.h): GPFSEL0/GPSET0/GPCLR0/GPLEV0 at the same byte
// offsets, one atomic load or store per access in place of gpio_regs.h's
// volatile-pointer-plus-__sync_synchronize(), for the same reason -- without a
// barrier the CPU can merge or reorder back-to-back writes to GPSET0/GPCLR0 and
// drop a clock edge.
//
// Pull-up/down control is deliberately not implemented, matching gpio_regs.h:
// this repo's own gateway runs correctly without it, on the assumption the board
// supplies SWDIO's pull-up externally, so there is nothing here to get wrong by
// guessing at a Pi model's pull-control register layout -- BCM2711 moved it, and
// the BCM2835 sequence is a silent no-op there.
//
// # Calibrating
//
// A driver comes back from Open uncalibrated: the cost of the delay loop is not
// known yet, so SetFrequencyHz cannot pace anything and the clock runs as fast
// as bare register writes allow. Call Calibrate to measure it, or SetSpeedCoeffs
// with numbers measured earlier, before setting a clock and expecting it to
// mean anything.
//
// Measuring is not done for you at Open because it clocks SWCLK for a fraction
// of a second while it runs, and when that is safe -- before a target is
// attached, between sessions, never mid-transfer -- is something only the caller
// knows.
//
// # Why three types
//
// The three wiring types below are deliberately separate. Folding them into one
// behind a flag puts a direction test on every turnaround, and hiding them
// behind an interface puts a dynamic call on every pin operation. Since the
// maximum reachable clock is speed_coeff / speed_offset, anything that inflates
// the per-edge cost lowers the top speed the hardware can be driven at --
// calibration measures that cost, it does not remove it.
//
// What that costs is duplication: SwdTransfer appears once per wiring type,
// because it calls SwdioOutEnable and SwdioOutDisable and so cannot live on the
// shared core. A fix to one belongs in all of them, and nothing here checks
// that -- this package's fake register block remembers only the last write to
// GPSET0 and GPCLR0, so it cannot count clock edges, which is what comparing
// the copies would turn on. drivers/linuxgpiod records every ioctl and does
// make that comparison; the same reasoning applies to these three.
package bcm2835

import (
	"fmt"
	"os"
	"sort"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"

	"github.com/kvth/swd-to-go"
)

// BCM2711 / Raspberry Pi GPIO register offsets, in bytes from the GPIO base --
// see gpio_regs.h.
const (
	blockSize = 4096

	gpfsel0 = 0x00
	gpset0  = 0x1C
	gpclr0  = 0x28
	gplev0  = 0x34

	funcInput  = 0
	funcOutput = 1
)

// DAP transfer request bits, as they arrive from the layer above.
const (
	dapTransferRnW       = 1 << 1
	dapTransferTIMESTAMP = 1 << 7
)

// DAP transfer response bits. The low three are the raw SWD ACK.
const (
	dapTransferOK    = 1 << 0
	dapTransferWAIT  = 1 << 1
	dapTransferFAULT = 1 << 2
	dapTransferERROR = 1 << 3
)

// checkPin rejects a pin number that is not on the header.
func checkPin(name string, pin int) error {
	if pin < 0 || pin > 31 {
		return fmt.Errorf("bcm2835: %s pin %d is outside GPIO0..31", name, pin)
	}
	return nil
}

// checkDistinct rejects a wiring that gives one pin two jobs.
func checkDistinct(pins map[string]int) error {
	seen := make(map[int]string, len(pins))
	for _, name := range sortedKeys(pins) {
		pin := pins[name]
		if other, dup := seen[pin]; dup {
			return fmt.Errorf("bcm2835: %s and %s cannot both be pin %d", other, name, pin)
		}
		seen[pin] = name
	}
	return nil
}

func sortedKeys(m map[string]int) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	sort.Strings(out)
	return out
}

// ---------------- Register access ----------------

// gpioRegs is the raw mmap, shared by every pin.
type gpioRegs struct {
	mem []byte
}

func openRegs() (*gpioRegs, error) {
	f, err := os.OpenFile("/dev/gpiomem", os.O_RDWR|os.O_SYNC, 0)
	if err != nil {
		// os.OpenFile's error already reads "open /dev/gpiomem: ...", so only
		// the part it cannot know is added here. Mmap's below is a bare errno
		// and does need saying what failed.
		return nil, fmt.Errorf("%w (need root, or the gpio group, "+
			"on a Pi 1-4 / CM1, CM3, CM4 -- a Pi 5 or CM5 exposes /dev/gpiomem0..4 with "+
			"an incompatible RP1 layout, not /dev/gpiomem)", err)
	}
	defer f.Close()

	mem, err := syscall.Mmap(int(f.Fd()), 0, blockSize,
		syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)
	if err != nil {
		return nil, fmt.Errorf("mmap /dev/gpiomem: %w", err)
	}
	return &gpioRegs{mem: mem}, nil
}

func (g *gpioRegs) reg(byteOffset uintptr) *uint32 {
	return (*uint32)(unsafe.Pointer(&g.mem[byteOffset]))
}

// pin is one GPIO resolved to the exact registers and bit positions that drive
// and read it.
//
// Resolving it once, at open, rather than deriving the register from a pin
// number on every edge is worth saying something honest about, because the
// obvious assumption is wrong. It is NOT a speed optimisation: BenchmarkEdge
// and the pair of resolve-versus-compute benchmarks beside it put the whole
// address computation -- divide, modulo, shift and the slice bounds check --
// at well under a nanosecond, which is noise next to the barrier below.
//
// What dominates an edge is that barrier, by roughly ten to one, and it cannot
// go: see [pin.high]. So the reason this type exists is that it reads better
// and groups a pin's four registers with the masks that go with them, and the
// reason to look elsewhere for speed is that there is none to be had here.
type pin struct {
	set  *uint32 // GPSET0 bank: write the mask to drive high
	clr  *uint32 // GPCLR0 bank: write the mask to drive low
	lev  *uint32 // GPLEV0 bank: read the levels
	fsel *uint32 // GPFSEL bank: three function-select bits per pin

	mask   uint32 // this pin's bit in set, clr and lev
	shift  uint   // where that bit sits, for reading lev
	fshift uint   // where its function field sits within fsel

	num uint // the BCM pin number, for messages and the tests
}

// pin resolves a pin number against the mapped register block.
func (g *gpioRegs) pin(n uint) pin {
	return pin{
		set:    g.reg(gpset0 + uintptr(n/32)*4),
		clr:    g.reg(gpclr0 + uintptr(n/32)*4),
		lev:    g.reg(gplev0 + uintptr(n/32)*4),
		fsel:   g.reg(gpfsel0 + uintptr(n/10)*4),
		mask:   1 << (n % 32),
		shift:  n % 32,
		fshift: (n % 10) * 3,
		num:    n,
	}
}

// high and low write a set-or-clear register. These are write-only masks, not
// read-modify-write fields: a one acts on that pin and a zero leaves it alone,
// so there is nothing to preserve and nothing to read back first.
//
// The store is atomic for the reason gpio_regs.h puts a __sync_synchronize()
// after every register write: without a barrier the CPU's write buffer can
// merge two stores to the same address into one and lose a clock edge. The
// speed coefficients [swd.Calibrator] measures assume those barriers are in
// place.
func (p *pin) high() { atomic.StoreUint32(p.set, p.mask) }
func (p *pin) low()  { atomic.StoreUint32(p.clr, p.mask) }

// drive puts the pin at the level in the low bit of `bit`.
func (p *pin) drive(bit uint32) {
	if bit&1 != 0 {
		atomic.StoreUint32(p.set, p.mask)
	} else {
		atomic.StoreUint32(p.clr, p.mask)
	}
}

func (p *pin) level() uint32 {
	return (atomic.LoadUint32(p.lev) >> p.shift) & 1
}

func (p *pin) setFunction(fn uint32) {
	v := atomic.LoadUint32(p.fsel)
	atomic.StoreUint32(p.fsel, (v&^(7<<p.fshift))|((fn&7)<<p.fshift))
}

func (p *pin) input()  { p.setFunction(funcInput) }
func (p *pin) output() { p.setFunction(funcOutput) }

// function reads a pin's current function select back by number, which is only
// of interest to the tests: the driver itself only ever writes it, and reaches
// its own pins through the resolved [pin] rather than by number.
func (g *gpioRegs) function(n uint) uint32 {
	return (atomic.LoadUint32(g.reg(gpfsel0+uintptr(n/10)*4)) >> ((n % 10) * 3)) & 0x7
}

func (g *gpioRegs) close() error {
	if g.mem == nil {
		return nil
	}
	mem := g.mem
	g.mem = nil
	return syscall.Munmap(mem)
}

// ---------------- core ----------------

// core is everything the wiring does not change: the pins that all three
// wirings have, the calibrated delay loop, and the bit and sequence primitives
// built on them.
//
// The wiring types below embed it by value. That is a plain struct embed, not
// an interface, so every call through it stays a direct, inlinable one.
type core struct {
	regs *gpioRegs

	// The pins, resolved once. clk and dout are driven and din is read; on a
	// single-pin wiring din and dout are the same GPIO.
	clk, din, dout pin

	idleCycles uint8
	turnaround uint8
	dataPhase  bool

	// The delay loop is paced by a pre-measured iteration count rather than by
	// polling a clock, following this repo's own C++ gateway (DAP.h's
	// SPEED_COEFF/SPEED_OFFSET, PIN_DELAY_SLOW() in DAP.cpp). The measured
	// constants there are specific to that C++ loop's compiled code, so they
	// cannot be copied in; measure() works out the equivalent numbers for
	// whatever this Go binary's loop actually costs on this CPU.
	loopNs     float64 // cost of one delayLoop() iteration
	toggleNs   float64 // cost of one bare clock edge, the register write alone
	iterations uint32
	clockHz    uint32

	sink uint32 // written every delayLoop() iteration; keeps the compiler from eliding the loop
}

// newCore maps the registers and floats the pins. It deliberately does not
// measure the delay loop; see Calibrating in the package comment.
func newCore(swclk, swdioIn, swdioOut int) (core, error) {
	regs, err := openRegs()
	if err != nil {
		return core{}, err
	}
	return makeCore(regs, swclk, swdioIn, swdioOut), nil
}

// makeCore is newCore over an already-mapped register block. The tests stand a
// driver up over a plain heap buffer through this, so that what they exercise
// is the same pin resolution production uses rather than a hand-built one.
func makeCore(regs *gpioRegs, swclk, swdioIn, swdioOut int) core {
	c := core{
		regs:       regs,
		clk:        regs.pin(uint(swclk)),
		din:        regs.pin(uint(swdioIn)),
		dout:       regs.pin(uint(swdioOut)),
		turnaround: 1,
	}

	c.clk.input()
	c.din.input()
	c.dout.input()

	// Remember a default clock, so that a later Calibrate has something sane to
	// apply even if the caller never names one.
	c.SetFrequencyHz(1_200_000)
	return c
}

// Close unmaps the register block.
//
// The resolved pins hold pointers into that mapping, so they go with it: a
// store through one afterwards would be a write into memory this process no
// longer owns. Cleared, they fail as a nil dereference instead, which is the
// version of that mistake you get told about.
func (c *core) Close() error {
	c.clk, c.din, c.dout = pin{}, pin{}, pin{}
	return c.regs.close()
}

// ---------------- Pin access ----------------

func (c *core) SwclkIn() uint8 { return uint8(c.clk.level()) }
func (c *core) SwclkSet()      { c.clk.high() }
func (c *core) SwclkClr()      { c.clk.low() }
func (c *core) SwdioIn() uint8 { return uint8(c.din.level()) }
func (c *core) SwdioSet()      { c.dout.high() }
func (c *core) SwdioClr()      { c.dout.low() }

func (c *core) pinSWDIOOut(bit uint32) { c.dout.drive(bit) }

// ---------------- Shared primitives ----------------
//
// From here to the wiring types below, nothing touches a register directly: it
// is all the pin accessors above, which is why it can live on the shared core
// rather than being copied per wiring the way SwdTransfer has to be.

func (c *core) SwdConfigure(turnaround uint8, dataPhase bool) {
	c.turnaround = turnaround
	c.dataPhase = dataPhase
}

// ---------------- Clock pacing ----------------

// delayLoop is PIN_DELAY_SLOW() in DAP.cpp, ported: a plain counted loop, no
// timer of any kind inside it. c.sink stops the Go compiler recognizing the
// loop as dead code and deleting it, the same job DAP.cpp's
// __asm__ __volatile__("") does for gcc and clang.
func (c *core) delayLoop(n uint32) {
	var x uint32
	for i := uint32(0); i < n; i++ {
		x++
	}
	c.sink = x
}

// measure times what one delay-loop iteration and one bare clock edge cost on
// this CPU, by driving exactly the calls the real edges drive.
func (c *core) measure() {
	const loopIterations = 20_000_000
	start := time.Now()
	c.delayLoop(loopIterations)
	c.loopNs = time.Since(start).Seconds() * 1e9 / float64(loopIterations)

	const togglePairs = 500_000
	start = time.Now()
	for i := 0; i < togglePairs; i++ {
		c.SwclkClr()
		c.SwclkSet()
	}
	// Two edges per iteration, clear then set.
	c.toggleNs = time.Since(start).Seconds() * 1e9 / float64(togglePairs) / 2
}

// SetFrequencyHz sets the SWCLK half period as a count of loop iterations:
// whatever the half period is, less the edge's own fixed cost.
func (c *core) SetFrequencyHz(hz uint32) {
	if hz == 0 {
		return
	}
	c.clockHz = hz
	if c.loopNs <= 0 {
		return
	}

	remaining := 1e9/float64(hz)/2 - c.toggleNs
	if remaining < 0 {
		// The requested clock is at or above what bare register writes can
		// reach; drive them back to back and accept whatever that gives.
		remaining = 0
	}
	c.iterations = uint32(remaining / c.loopNs)
}

// Calibrate re-measures and applies the result at the clock currently in
// effect. See [swc.Calibrator].
func (c *core) Calibrate() (coeff, offset uint32, err error) {
	c.measure()
	if c.loopNs <= 0 {
		// A clock that did not advance, or a loop optimised away underneath
		// us; either way there is nothing sane to divide by.
		return 0, 0, fmt.Errorf("bcm2835: delay loop measured as taking no time")
	}
	c.SetFrequencyHz(c.clockHz)
	coeff, offset = c.SpeedCoeffs()
	return coeff, offset, nil
}

// SpeedCoeffs converts the measured nanosecond costs into the speed_coeff and
// speed_offset pair. One iteration costs 500000/coeff ns, so coeff is
// 500000/iterationCost, and offset is the edge cost in iteration units.
func (c *core) SpeedCoeffs() (coeff, offset uint32) {
	if c.loopNs <= 0 {
		return 0, 0
	}
	return clampCoeffs(
		int64(500000.0/c.loopNs+0.5),
		int64(c.toggleNs/c.loopNs+0.5),
	)
}

// SetSpeedCoeffs goes the other way: recover the nanosecond costs the loop is
// actually paced by from values measured earlier.
func (c *core) SetSpeedCoeffs(coeff, offset uint32) error {
	if coeff == 0 {
		return fmt.Errorf("bcm2835: speed coefficient must not be zero")
	}
	if offset == 0 || offset >= coeff {
		return fmt.Errorf("bcm2835: speed offset must be at least 1 and below the coefficient %d, got %d",
			coeff, offset)
	}
	c.loopNs = 500000.0 / float64(coeff)
	c.toggleNs = float64(offset) * c.loopNs
	c.SetFrequencyHz(c.clockHz)
	return nil
}

func clampCoeffs(coeff, offset int64) (uint32, uint32) {
	if coeff < 1 {
		coeff = 1
	}
	if offset < 1 {
		// Zero would claim an unbounded maximum clock.
		offset = 1
	}
	if offset >= coeff {
		offset = coeff - 1
	}
	if offset < 1 {
		offset = 1
	}
	return uint32(coeff), uint32(offset)
}

func (c *core) pinDelay() { c.delayLoop(c.iterations) }

func (c *core) swClockCycle() {
	c.SwclkClr()
	c.pinDelay()
	c.SwclkSet()
	c.pinDelay()
}

func (c *core) swWriteBit(bit uint32) {
	c.pinSWDIOOut(bit)
	c.SwclkClr()
	c.pinDelay()
	c.SwclkSet()
	c.pinDelay()
}

func (c *core) swReadBit() uint32 {
	c.SwclkClr()
	c.pinDelay()
	bit := c.SwdioIn()
	c.SwclkSet()
	c.pinDelay()
	return uint32(bit)
}

// ---------------- SWD wire protocol ----------------

func (c *core) SwjSequence(bitCount uint32, data []byte) {
	var val uint32
	var n uint32

	idx := 0
	for bitCount > 0 {
		if n == 0 {
			val = uint32(data[idx])
			idx++
			n = 8
		}
		if val&1 != 0 {
			c.SwdioSet()
		} else {
			c.SwdioClr()
		}
		c.swClockCycle()
		val >>= 1
		n--
		bitCount--
	}
}

func (c *core) SwdWriteSequence(bitCount uint32, data []byte) {
	inIdx := 0
	for bitCount > 0 {
		val := uint32(data[inIdx])
		inIdx++

		k := uint32(8)
		for k > 0 && bitCount > 0 {
			c.swWriteBit(val & 1)
			val >>= 1
			k--
			bitCount--
		}
	}
}

func (c *core) SwdReadSequence(bitCount uint32, data []byte) {
	outIdx := 0
	for bitCount > 0 {
		var val uint32
		k := uint32(8)

		for k > 0 && bitCount > 0 {
			bit := c.swReadBit()
			val >>= 1
			val |= bit << 7
			k--
			bitCount--
		}

		val >>= k
		data[outIdx] = byte(val)
		outIdx++
	}
}

// ---------------- DriverBidir ----------------

// DriverBidir drives SWD over a single bidirectional SWDIO pin wired straight
// to the target, with no buffer: the GPIO itself is flipped between input and
// output to turn the bus around.
//
// There is no direction pin and no branch anywhere below deciding whether there
// is one -- that is the whole reason this is its own type rather than a flag on
// a shared one. A single data pin that does sit behind a buffer wants
// [DriverBidirBuffered].
type DriverBidir struct {
	core
}

var (
	_ swd.BitBanger  = (*DriverBidir)(nil)
	_ swd.Calibrator = (*DriverBidir)(nil)
)

// OpenBidir claims the pins. They are left as inputs until PortOn, and the
// delay loop is not measured -- call Calibrate or SetSpeedCoeffs before setting a
// clock. See Calibrating in the package comment.
func OpenBidir(swclk, swdio int) (*DriverBidir, error) {
	if err := checkPin("swclk", swclk); err != nil {
		return nil, err
	}
	if err := checkPin("swdio", swdio); err != nil {
		return nil, err
	}
	if swclk == swdio {
		return nil, fmt.Errorf("bcm2835: swclk and swdio cannot be the same pin (%d)", swclk)
	}

	c, err := newCore(swclk, swdio, swdio)
	if err != nil {
		return nil, err
	}
	d := &DriverBidir{core: c}
	return d, nil
}

func (d *DriverBidir) PortOn() {
	// Clock is output, idle low.
	d.clk.output()
	d.clk.low()

	// One pin carries both directions: drive it high, the SWD idle level.
	d.dout.output()
	d.dout.high()
}

func (d *DriverBidir) PortOff() {
	d.clk.input()
	d.dout.input()
}

// SwdioOutEnable hands SWDIO to the host, SwdioOutDisable to the target. With
// one unbuffered pin, that is the GPIO's own direction and nothing else.
func (d *DriverBidir) SwdioOutEnable()  { d.dout.output() }
func (d *DriverBidir) SwdioOutDisable() { d.din.input() }

func (d *DriverBidir) SwdTransfer(req uint8, data *uint32, requestTimestamp func()) uint8 {
	request := uint32(req)
	var ack uint32
	var bit uint32
	var val uint32
	var parity uint32

	// Request
	parity = 0
	d.swWriteBit(1)
	bit = (request >> 0) & 1
	d.swWriteBit(bit)
	parity += bit

	bit = (request >> 1) & 1
	d.swWriteBit(bit)
	parity += bit

	bit = (request >> 2) & 1
	d.swWriteBit(bit)
	parity += bit

	bit = (request >> 3) & 1
	d.swWriteBit(bit)
	parity += bit

	d.swWriteBit(parity)
	d.swWriteBit(0)
	d.swWriteBit(1)

	// Turnaround
	d.SwdioOutDisable()
	for n := uint32(d.turnaround); n > 0; n-- {
		d.swClockCycle()
	}

	// ACK
	bit = d.swReadBit()
	ack = bit << 0
	bit = d.swReadBit()
	ack |= bit << 1
	bit = d.swReadBit()
	ack |= bit << 2

	if ack == dapTransferOK {
		if request&dapTransferRnW != 0 {
			val = 0
			parity = 0
			for n := uint32(32); n > 0; n-- {
				bit = d.swReadBit()
				parity += bit
				val >>= 1
				val |= bit << 31
			}
			bit = d.swReadBit()
			if ((parity ^ bit) & 1) != 0 {
				ack = dapTransferERROR
			}
			if data != nil {
				*data = val
			}
			for n := uint32(d.turnaround); n > 0; n-- {
				d.swClockCycle()
			}
			d.SwdioOutEnable()
		} else {
			for n := uint32(d.turnaround); n > 0; n-- {
				d.swClockCycle()
			}
			d.SwdioOutEnable()

			val = *data
			parity = 0
			for n := uint32(32); n > 0; n-- {
				bit = val & 1
				d.swWriteBit(bit)
				parity += bit
				val >>= 1
			}
			d.swWriteBit(parity)
		}

		if request&dapTransferTIMESTAMP != 0 && requestTimestamp != nil {
			requestTimestamp()
		}

		n := uint32(d.idleCycles)
		if n != 0 {
			d.pinSWDIOOut(0)
			for ; n > 0; n-- {
				d.swClockCycle()
			}
		}
		d.pinSWDIOOut(1)
		return uint8(ack)
	}

	if ack == dapTransferWAIT || ack == dapTransferFAULT {
		if d.dataPhase && (request&dapTransferRnW) != 0 {
			for n := uint32(33); n > 0; n-- {
				d.swClockCycle()
			}
		}
		for n := uint32(d.turnaround); n > 0; n-- {
			d.swClockCycle()
		}
		d.SwdioOutEnable()
		if d.dataPhase && (request&dapTransferRnW) == 0 {
			d.pinSWDIOOut(0)
			for n := uint32(33); n > 0; n-- {
				d.swClockCycle()
			}
		}
		d.pinSWDIOOut(1)
		return uint8(ack)
	}

	for n := uint32(d.turnaround + 33); n > 0; n-- {
		d.swClockCycle()
	}
	d.SwdioOutEnable()
	d.pinSWDIOOut(1)
	return uint8(ack)
}

// ---------------- DriverBidirBuffered ----------------

// DriverBidirBuffered drives SWD over a single bidirectional SWDIO pin that
// sits behind a buffer with its own direction line. It is OpenOCD's
// "adapter gpio swdio" plus "adapter gpio swdio_dir", with no swdio_in.
//
// Both have to move to turn the bus around: the GPIO because there is only one
// data pin, and the buffer because there is one of those too. The order is
// not arbitrary -- whichever side is about to stop driving goes passive
// first, so the host pin and the buffer output are
// never both driving the wire.
type DriverBidirBuffered struct {
	core
	dir pin
}

var (
	_ swd.BitBanger  = (*DriverBidirBuffered)(nil)
	_ swd.Calibrator = (*DriverBidirBuffered)(nil)
)

// OpenBidirBuffered claims the pins. They are left as inputs until PortOn, and the
// delay loop is not measured -- call Calibrate or SetSpeedCoeffs before setting a
// clock. See Calibrating in the package comment.
func OpenBidirBuffered(swclk, swdio, swdioDir int) (*DriverBidirBuffered, error) {
	if err := checkPin("swclk", swclk); err != nil {
		return nil, err
	}
	if err := checkPin("swdio", swdio); err != nil {
		return nil, err
	}
	if err := checkPin("swdioDir", swdioDir); err != nil {
		return nil, err
	}
	if err := checkDistinct(map[string]int{"swclk": swclk, "swdio": swdio, "swdioDir": swdioDir}); err != nil {
		return nil, err
	}

	c, err := newCore(swclk, swdio, swdio)
	if err != nil {
		return nil, err
	}
	d := &DriverBidirBuffered{core: c}
	d.dir = c.regs.pin(uint(swdioDir))
	d.dir.input()
	return d, nil
}

func (d *DriverBidirBuffered) PortOn() {
	// Clock is output, idle low.
	d.clk.output()
	d.clk.low()

	// One pin carries both directions: drive it high, the SWD idle level.
	d.dout.output()
	d.dout.high()

	// Direction pin: low = host drives target, high = host reads target.
	d.dir.output()
	d.dir.low()
}

func (d *DriverBidirBuffered) PortOff() {
	d.clk.input()
	d.dout.input()
	d.dir.input()
}

// SwdioOutEnable hands SWDIO to the host, SwdioOutDisable to the target.
//
// Both the pin and the buffer move, and the order matters: point the buffer
// away from whichever side is about to stop driving before it does, so the host
// pin and the buffer output are never both driving the wire.
func (d *DriverBidirBuffered) SwdioOutEnable() {
	d.dir.low()
	d.dout.output()
}

func (d *DriverBidirBuffered) SwdioOutDisable() {
	d.din.input()
	d.dir.high()
}

// Close clears the direction pin along with the core's, for the reason
// [core.Close] gives.
func (d *DriverBidirBuffered) Close() error {
	d.dir = pin{}
	return d.core.Close()
}

func (d *DriverBidirBuffered) SwdTransfer(req uint8, data *uint32, requestTimestamp func()) uint8 {
	request := uint32(req)
	var ack uint32
	var bit uint32
	var val uint32
	var parity uint32

	// Request
	parity = 0
	d.swWriteBit(1)
	bit = (request >> 0) & 1
	d.swWriteBit(bit)
	parity += bit

	bit = (request >> 1) & 1
	d.swWriteBit(bit)
	parity += bit

	bit = (request >> 2) & 1
	d.swWriteBit(bit)
	parity += bit

	bit = (request >> 3) & 1
	d.swWriteBit(bit)
	parity += bit

	d.swWriteBit(parity)
	d.swWriteBit(0)
	d.swWriteBit(1)

	// Turnaround
	d.SwdioOutDisable()
	for n := uint32(d.turnaround); n > 0; n-- {
		d.swClockCycle()
	}

	// ACK
	bit = d.swReadBit()
	ack = bit << 0
	bit = d.swReadBit()
	ack |= bit << 1
	bit = d.swReadBit()
	ack |= bit << 2

	if ack == dapTransferOK {
		if request&dapTransferRnW != 0 {
			val = 0
			parity = 0
			for n := uint32(32); n > 0; n-- {
				bit = d.swReadBit()
				parity += bit
				val >>= 1
				val |= bit << 31
			}
			bit = d.swReadBit()
			if ((parity ^ bit) & 1) != 0 {
				ack = dapTransferERROR
			}
			if data != nil {
				*data = val
			}
			for n := uint32(d.turnaround); n > 0; n-- {
				d.swClockCycle()
			}
			d.SwdioOutEnable()
		} else {
			for n := uint32(d.turnaround); n > 0; n-- {
				d.swClockCycle()
			}
			d.SwdioOutEnable()

			val = *data
			parity = 0
			for n := uint32(32); n > 0; n-- {
				bit = val & 1
				d.swWriteBit(bit)
				parity += bit
				val >>= 1
			}
			d.swWriteBit(parity)
		}

		if request&dapTransferTIMESTAMP != 0 && requestTimestamp != nil {
			requestTimestamp()
		}

		n := uint32(d.idleCycles)
		if n != 0 {
			d.pinSWDIOOut(0)
			for ; n > 0; n-- {
				d.swClockCycle()
			}
		}
		d.pinSWDIOOut(1)
		return uint8(ack)
	}

	if ack == dapTransferWAIT || ack == dapTransferFAULT {
		if d.dataPhase && (request&dapTransferRnW) != 0 {
			for n := uint32(33); n > 0; n-- {
				d.swClockCycle()
			}
		}
		for n := uint32(d.turnaround); n > 0; n-- {
			d.swClockCycle()
		}
		d.SwdioOutEnable()
		if d.dataPhase && (request&dapTransferRnW) == 0 {
			d.pinSWDIOOut(0)
			for n := uint32(33); n > 0; n-- {
				d.swClockCycle()
			}
		}
		d.pinSWDIOOut(1)
		return uint8(ack)
	}

	for n := uint32(d.turnaround + 33); n > 0; n-- {
		d.swClockCycle()
	}
	d.SwdioOutEnable()
	d.pinSWDIOOut(1)
	return uint8(ack)
}

// ---------------- DriverSplit ----------------

// DriverSplit drives SWD over separate input and output pins behind a buffer,
// with a direction line to turn it around. This is the wiring most boards with
// a level shifter use.
//
// Only the buffer turns round: the data pins keep their directions for the life
// of the session, so a turnaround is one register write.
type DriverSplit struct {
	core
	dir pin
}

var (
	_ swd.BitBanger  = (*DriverSplit)(nil)
	_ swd.Calibrator = (*DriverSplit)(nil)
)

// OpenSplit claims the pins. They are left as inputs until PortOn, and the
// delay loop is not measured -- call Calibrate or SetSpeedCoeffs before setting a
// clock. See Calibrating in the package comment.
func OpenSplit(swclk, swdioIn, swdioOut, swdioDir int) (*DriverSplit, error) {
	for name, pin := range map[string]int{
		"swclk": swclk, "swdioIn": swdioIn, "swdioOut": swdioOut, "swdioDir": swdioDir,
	} {
		if err := checkPin(name, pin); err != nil {
			return nil, err
		}
	}
	if err := checkDistinct(map[string]int{
		"swclk": swclk, "swdioIn": swdioIn, "swdioOut": swdioOut, "swdioDir": swdioDir,
	}); err != nil {
		return nil, err
	}

	c, err := newCore(swclk, swdioIn, swdioOut)
	if err != nil {
		return nil, err
	}
	d := &DriverSplit{core: c}
	d.dir = c.regs.pin(uint(swdioDir))
	d.dir.input()
	return d, nil
}

func (d *DriverSplit) PortOn() {
	// Clock is output, idle low.
	d.clk.output()
	d.clk.low()

	// SWDIO starts low here where the bidirectional wirings start it high.
	// Nothing turns on it: whatever a caller does first begins with a line
	// reset, which is fifty clocks of SWDIO high, and SwdTransfer leaves the
	// line high on the way out from then on. Left as it is rather than made
	// uniform because the difference is only observable on a scope, and that
	// is not a claim to make from a test bench without one.
	d.dout.output()
	d.dout.low()
	d.din.input()

	// Direction pin: low = host drives target, high = host reads target.
	d.dir.output()
	d.dir.low()
}

func (d *DriverSplit) PortOff() {
	d.clk.input()
	d.din.input()
	d.dout.input()
	d.dir.input()
}

// SwdioOutEnable hands SWDIO to the host, SwdioOutDisable to the target. The
// data pins never move: turning the buffer round is the whole of it, which is
// one register write.
func (d *DriverSplit) SwdioOutEnable()  { d.dir.low() }
func (d *DriverSplit) SwdioOutDisable() { d.dir.high() }

// Close clears the direction pin along with the core's, for the reason
// [core.Close] gives.
func (d *DriverSplit) Close() error {
	d.dir = pin{}
	return d.core.Close()
}

func (d *DriverSplit) SwdTransfer(req uint8, data *uint32, requestTimestamp func()) uint8 {
	request := uint32(req)
	var ack uint32
	var bit uint32
	var val uint32
	var parity uint32

	// Request
	parity = 0
	d.swWriteBit(1)
	bit = (request >> 0) & 1
	d.swWriteBit(bit)
	parity += bit

	bit = (request >> 1) & 1
	d.swWriteBit(bit)
	parity += bit

	bit = (request >> 2) & 1
	d.swWriteBit(bit)
	parity += bit

	bit = (request >> 3) & 1
	d.swWriteBit(bit)
	parity += bit

	d.swWriteBit(parity)
	d.swWriteBit(0)
	d.swWriteBit(1)

	// Turnaround
	d.SwdioOutDisable()
	for n := uint32(d.turnaround); n > 0; n-- {
		d.swClockCycle()
	}

	// ACK
	bit = d.swReadBit()
	ack = bit << 0
	bit = d.swReadBit()
	ack |= bit << 1
	bit = d.swReadBit()
	ack |= bit << 2

	if ack == dapTransferOK {
		if request&dapTransferRnW != 0 {
			val = 0
			parity = 0
			for n := uint32(32); n > 0; n-- {
				bit = d.swReadBit()
				parity += bit
				val >>= 1
				val |= bit << 31
			}
			bit = d.swReadBit()
			if ((parity ^ bit) & 1) != 0 {
				ack = dapTransferERROR
			}
			if data != nil {
				*data = val
			}
			for n := uint32(d.turnaround); n > 0; n-- {
				d.swClockCycle()
			}
			d.SwdioOutEnable()
		} else {
			for n := uint32(d.turnaround); n > 0; n-- {
				d.swClockCycle()
			}
			d.SwdioOutEnable()

			val = *data
			parity = 0
			for n := uint32(32); n > 0; n-- {
				bit = val & 1
				d.swWriteBit(bit)
				parity += bit
				val >>= 1
			}
			d.swWriteBit(parity)
		}

		if request&dapTransferTIMESTAMP != 0 && requestTimestamp != nil {
			requestTimestamp()
		}

		n := uint32(d.idleCycles)
		if n != 0 {
			d.pinSWDIOOut(0)
			for ; n > 0; n-- {
				d.swClockCycle()
			}
		}
		d.pinSWDIOOut(1)
		return uint8(ack)
	}

	if ack == dapTransferWAIT || ack == dapTransferFAULT {
		if d.dataPhase && (request&dapTransferRnW) != 0 {
			for n := uint32(33); n > 0; n-- {
				d.swClockCycle()
			}
		}
		for n := uint32(d.turnaround); n > 0; n-- {
			d.swClockCycle()
		}
		d.SwdioOutEnable()
		if d.dataPhase && (request&dapTransferRnW) == 0 {
			d.pinSWDIOOut(0)
			for n := uint32(33); n > 0; n-- {
				d.swClockCycle()
			}
		}
		d.pinSWDIOOut(1)
		return uint8(ack)
	}

	for n := uint32(d.turnaround + 33); n > 0; n-- {
		d.swClockCycle()
	}
	d.SwdioOutEnable()
	d.pinSWDIOOut(1)
	return uint8(ack)
}
