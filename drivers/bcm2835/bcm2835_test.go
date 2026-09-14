package bcm2835

import (
	"sync/atomic"
	"testing"
)

// The tests stand drivers up over a plain heap buffer instead of a real
// /dev/gpiomem mmap, so the wiring and the calibration arithmetic can be
// exercised without a Raspberry Pi. The atomic load and store in gpioRegs work
// identically against any 4-byte-aligned memory; they neither know nor care
// that it is not real device memory.

const (
	pinSWCLK    = 16
	pinSWDIOIn  = 12
	pinSWDIOOut = 20
	pinSWDIODir = 21

	// pinSWDIO is the single data pin the bidirectional wirings use.
	pinSWDIO = 20
)

// testCore stands a core up over a heap buffer through the same pin resolution
// production uses, so what the tests drive is the real thing with only the
// mapping swapped out.
func testCore(swclk, swdioIn, swdioOut uint) core {
	regs := &gpioRegs{mem: make([]byte, blockSize)}
	return makeCore(regs, int(swclk), int(swdioIn), int(swdioOut))
}

func newBidir() *DriverBidir {
	return &DriverBidir{core: testCore(pinSWCLK, pinSWDIO, pinSWDIO)}
}

func newBidirBuffered() *DriverBidirBuffered {
	c := testCore(pinSWCLK, pinSWDIO, pinSWDIO)
	return &DriverBidirBuffered{core: c, dir: c.regs.pin(pinSWDIODir)}
}

func newSplit() *DriverSplit {
	c := testCore(pinSWCLK, pinSWDIOIn, pinSWDIOOut)
	return &DriverSplit{core: c, dir: c.regs.pin(pinSWDIODir)}
}

// level is what watchPin saw a pin driven to.
type level int

const (
	untouched level = iota
	low
	high
)

func (l level) String() string {
	switch l {
	case low:
		return "low"
	case high:
		return "high"
	default:
		return "untouched"
	}
}

// watchPin reports which way act drove one pin.
//
// The fake register block is plain memory, so a GPSET0 write does not show up
// in GPLEV0 the way it would on real silicon. Clearing both the set and the
// clear register first and seeing which one comes back with the bit is what the
// hardware is actually being told, which is the thing under test.
func watchPin(c *core, pin uint, act func()) level {
	atomic.StoreUint32(c.regs.reg(gpset0), 0)
	atomic.StoreUint32(c.regs.reg(gpclr0), 0)

	act()

	mask := uint32(1) << pin
	set := atomic.LoadUint32(c.regs.reg(gpset0))&mask != 0
	clr := atomic.LoadUint32(c.regs.reg(gpclr0))&mask != 0
	switch {
	case set && !clr:
		return high
	case clr && !set:
		return low
	default:
		return untouched
	}
}

// ---------------- Constructors ----------------

func TestConstructorsRejectBadPins(t *testing.T) {
	// These all fail before anything is mapped, so they run anywhere.
	for _, c := range []struct {
		name string
		open func() error
	}{
		{"bidir: clock and data on one pin", func() error {
			_, err := OpenBidir(16, 16)
			return err
		}},
		{"bidir: pin off the end of the header", func() error {
			_, err := OpenBidir(16, 99)
			return err
		}},
		{"bidir: negative pin", func() error {
			_, err := OpenBidir(16, -3)
			return err
		}},
		{"buffered: direction pin doubling as data", func() error {
			_, err := OpenBidirBuffered(16, 20, 20)
			return err
		}},
		{"buffered: clock and data on one pin", func() error {
			_, err := OpenBidirBuffered(16, 16, 21)
			return err
		}},
		{"split: direction pin doubling as clock", func() error {
			_, err := OpenSplit(16, 12, 20, 16)
			return err
		}},
		{"split: in and out on one pin", func() error {
			_, err := OpenSplit(16, 20, 20, 21)
			return err
		}},
	} {
		if err := c.open(); err == nil {
			t.Errorf("%s: accepted, want an error", c.name)
		}
	}
}

// ---------------- Wiring ----------------

// With one bidirectional pin the GPIO itself has to be flipped; with the split
// wiring only the buffer turns round and the data pins keep their directions.
func TestDirectionSwitching(t *testing.T) {
	t.Run("bidir", func(t *testing.T) {
		d := newBidir()
		d.PortOn()

		d.SwdioOutEnable()
		if got := d.regs.function(pinSWDIO); got != funcOutput {
			t.Errorf("after OutEnable, SWDIO function = %d, want output (%d)", got, funcOutput)
		}
		d.SwdioOutDisable()
		if got := d.regs.function(pinSWDIO); got != funcInput {
			t.Errorf("after OutDisable, SWDIO function = %d, want input (%d)", got, funcInput)
		}
	})

	// Both the pin and the buffer move, and the order is what keeps the host
	// pin and the buffer output from ever both driving the wire.
	t.Run("bidir buffered", func(t *testing.T) {
		d := newBidirBuffered()
		d.PortOn()

		if drove := watchPin(&d.core, pinSWDIODir, d.SwdioOutDisable); drove != high {
			t.Errorf("the direction pin went %v; want high while the target drives", drove)
		}
		if got := d.regs.function(pinSWDIO); got != funcInput {
			t.Errorf("SWDIO function = %d, want input", got)
		}

		if drove := watchPin(&d.core, pinSWDIODir, d.SwdioOutEnable); drove != low {
			t.Errorf("the direction pin went %v; want low while the host drives", drove)
		}
		if got := d.regs.function(pinSWDIO); got != funcOutput {
			t.Errorf("SWDIO function = %d, want output", got)
		}
	})

	t.Run("split", func(t *testing.T) {
		d := newSplit()
		d.PortOn()

		if drove := watchPin(&d.core, pinSWDIODir, d.SwdioOutEnable); drove != low {
			t.Errorf("after OutEnable the direction pin went %v; want low while the host drives", drove)
		}
		if got := d.regs.function(pinSWDIOOut); got != funcOutput {
			t.Errorf("after OutEnable, the SWDIO out pin's function = %d, want output", got)
		}

		if drove := watchPin(&d.core, pinSWDIODir, d.SwdioOutDisable); drove != high {
			t.Errorf("after OutDisable the direction pin went %v; want high while the target drives", drove)
		}
		// The data pins do not move: turning the buffer round is what the
		// direction pin is for.
		if got := d.regs.function(pinSWDIOOut); got != funcOutput {
			t.Errorf("after OutDisable, the SWDIO out pin's function = %d, want it left as output", got)
		}
		if got := d.regs.function(pinSWDIOIn); got != funcInput {
			t.Errorf("after OutDisable, the SWDIO in pin's function = %d, want it left as input", got)
		}
	})
}

func TestPortOnIdlesSWDIOHighWhenBidirectional(t *testing.T) {
	// With one pin there is no buffer to hold the line, so the host has to
	// drive the SWD idle level itself.
	d := newBidir()
	if drove := watchPin(&d.core, pinSWDIO, d.PortOn); drove != high {
		t.Errorf("PortOn drove SWDIO %v, want high", drove)
	}
	if got := d.regs.function(pinSWDIO); got != funcOutput {
		t.Errorf("SWDIO function = %d after PortOn, want output", got)
	}

	d2 := newBidir()
	if drove := watchPin(&d2.core, pinSWCLK, d2.PortOn); drove != low {
		t.Errorf("PortOn drove SWCLK %v, want low", drove)
	}
}

func TestPortOffReleasesEveryPin(t *testing.T) {
	t.Run("bidir", func(t *testing.T) {
		d := newBidir()
		d.PortOn()
		d.PortOff()
		for _, pin := range []uint{pinSWCLK, pinSWDIO} {
			if got := d.regs.function(pin); got != funcInput {
				t.Errorf("pin %d function = %d after PortOff, want input", pin, got)
			}
		}
	})

	t.Run("bidir buffered", func(t *testing.T) {
		d := newBidirBuffered()
		d.PortOn()
		d.PortOff()
		for _, pin := range []uint{pinSWCLK, pinSWDIO, pinSWDIODir} {
			if got := d.regs.function(pin); got != funcInput {
				t.Errorf("pin %d function = %d after PortOff, want input", pin, got)
			}
		}
	})

	t.Run("split", func(t *testing.T) {
		d := newSplit()
		d.PortOn()
		d.PortOff()
		for _, pin := range []uint{pinSWCLK, pinSWDIOIn, pinSWDIOOut, pinSWDIODir} {
			if got := d.regs.function(pin); got != funcInput {
				t.Errorf("pin %d function = %d after PortOff, want input", pin, got)
			}
		}
	})
}

// ---------------- Clock pacing ----------------

// A driver arrives from Open uncalibrated, and has to say so rather than
// reporting numbers it never measured.
func TestUncalibratedReportsNothing(t *testing.T) {
	d := newSplit()

	if coeff, offset := d.SpeedCoeffs(); coeff != 0 || offset != 0 {
		t.Fatalf("SpeedCoeffs() = %d, %d before any measurement, want 0, 0", coeff, offset)
	}
	// A clock can still be asked for; it is remembered and applied once the
	// measurement arrives.
	d.SetFrequencyHz(2_000_000)
	if d.clockHz != 2_000_000 {
		t.Fatalf("clockHz = %d, want the requested 2000000 to be remembered", d.clockHz)
	}
	d.measure()
	d.SetFrequencyHz(d.clockHz)
	if d.iterations == 0 && d.loopNs > 0 && d.toggleNs < 1e9/2_000_000/2 {
		t.Fatal("after calibrating, a 2MHz clock still paces no delay at all")
	}
}

func TestMeasureProducesPositiveCosts(t *testing.T) {
	d := newSplit()
	d.measure()

	if d.loopNs <= 0 {
		t.Errorf("loopNs = %v, want > 0", d.loopNs)
	}
	if d.toggleNs <= 0 {
		t.Errorf("toggleNs = %v, want > 0", d.toggleNs)
	}
}

func TestFasterClockNeedsFewerIterations(t *testing.T) {
	d := newSplit()
	d.measure()

	d.SetFrequencyHz(1_000_000)
	slow := d.iterations
	d.SetFrequencyHz(4_000_000)
	fast := d.iterations

	if fast > slow {
		t.Fatalf("4MHz needs %d iterations and 1MHz needs %d; a faster clock must need fewer",
			fast, slow)
	}
}

func TestClockAboveTheEdgeCostClampsToZero(t *testing.T) {
	d := newSplit()
	// Pretend the loop and the edge are both expensive, so an unreasonably high
	// requested clock has nothing left to remove.
	d.loopNs, d.toggleNs = 1000, 1000

	d.SetFrequencyHz(1_000_000_000) // 1GHz: a 0.5ns half period
	if d.iterations != 0 {
		t.Fatalf("iterations = %d, want 0 when the requested period is below the edge cost",
			d.iterations)
	}
}

func TestDelayLoopIsNotOptimizedAway(t *testing.T) {
	d := newSplit()
	d.delayLoop(1000)
	if d.sink != 1000 {
		t.Fatalf("sink = %d after delayLoop(1000), want 1000 -- the loop body did not run", d.sink)
	}
}

// TestDelayLoopRunsBothHalves guards the driver, not just a benchmark.
//
// delayLoop's only effect is the write to c.sink, and swClockCycle calls it
// twice with the second write overwriting the first. If the compiler ever
// worked out that it could drop the first loop, every clock would come out at
// twice the requested frequency and nothing else here would notice.
func TestDelayLoopRunsBothHalves(t *testing.T) {
	d := newSplit()

	const n = 1 << 16
	d.iterations = n
	d.delayLoop(d.iterations)
	if d.sink != n {
		t.Fatalf("delayLoop(%d) left sink=%d, want %d -- the loop did not run to completion", n, d.sink, n)
	}

	d.sink = 0
	d.swClockCycle()
	if d.sink != n {
		t.Fatalf("after one clock cycle sink=%d, want %d -- a half was elided", d.sink, n)
	}
}

// The calibration is reported as speed_coeff/speed_offset and setting it back
// has to land on the same delay. That round trip is what a client of the
// Calibrate vendor command relies on when it persists the numbers and passes
// them back on the next run.
func TestSpeedCoeffRoundTrip(t *testing.T) {
	d := newSplit()
	d.measure()
	d.SetFrequencyHz(1_000_000)

	coeff, offset := d.SpeedCoeffs()
	if coeff == 0 || offset == 0 {
		t.Fatalf("SpeedCoeffs() = %d, %d, want both non-zero", coeff, offset)
	}
	if offset >= coeff {
		t.Fatalf("offset %d >= coeff %d, which claims an unbounded maximum clock", offset, coeff)
	}
	want := d.iterations

	other := newSplit()
	other.SetFrequencyHz(1_000_000)
	if err := other.SetSpeedCoeffs(coeff, offset); err != nil {
		t.Fatalf("SetSpeedCoeffs(%d, %d): %v", coeff, offset, err)
	}

	// Both forms round to integers, so allow a hair of drift rather than
	// demanding they land on exactly the same count.
	if diff := int(other.iterations) - int(want); diff > 2 || diff < -2 {
		t.Fatalf("round trip gave %d delay iterations, want about %d", other.iterations, want)
	}
}

func TestSetSpeedCoeffsRejectsNonsense(t *testing.T) {
	d := newSplit()

	for _, c := range []struct {
		name          string
		coeff, offset uint32
	}{
		{"zero coefficient", 0, 1},
		{"zero offset", 1000, 0},
		{"offset at the coefficient", 1000, 1000},
		{"offset above the coefficient", 1000, 2000},
	} {
		if err := d.SetSpeedCoeffs(c.coeff, c.offset); err == nil {
			t.Errorf("%s: SetSpeedCoeffs(%d, %d) was accepted", c.name, c.coeff, c.offset)
		}
	}
}

// ---------------- The three copies of SwdTransfer ----------------

// The three wiring types share these, so that a test asserting something of all
// of them does not have to spell each out.
//
// They deliberately do not compare the three copies of SwdTransfer against each
// other. This package's fake register block cannot see enough for that to mean
// much: GPSET0 and GPCLR0 are write-only masks it only remembers the last write
// to, so clock edges are not countable, and nothing loops back into GPLEV0, so
// only the "no valid ACK" path runs. drivers/linuxgpiod records every ioctl and
// so can count edges; that is where the cross-wiring comparison lives.
func eachWiring() []struct {
	name  string
	drive func() (swd swdTransferrer, inputPin uint)
} {
	return []struct {
		name  string
		drive func() (swd swdTransferrer, inputPin uint)
	}{
		{"bidir", func() (swdTransferrer, uint) { return newBidir(), pinSWDIO }},
		{"bidir buffered", func() (swdTransferrer, uint) { return newBidirBuffered(), pinSWDIO }},
		{"split", func() (swdTransferrer, uint) { return newSplit(), pinSWDIOIn }},
	}
}

// swdTransferrer is the part of a wiring these tests drive. The driver types
// never reach each other through an interface -- that is the whole point of
// there being three of them -- but a test comparing them has to.
type swdTransferrer interface {
	PortOn()
	SwdConfigure(turnaround uint8, dataPhase bool)
	SwdTransfer(req uint8, data *uint32, requestTimestamp func()) uint8
}

// Whatever happened, the line is handed back to the host driving it high --
// the SWD idle level, and where the next transfer expects to start.
func TestEveryWiringLeavesSWDIOIdleHigh(t *testing.T) {
	for _, w := range eachWiring() {
		t.Run(w.name, func(t *testing.T) {
			d, _ := w.drive()
			d.PortOn()

			core := coreOf(d)
			atomic.StoreUint32(core.regs.reg(gpset0), 0)
			atomic.StoreUint32(core.regs.reg(gpclr0), 0)

			var data uint32
			d.SwdTransfer(0x8D, &data, nil)

			set := atomic.LoadUint32(core.regs.reg(gpset0))
			if set&(1<<pinSWDIOOut) == 0 && set&(1<<pinSWDIO) == 0 {
				t.Error("SWDIO was not left driven high")
			}
		})
	}
}

// coreOf reaches the embedded core, which is where the register block lives.
func coreOf(d swdTransferrer) *core {
	switch v := d.(type) {
	case *DriverBidir:
		return &v.core
	case *DriverBidirBuffered:
		return &v.core
	case *DriverSplit:
		return &v.core
	}
	panic("unknown wiring")
}

// ---------------- Shared sequence primitives ----------------

// These live on core, in one copy, so there is no drift to catch -- but the bit
// packing in SwdReadSequence is fiddly enough to be worth pinning down. A run
// that is not a whole number of bytes has to end up right-aligned in the last
// byte, which is what the trailing shift is for.
//
// There is no matching test of the write side's bit order, and the fake is why:
// GPSET0 and GPCLR0 are write-only masks, so the block only remembers the last
// write to each. Every routine here ends on a clock edge, which overwrites
// whatever the data pin put there. That leaves only what a routine leaves
// behind observable, which is what the tests above check and all the write side
// would have to offer.
func TestReadSequencePacksPartialBytes(t *testing.T) {
	for _, tc := range []struct {
		bits uint32
		want []byte
	}{
		{1, []byte{0x01}},
		{3, []byte{0x07}},
		{8, []byte{0xFF}},
		{9, []byte{0xFF, 0x01}},
		{12, []byte{0xFF, 0x0F}},
		{16, []byte{0xFF, 0xFF}},
	} {
		c := testCore(pinSWCLK, pinSWDIO, pinSWDIO)
		// Every bit read comes back set.
		atomic.StoreUint32(c.regs.reg(gplev0), 1<<pinSWDIO)

		got := make([]byte, len(tc.want))
		c.SwdReadSequence(tc.bits, got)

		if !bytesEqual(got, tc.want) {
			t.Errorf("%d bits of ones packed to %x, want %x", tc.bits, got, tc.want)
		}
	}
}

func bytesEqual(a, b []byte) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}
