//go:build linux

// Package linuxgpiod bit-bangs SWD through the Linux GPIO character device --
// the kernel interface libgpiod speaks.
//
// # Which boards
//
// Any of them. That is the entire point of it. There is no register layout in
// this package and no board-specific address anywhere: it asks the kernel for
// lines by number on a named /dev/gpiochipN and drives them, so it works on a
// Pi 5 (RP1 across PCIe), on a Pi 1-4 (BCM283x/BCM2711), and on hardware that
// is not a Raspberry Pi at all.
//
// [github.com/kvth/swd-to-go/drivers/bcm2835] is the other half of that trade.
// It writes BCM2835 GPIO registers through an mmap of /dev/gpiomem, which is
// a store instruction per edge and cannot work anywhere else; this one is a
// syscall per edge and works everywhere. Expect, on a Pi:
//
//	drivers/bcm2835     ~30-60 ns per edge      megahertz of SWCLK
//	drivers/linuxgpiod  ~1-3 us per edge        hundreds of kilohertz
//
// Those two are order-of-magnitude expectations, not numbers measured here.
// Calibrate measures the real one on the board in front of it and reports it as
// speed_offset, which is where the ceiling actually is.
//
// So: on a Pi 1-4, use bcm2835. On a Pi 5, Pi 500 or CM5, this is the only
// thing here that runs at all -- those boards put their GPIO behind an RP1
// southbridge with a different register file and no /dev/gpiomem.
//
// # Why the kernel uAPI and not libgpiod
//
// libgpiod v2 is a wrapper over the ioctls in uapi.go, so this is the same
// interface with one fewer moving part. It buys three things. Nothing in this
// module except the USB HID transport is cgo, which is what makes `make pi` a
// toolchain-free cross-build -- linking libgpiod would take that away from the
// binary that needs it most. There is no libgpiod v1-versus-v2 problem, which
// otherwise costs a driver a slab of compatibility shims. And the ioctl
// payload is ours to shape, which is where the speed below comes from.
//
// # One request, so that two pins move in one syscall
//
// Every pin a wiring uses on one chip is claimed in a single GPIO_V2_GET_LINE
// request, so they share one GPIO_V2_LINE_SET_VALUES bitmap and one ioctl can
// drive any combination of them at once. A written SWD bit is therefore:
//
//	SWDIO := bit and SWCLK := low     one ioctl
//	SWCLK := high                     one ioctl
//
// A driver that claims each signal as its own libgpiod line -- which that
// library's API gives it little choice about -- pays two or three instead.
// Since every edge in this driver is a syscall, that ratio is very nearly the
// ratio of the achievable clocks.
//
// A read bit still costs three, because sampling SWDIO is its own ioctl and
// cannot be folded into either clock edge.
//
// # Pins on more than one chip
//
// A wiring does not have to be on one chip. The Pins constructors here take a
// [Pin] per signal and open a chip handle for each -- for the board whose data
// pins are on the SoC and whose buffer direction line is on an I2C or SPI
// expander.
//
// What it costs depends on which signal moved, and the driver is explicit about
// it rather than quietly slower:
//
//   - A direction line elsewhere is free. A turnaround was always an ioctl of
//     its own, and now it goes to a different file descriptor. Nothing on the
//     bit path changes.
//   - SWDIO elsewhere is not free. Lines in different requests cannot share a
//     SET_VALUES, so SWDIO needs an ioctl of its own and a written bit costs
//     three syscalls instead of two -- about a third off the clock.
//     [Driver.Fused] reports which of the two a driver ended up as, and
//     swd-gateway logs a warning when it is the slow one.
//
// One chip named two ways -- "0" and "/dev/gpiochip0", or a label -- is one
// chip: it is opened once and its pins are claimed together, because two
// requests on one chip would be refused for any line the first already holds
// and would give up the fusion for nothing.
//
// # Why one type here, and three in bcm2835
//
// [github.com/kvth/swd-to-go/drivers/bcm2835] has a separate type per SWDIO wiring
// and three copies of SwdTransfer, because a branch or an indirect call on a
// path where an edge costs 30 ns is a real fraction of the clock. That
// argument does not survive
// the move to a syscall per edge: the same branch is now a tenth of a percent,
// and BenchmarkDispatch and BenchmarkIoctlFloor in this package exist to put
// numbers on it rather than leave it as an assertion.
//
// So there is one Driver here with three constructors. What the three wirings
// differ in is which lines go into the request and which prebuilt SET_CONFIG
// payload a turnaround sends -- both settled once, at Open -- and the protocol
// lives in one copy that all of them run.
//
// # Speed, and why there is still a delay loop
//
// The clock this driver can reach is set by how fast the machine can complete
// an ioctl, not by anything in this package. Ask for 1 MHz and you will get
// whatever the syscalls allow, which on a Pi 5 is a few hundred kHz.
//
// It still implements [swd.Calibrator], for two reasons. Slow targets exist and
// a clock below what the syscalls happen to give has to be reachable, which is
// what the counted delay loop is for. And the measurement reports the same
// speed_coeff/speed_offset pair the bcm2835 driver does, so the gateway's flags,
// its Calibrate vendor command and its logging mean the same thing whichever
// backend is underneath -- with an offset an order of magnitude larger, saying
// exactly where the ceiling is.
//
// # Errors
//
// An ioctl can fail where a store to a mapped register cannot, and
// [swd.BitBanger] has nowhere to say so -- by design, since at these rates a
// returned error per bit would cost more than the bit. A failure is therefore
// recorded and reported by [Driver.Err], and the transfer carries on and comes
// back with whatever the wire looked like, which will be a protocol error.
// Err is worth checking when transfers start failing: it separates "the target
// is not answering" from "the GPIO went away underneath us".
//
// # Pull-ups
//
// Not set, matching [github.com/kvth/swd-to-go/drivers/bcm2835] and on the same
// reasoning: the board is assumed to supply SWDIO's pull-up, the C++ gateway
// has always run that way, and a bias this driver picks for itself is a bias
// nobody asked for. The
// uAPI has the flags if that ever needs revisiting -- it is a one-line change
// in requestLines, unlike on the BCM registers, where it would be a guess about
// which pull-control layout the board has.
package linuxgpiod

import (
	"fmt"
	"os"
	"strconv"
	"strings"
	"time"

	"github.com/kvth/swd-to-go"
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

// consumer is the label this process puts on the lines it claims. It shows up
// in `gpioinfo` and in the EBUSY another program gets for asking for the same
// line, which is the only reason it is worth setting.
const consumer = "swd-to-go"

// Pin is one GPIO line: the chip it is on, and its line number there.
//
// It exists for the wirings whose pins are not all on the same chip -- SWCLK
// and SWDIO on the SoC's own controller, say, with a buffer direction line on
// an I2C or SPI GPIO expander, which is a different /dev/gpiochipN.
//
// Chip is spelled the way [Chips] reports one, and the same chip may be named
// differently by two pins -- "0", "gpiochip0", "/dev/gpiochip0" and a label all
// resolve to the same device and are opened once.
//
// Where every pin is on one chip, which is every ordinary wiring, the
// constructors that take a single chip and plain line numbers say the same
// thing with less ceremony, and are what to reach for from Go. A command line
// is the other way round -- swd-gateway has no default chip and makes every pin
// name its own under --driver linuxgpiod, so that no pin can quietly land on
// the wrong controller -- and [ParsePin] is what reads one.
type Pin struct {
	Chip string
	Line int
}

func (p Pin) String() string { return fmt.Sprintf("%s:%d", p.Chip, p.Line) }

// ParsePin reads a pin the way a command line writes one: "gpiochip0:16", or a
// bare "16" for a line number with no chip named.
//
// The chip is everything before the *last* colon, so a path works as well as a
// name, a number or a label: "/dev/gpiochip0:16" and "pinctrl-rp1:16" are the
// same pin as "0:16". A bare number comes back with an empty Chip rather than
// an error, because what that means belongs to the caller: swd-gateway rejects
// one under --driver linuxgpiod and requires it under --driver bcm2835.
//
// It lives here rather than in each command because the spelling of a pin is
// this package's own convention -- the same one [Chips] reports and openChip
// resolves -- and two programs disagreeing about it would be two programs
// driving different lines.
func ParsePin(spec string) (Pin, error) {
	spec = strings.TrimSpace(spec)
	chip := ""
	if i := strings.LastIndex(spec, ":"); i >= 0 {
		chip, spec = strings.TrimSpace(spec[:i]), strings.TrimSpace(spec[i+1:])
		if chip == "" {
			return Pin{}, fmt.Errorf("%q has an empty chip before the colon", spec)
		}
	}
	line, err := strconv.Atoi(spec)
	if err != nil {
		return Pin{}, fmt.Errorf("%q is not a line number, or chip:line", spec)
	}
	return Pin{Chip: chip, Line: line}, nil
}

// line is one claimed GPIO resolved to the request it lives in and its bit
// there. The kernel indexes SET_VALUES and GET_VALUES by a line's position in
// the offsets array it was claimed with, not by its GPIO number, so this is
// what the whole driver works in.
type line struct {
	io    lineIO
	mask  uint64
	shift uint

	chip string // the chip's name, for messages only
	num  int    // the GPIO number, for messages only
}

func (l line) String() string { return fmt.Sprintf("%s:%d", l.chip, l.num) }

// chipConfig is a SET_CONFIG payload aimed at one request. A wiring on a single
// chip has one of these per direction change; a wiring spread over two has one
// per chip, which is why PortOn takes a slice and a turnaround does not.
type chipConfig struct {
	io  lineIO
	cfg lineConfig
}

// Driver bit-bangs SWD over a set of GPIO lines, on one chip or on several.
//
// Construct it with [OpenBidir], [OpenBidirBuffered] or [OpenSplit] -- or their
// Pins variants, where the pins are not all on one chip. Each takes exactly the
// pins its wiring has. A Driver arrives uncalibrated; see Speed in the package
// comment.
type Driver struct {
	files []*os.File
	ios   []lineIO

	clk, dout, din, dir line

	// fused is whether SWDIO and SWCLK ended up in the same line request,
	// which is what lets one ioctl carry both and is what makes a written bit
	// cost two syscalls instead of three. True whenever they are on the same
	// chip -- so, normally. See [Driver.Fused].
	fused bool

	// writeMask is what a bit write is allowed to touch on SWCLK's request:
	// SWCLK alone while the target owns SWDIO or while the two are on
	// different chips, SWCLK and SWDIO together when they are fused and the
	// host is driving.
	//
	// Masking is not an optimisation, it is the rule. Older kernels answer
	// -EPERM for a SET_VALUES whose mask names a line that is an input, and no
	// kernel promises anything useful for one.
	writeMask                       uint64
	writeMaskDriving, writeMaskIdle uint64

	// driving is whether the host currently owns SWDIO. Only the unfused path
	// reads it; the fused one has the answer folded into writeMask.
	driving bool

	// flipSwdio says whether turning the bus round means changing the SWDIO
	// GPIO's own direction -- true wherever one pin carries both, false for
	// the split wiring, where the data pins never move and the buffer does all
	// of it.
	flipSwdio bool

	// The SET_CONFIG payloads, built once at Open. A direction change is then
	// one ioctl carrying a struct that is already filled in.
	cfgPortOn, cfgPortOff   []chipConfig
	cfgSwdioOut, cfgSwdioIn chipConfig
	cfgClkOut, cfgClkIn     chipConfig

	// portOn is whether PortOn has been called, which measure needs to know;
	// nothing on the transfer path asks.
	portOn bool

	idleCycles uint8
	turnaround uint8
	dataPhase  bool

	// The delay loop, paced by a measured iteration count rather than by
	// polling a clock. Same parameterisation as drivers/bcm2835 and as
	// OpenOCD's bcm2835gpio speed_coeffs, so the two backends' numbers are
	// comparable -- see [swd.Calibrator].
	loopNs     float64 // cost of one delayLoop() iteration
	toggleNs   float64 // cost of one bare clock edge, the ioctl alone
	iterations uint32
	clockHz    uint32

	sink uint32 // written every delayLoop() iteration; keeps the compiler from eliding the loop

	err error // the first ioctl that failed; see [Driver.Err]
}

var (
	_ swd.BitBanger  = (*Driver)(nil)
	_ swd.Calibrator = (*Driver)(nil)
)

// ---------------- Construction ----------------

// OpenBidir claims the pins for a single bidirectional SWDIO wired straight to
// the target, with no buffer: the GPIO itself is flipped to turn the bus round.
// It is OpenOCD's "adapter gpio swdio" with neither swdio_dir nor swdio_in.
//
// chip names the GPIO chip both pins are on, as a number ("0"), a name
// ("gpiochip0"), a path ("/dev/gpiochip0") or a label ("pinctrl-rp1"). [Chips]
// lists what this machine has; guessing a number is how a driver ends up
// wiggling the wrong thing. [OpenBidirPins] is the same wiring with the pins on
// different chips.
func OpenBidir(chip string, swclk, swdio int) (*Driver, error) {
	return OpenBidirPins(Pin{chip, swclk}, Pin{chip, swdio})
}

// OpenBidirPins is [OpenBidir] with each pin naming its own chip.
func OpenBidirPins(swclk, swdio Pin) (*Driver, error) {
	return open(wiring{swclk: swclk, swdioIn: swdio, swdioOut: swdio,
		swdioDir: Pin{Line: -1}, shared: true})
}

// OpenBidirBuffered claims the pins for a single bidirectional SWDIO behind a
// buffer with its own direction line. It is OpenOCD's "adapter gpio swdio" plus
// "adapter gpio swdio_dir", with no swdio_in.
//
// Both the GPIO and the buffer have to move to turn the bus round; see
// [Driver.SwdioOutEnable] for the order and why it is not arbitrary.
func OpenBidirBuffered(chip string, swclk, swdio, swdioDir int) (*Driver, error) {
	return OpenBidirBufferedPins(Pin{chip, swclk}, Pin{chip, swdio}, Pin{chip, swdioDir})
}

// OpenBidirBufferedPins is [OpenBidirBuffered] with each pin naming its own
// chip. A direction line on a GPIO expander while the data pins are on the SoC
// is the wiring this is for, and it costs nothing on the bit path: a turnaround
// was already its own ioctl.
func OpenBidirBufferedPins(swclk, swdio, swdioDir Pin) (*Driver, error) {
	return open(wiring{swclk: swclk, swdioIn: swdio, swdioOut: swdio,
		swdioDir: swdioDir, shared: true})
}

// OpenSplit claims the pins for separate SWDIO input and output pins behind a
// buffer turned round by a direction line -- the wiring most boards with a
// level shifter use, and the one the C++ gateway ships with.
//
// It is also the fastest of the three here, by more than it is on the BCM
// registers: the data pins keep their directions for the life of the session,
// so a turnaround is one SET_VALUES on the direction line rather than a
// SET_CONFIG that reconfigures a line.
func OpenSplit(chip string, swclk, swdioIn, swdioOut, swdioDir int) (*Driver, error) {
	return OpenSplitPins(Pin{chip, swclk}, Pin{chip, swdioIn}, Pin{chip, swdioOut}, Pin{chip, swdioDir})
}

// OpenSplitPins is [OpenSplit] with each pin naming its own chip.
func OpenSplitPins(swclk, swdioIn, swdioOut, swdioDir Pin) (*Driver, error) {
	return open(wiring{swclk: swclk, swdioIn: swdioIn, swdioOut: swdioOut, swdioDir: swdioDir})
}

// wiring is the pins a constructor was given, with a negative line for the
// direction pin a wiring does not have. It exists only between a constructor
// and build; nothing downstream ever asks a Driver how it is wired.
type wiring struct {
	swclk, swdioIn, swdioOut, swdioDir Pin

	// shared says the constructor meant SWDIO in and out to be one pin. It is
	// what separates a bidirectional wiring from a split one given the same
	// line twice by mistake.
	shared bool
}

func (w wiring) signals() []struct {
	name string
	pin  Pin
} {
	out := []struct {
		name string
		pin  Pin
	}{
		{"swclk", w.swclk},
		{"swdio-out", w.swdioOut},
		{"swdio-in", w.swdioIn},
	}
	if w.swdioDir.Line >= 0 {
		out = append(out, struct {
			name string
			pin  Pin
		}{"swdio-dir", w.swdioDir})
	}
	return out
}

// hooks are build's two edges on the outside world: naming a chip and claiming
// lines on one. Production passes the real ones; the tests pass fakes, so what
// they drive is the whole driver with only the syscalls swapped out.
type hooks struct {
	openChip func(name string) (*os.File, Chip, error)
	claim    func(chip Chip, f *os.File, offsets []uint32) (lineIO, error)
}

func open(w wiring) (*Driver, error) {
	return build(w, hooks{
		openChip: openChip,
		claim: func(_ Chip, f *os.File, offsets []uint32) (lineIO, error) {
			return requestLines(f, consumer, offsets)
		},
	})
}

// request is one chip's worth of the wiring: the chip, its open file, and the
// lines this driver wants from it, in the order they will be claimed.
type request struct {
	info    Chip
	file    *os.File
	offsets []uint32
	io      lineIO
}

// build resolves a wiring into claimed lines.
//
// The shape of the work is: open each distinct chip once, group the pins by
// chip, claim one request per chip, then compute every mask and every
// SET_CONFIG payload from where each pin landed. All of it happens here, at
// open, so that nothing on the transfer path has to ask which chip a pin is on.
func build(w wiring, h hooks) (*Driver, error) {
	d := &Driver{turnaround: 1}

	var reqs []*request
	byName := map[string]*request{} // as spelled by the caller
	byPath := map[string]*request{} // as the kernel knows it
	owner := map[string]string{}    // "path:line" -> the signal that claimed it

	closeAll := func() {
		for _, r := range reqs {
			if r.io != nil {
				r.io.close()
			}
			if r.file != nil {
				r.file.Close()
			}
		}
	}

	// chipFor opens a chip, or hands back the one already open under another
	// spelling. Two spellings of one chip must not become two requests: the
	// second would be refused for any line the first already holds, and a
	// wiring split across two requests on one chip loses the fusion below for
	// no reason at all.
	chipFor := func(name string) (*request, error) {
		if name == "" {
			return nil, fmt.Errorf("linuxgpiod: no gpiochip named for one of the pins")
		}
		if r, ok := byName[name]; ok {
			return r, nil
		}
		f, info, err := h.openChip(name)
		if err != nil {
			return nil, err
		}
		if r, ok := byPath[info.Path]; ok {
			f.Close()
			byName[name] = r
			return r, nil
		}
		r := &request{info: info, file: f}
		reqs = append(reqs, r)
		byName[name] = r
		byPath[info.Path] = r
		return r, nil
	}

	// place resolves one signal's pin to the request it will live in and its
	// index there, rejecting a line that is not on the chip and a line another
	// signal has already claimed.
	place := func(name string, p Pin) (line, error) {
		r, err := chipFor(p.Chip)
		if err != nil {
			return line{}, err
		}
		if p.Line < 0 || uint32(p.Line) >= r.info.Lines {
			return line{}, fmt.Errorf("linuxgpiod: %s line %d is outside %s, which has lines 0..%d",
				name, p.Line, r.info.Name, r.info.Lines-1)
		}

		key := fmt.Sprintf("%s:%d", r.info.Path, p.Line)
		if other, taken := owner[key]; taken {
			// SWDIO in and out sharing a pin is the bidirectional wiring, not
			// a mistake -- but only when a constructor said so.
			shared := w.shared && (other == "swdio-out" && name == "swdio-in")
			if !shared {
				return line{}, fmt.Errorf("linuxgpiod: %s and %s cannot both be %s line %d",
					other, name, r.info.Name, p.Line)
			}
		} else {
			owner[key] = name
		}

		idx := -1
		for i, o := range r.offsets {
			if o == uint32(p.Line) {
				idx = i
				break
			}
		}
		if idx < 0 {
			r.offsets = append(r.offsets, uint32(p.Line))
			idx = len(r.offsets) - 1
		}
		return line{mask: 1 << uint(idx), shift: uint(idx), chip: r.info.Name, num: p.Line}, nil
	}

	if !w.shared && w.swdioIn == w.swdioOut {
		return nil, fmt.Errorf("linuxgpiod: the split wiring needs separate SWDIO in and out lines, "+
			"both given as %s; OpenBidir or OpenBidirBuffered is the one-pin wiring", w.swdioIn)
	}

	// The claim order is fixed: clock, SWDIO out, SWDIO in, direction.
	placed := map[string]line{}
	for _, s := range w.signals() {
		l, err := place(s.name, s.pin)
		if err != nil {
			closeAll()
			return nil, err
		}
		placed[s.name] = l
	}
	d.clk, d.dout, d.din = placed["swclk"], placed["swdio-out"], placed["swdio-in"]
	d.dir = placed["swdio-dir"] // the zero line, with a nil io, when unwired

	// Claim them, one request per chip.
	for _, r := range reqs {
		io, err := h.claim(r.info, r.file, r.offsets)
		if err != nil {
			closeAll()
			return nil, err
		}
		r.io = io
		d.ios = append(d.ios, io)
		d.files = append(d.files, r.file)
	}

	// Now every line knows which request it is in.
	bind := func(l *line, p Pin) {
		if l.mask == 0 && l.chip == "" {
			return // an unwired direction pin
		}
		l.io = byName[p.Chip].io
	}
	bind(&d.clk, w.swclk)
	bind(&d.dout, w.swdioOut)
	bind(&d.din, w.swdioIn)
	if w.swdioDir.Line >= 0 {
		bind(&d.dir, w.swdioDir)
	}

	// One pin carrying both directions is what makes a turnaround a
	// reconfiguration rather than a value write.
	d.flipSwdio = w.swdioIn == w.swdioOut

	// Fusion: one ioctl can carry SWDIO and SWCLK only if they were claimed
	// together, which means only if they are on the same chip.
	d.fused = d.dout.io == d.clk.io
	d.writeMaskIdle = d.clk.mask
	d.writeMaskDriving = d.clk.mask
	if d.fused {
		d.writeMaskDriving |= d.dout.mask
	}
	d.writeMask = d.writeMaskIdle

	d.buildConfigs(reqs)

	// Remember a default clock, so that a later Calibrate has something sane
	// to apply even if the caller never names one.
	d.SetFrequencyHz(1_200_000)
	return d, nil
}

// buildConfigs works out, per request, which of its lines this driver drives
// and at what level, and turns that into the SET_CONFIG payloads the port and
// direction calls send.
func (d *Driver) buildConfigs(reqs []*request) {
	// PortOn: clock out and low, SWDIO out, direction out and low (host
	// drives). SWDIO's idle level follows drivers/bcm2835 exactly, high on the
	// bidirectional wirings and low on the split one, so that the two backends
	// leave the same thing on a scope. The bidirectional wirings have to drive
	// the SWD idle level themselves because there is no buffer holding it;
	// nothing turns on the split case, whose first caller begins with a line
	// reset.
	for _, r := range reqs {
		var outputs, inputs, values, all uint64
		for _, l := range []line{d.clk, d.dout, d.dir} {
			if l.io == r.io && l.mask != 0 {
				outputs |= l.mask
				all |= l.mask
			}
		}
		if d.din.io == r.io {
			all |= d.din.mask
			// On the split wiring SWDIO in is a line of its own and stays an
			// input; on the others it is the same line as SWDIO out and must
			// not be named twice.
			inputs = d.din.mask &^ d.dout.mask
		}
		if d.flipSwdio && d.dout.io == r.io {
			values |= d.dout.mask
		}
		if all == 0 {
			continue
		}
		d.cfgPortOn = append(d.cfgPortOn, chipConfig{r.io, makeConfig(outputs, inputs, values)})
		d.cfgPortOff = append(d.cfgPortOff, chipConfig{r.io, makeConfig(0, all, 0)})
	}

	// A turnaround on a shared pin. Out-enable drives it high, the SWD idle
	// level.
	d.cfgSwdioOut = chipConfig{d.dout.io, makeConfig(d.dout.mask, 0, d.dout.mask)}
	d.cfgSwdioIn = chipConfig{d.dout.io, makeConfig(0, d.dout.mask, 0)}

	// SWCLK on its own, for measure() to borrow the line with; see there.
	d.cfgClkOut = chipConfig{d.clk.io, makeConfig(d.clk.mask, 0, 0)}
	d.cfgClkIn = chipConfig{d.clk.io, makeConfig(0, d.clk.mask, 0)}
}

// ---------------- What it ended up with ----------------

// Wiring reports which chip and line each signal landed on, in the form
// `gpiochip0:16`. It is worth logging: the whole risk of a chardev driver is
// naming the wrong chip, and this is the resolved answer rather than what was
// asked for.
func (d *Driver) Wiring() string {
	parts := []string{"swclk " + d.clk.String()}
	if d.flipSwdio {
		parts = append(parts, "swdio "+d.dout.String())
	} else {
		parts = append(parts, "swdio in "+d.din.String(), "out "+d.dout.String())
	}
	if d.dir.io != nil {
		parts = append(parts, "dir "+d.dir.String())
	}
	return strings.Join(parts, ", ")
}

// Fused reports whether SWDIO and SWCLK were claimed in the same request, which
// they are whenever they are on the same chip.
//
// When they are, one ioctl drives both and a written bit costs two syscalls.
// When they are not -- a wiring spread across two chips -- SWDIO needs an ioctl
// of its own and a written bit costs three, so the clock is about a third
// slower. Nothing else changes, and a read bit costs the same either way.
func (d *Driver) Fused() bool { return d.fused }

// Err reports the first ioctl that failed, if any. See Errors in the package
// comment: the bit-banging interface has nowhere to return one, so failures
// are recorded here and the transfer carries on.
func (d *Driver) Err() error { return d.err }

// Close releases the lines and the chips. The kernel returns them to whatever
// state the pinctrl driver has for them, so SWCLK and SWDIO stop being driven.
func (d *Driver) Close() error {
	var err error
	for _, io := range d.ios {
		if cerr := io.close(); err == nil {
			err = cerr
		}
	}
	for _, f := range d.files {
		if cerr := f.Close(); err == nil {
			err = cerr
		}
	}
	d.ios, d.files = nil, nil
	return err
}

// ---------------- The kernel, with the error put somewhere ----------------

func (d *Driver) fail(what string, err error) {
	if d.err == nil {
		d.err = fmt.Errorf("linuxgpiod: %s: %w", what, err)
	}
}

// write drives the lines in mask, on one request, to the levels in bits. Bits
// outside the mask are ignored by the kernel, which is what lets swWriteBit
// compute SWDIO's level unconditionally and let writeMask decide whether it is
// sent.
func (d *Driver) write(io lineIO, bits, mask uint64) {
	if mask == 0 {
		// The kernel answers -EINVAL for a SET_VALUES that names no line, and
		// an empty write is a wiring that has nothing to say, not an error.
		return
	}
	if err := io.setValues(bits, mask); err != nil {
		d.fail("SET_VALUES", err)
	}
}

func (d *Driver) read(l line) uint64 {
	v, err := l.io.getValues(l.mask)
	if err != nil {
		d.fail("GET_VALUES", err)
		return 0
	}
	return v
}

func (d *Driver) config(c chipConfig) {
	if err := c.io.setConfig(&c.cfg); err != nil {
		d.fail("SET_CONFIG", err)
	}
}

func (d *Driver) configAll(cs []chipConfig) {
	for _, c := range cs {
		d.config(c)
	}
}

// ---------------- Port ----------------

func (d *Driver) PortOn() {
	d.configAll(d.cfgPortOn)
	d.driving = true
	d.writeMask = d.writeMaskDriving
	d.portOn = true
}

func (d *Driver) PortOff() {
	d.configAll(d.cfgPortOff)
	d.driving = false
	d.writeMask = d.writeMaskIdle
	d.portOn = false
}

// ---------------- Direction ----------------

// SwdioOutEnable hands SWDIO to the host, SwdioOutDisable to the target.
//
// The order is the one drivers/bcm2835 uses, and it is not arbitrary:
// whichever side is about to stop driving is pointed away from first, so the
// host pin and the buffer output are never both driving the wire. Direction
// low is the host driving, matching drivers/bcm2835 and the C++ gateway it
// follows; OpenOCD's swdio_dir has the opposite polarity.
func (d *Driver) SwdioOutEnable() {
	if d.dir.io != nil {
		d.write(d.dir.io, 0, d.dir.mask)
	}
	if d.flipSwdio {
		d.config(d.cfgSwdioOut)
	}
	d.driving = true
	d.writeMask = d.writeMaskDriving
}

func (d *Driver) SwdioOutDisable() {
	if d.flipSwdio {
		d.config(d.cfgSwdioIn)
	}
	if d.dir.io != nil {
		d.write(d.dir.io, d.dir.mask, d.dir.mask)
	}
	d.driving = false
	d.writeMask = d.writeMaskIdle
}

// ---------------- Pin access ----------------
//
// These exist for DAP_SWJ_Pins and for nothing else; the protocol below reaches
// the lines through write and read directly.

func (d *Driver) SwclkSet() { d.write(d.clk.io, d.clk.mask, d.clk.mask) }
func (d *Driver) SwclkClr() { d.write(d.clk.io, 0, d.clk.mask) }

// SwclkIn reads SWCLK back. It is an output for the life of a session, and the
// kernel answers a GET_VALUES on an output line with the level it is driving,
// which is what DAP_SWJ_Pins wants to hear.
func (d *Driver) SwclkIn() uint8 {
	return uint8((d.read(d.clk) >> d.clk.shift) & 1)
}

func (d *Driver) SwdioIn() uint8 {
	return uint8((d.read(d.din) >> d.din.shift) & 1)
}

// SwdioSet and SwdioClr drive SWDIO, and do nothing at all while the target
// owns it. That is the same no-op drivers/bcm2835 gives -- a GPSET0 write to a
// pin that is an input changes nothing visible -- but here it has to be
// explicit, because naming an input line in a SET_VALUES mask is a kernel error
// rather than a write into the void.
func (d *Driver) SwdioSet() { d.pinSWDIOOut(1) }
func (d *Driver) SwdioClr() { d.pinSWDIOOut(0) }

func (d *Driver) pinSWDIOOut(bit uint32) {
	if !d.driving {
		return
	}
	d.write(d.dout.io, uint64(bit&1)<<d.dout.shift, d.dout.mask)
}

// ---------------- Configuration ----------------

func (d *Driver) SwdConfigure(turnaround uint8, dataPhase bool) {
	d.turnaround = turnaround
	d.dataPhase = dataPhase
}

// ---------------- Clock pacing ----------------

// delayLoop is a plain counted loop with no timer of any kind inside it,
// matching drivers/bcm2835 so that the two backends' speed coefficients are
// measured against the same thing. d.sink stops the Go compiler recognising the
// loop as dead code and deleting it.
func (d *Driver) delayLoop(n uint32) {
	var x uint32
	for i := uint32(0); i < n; i++ {
		x++
	}
	d.sink = x
}

// measure times one delay-loop iteration and one bare clock edge, by driving
// exactly the calls the real edges drive.
//
// The edge count is far lower than the bcm2835 driver's, because an edge here
// is a syscall: at a microsecond apiece, its 500,000 pairs would be a second of
// SWCLK. Twenty thousand pairs is a few tens of milliseconds and is already
// hundreds of times more samples than the variance needs.
func (d *Driver) measure() {
	const loopIterations = 20_000_000
	start := time.Now()
	d.delayLoop(loopIterations)
	d.loopNs = time.Since(start).Seconds() * 1e9 / float64(loopIterations)

	// Calibrating before PortOn is the normal case and the safe one -- it is
	// what the gateway does, before a target is attached -- but SWCLK is still
	// an input there, and an edge on an input line is not the edge being
	// measured. The kernel either refuses it or drops it early, and either way
	// it is cheaper than the real thing, which would show up as a maximum
	// clock this driver cannot actually reach. So borrow the line, and give it
	// back exactly as it was found.
	borrowed := !d.portOn
	if borrowed {
		d.config(d.cfgClkOut)
	}

	const togglePairs = 20_000
	start = time.Now()
	for i := 0; i < togglePairs; i++ {
		d.SwclkClr()
		d.SwclkSet()
	}
	// Two edges per iteration, clear then set.
	d.toggleNs = time.Since(start).Seconds() * 1e9 / float64(togglePairs) / 2

	if borrowed {
		d.config(d.cfgClkIn)
	}
}

// SetFrequencyHz sets the SWCLK half period as a count of loop iterations:
// whatever the half period is, less the edge's own fixed cost. On this backend
// that cost is a syscall, so most requested clocks leave nothing to pace and
// the answer is zero iterations -- as fast as the kernel goes, and slower than
// was asked for.
func (d *Driver) SetFrequencyHz(hz uint32) {
	if hz == 0 {
		return
	}
	d.clockHz = hz
	if d.loopNs <= 0 {
		return
	}

	remaining := 1e9/float64(hz)/2 - d.toggleNs
	if remaining < 0 {
		remaining = 0
	}
	d.iterations = uint32(remaining / d.loopNs)
}

// Calibrate re-measures and applies the result at the clock currently in
// effect. See [swd.Calibrator].
func (d *Driver) Calibrate() (coeff, offset uint32, err error) {
	d.measure()
	if d.loopNs <= 0 {
		return 0, 0, fmt.Errorf("linuxgpiod: delay loop measured as taking no time")
	}
	if d.err != nil {
		// Measuring is thousands of ioctls; if they are failing, the numbers
		// describe a broken fd rather than a clock.
		return 0, 0, fmt.Errorf("linuxgpiod: calibrating: %w", d.err)
	}
	d.SetFrequencyHz(d.clockHz)
	coeff, offset = d.SpeedCoeffs()
	return coeff, offset, nil
}

// SpeedCoeffs converts the measured nanosecond costs into the speed_coeff and
// speed_offset pair. One iteration costs 500000/coeff ns, so coeff is
// 500000/iterationCost, and offset is the edge cost in iteration units --
// which on this backend is a large number, and is exactly the statement that
// the maximum clock is coeff/offset kHz.
func (d *Driver) SpeedCoeffs() (coeff, offset uint32) {
	if d.loopNs <= 0 {
		return 0, 0
	}
	return clampCoeffs(
		int64(500000.0/d.loopNs+0.5),
		int64(d.toggleNs/d.loopNs+0.5),
	)
}

// SetSpeedCoeffs goes the other way: recover the nanosecond costs the loop is
// actually paced by from values measured earlier.
func (d *Driver) SetSpeedCoeffs(coeff, offset uint32) error {
	if coeff == 0 {
		return fmt.Errorf("linuxgpiod: speed coefficient must not be zero")
	}
	if offset == 0 || offset >= coeff {
		return fmt.Errorf("linuxgpiod: speed offset must be at least 1 and below the coefficient %d, got %d",
			coeff, offset)
	}
	d.loopNs = 500000.0 / float64(coeff)
	d.toggleNs = float64(offset) * d.loopNs
	d.SetFrequencyHz(d.clockHz)
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

func (d *Driver) pinDelay() { d.delayLoop(d.iterations) }

// ---------------- Bits ----------------

// swWriteBit is where the single-request layout pays for itself: SWDIO's level
// and SWCLK's falling edge go out as one ioctl rather than two.
//
// On the fused path the SWDIO bit is computed whether or not the host is
// driving the line. writeMask decides whether it is sent, and the kernel
// ignores bits outside the mask, so the branch that would otherwise be there
// does not need to exist.
//
// The unfused path is a wiring whose SWDIO and SWCLK are on different chips.
// They are then in different requests and no ioctl can carry both, so SWDIO
// gets one of its own -- written first, because SWCLK's falling edge is what
// the target latches on. The test for which path this is costs about a
// nanosecond against a syscall that costs hundreds; see BenchmarkDispatch.
func (d *Driver) swWriteBit(bit uint32) {
	if d.fused {
		d.write(d.clk.io, uint64(bit&1)<<d.dout.shift, d.writeMask)
	} else {
		if d.driving {
			d.write(d.dout.io, uint64(bit&1)<<d.dout.shift, d.dout.mask)
		}
		d.write(d.clk.io, 0, d.clk.mask)
	}
	d.pinDelay()
	d.write(d.clk.io, d.clk.mask, d.clk.mask)
	d.pinDelay()
}

func (d *Driver) swReadBit() uint32 {
	d.write(d.clk.io, 0, d.clk.mask)
	d.pinDelay()
	bit := uint32(d.read(d.din)>>d.din.shift) & 1
	d.write(d.clk.io, d.clk.mask, d.clk.mask)
	d.pinDelay()
	return bit
}

// swClockCycle is one SWCLK cycle with SWDIO left exactly as it is.
func (d *Driver) swClockCycle() {
	d.write(d.clk.io, 0, d.clk.mask)
	d.pinDelay()
	d.write(d.clk.io, d.clk.mask, d.clk.mask)
	d.pinDelay()
}

// ---------------- Sequences ----------------

// SwjSequence clocks raw bits out with SWDIO driven throughout.
//
// drivers/bcm2835 writes SWDIO and then clocks, which is three register stores;
// here the two are fused into swWriteBit's two ioctls, which is the same
// waveform and a third fewer syscalls. It assumes the host is driving SWDIO,
// which is [swd.BitBanger]'s contract for the sequence calls -- the caller
// brackets its own direction changes, and probe/bitbang leaves the line with
// the host after every transfer and every sequence run.
func (d *Driver) SwjSequence(bitCount uint32, data []byte) {
	var val uint32
	var n uint32

	idx := 0
	for bitCount > 0 {
		if n == 0 {
			val = uint32(data[idx])
			idx++
			n = 8
		}
		d.swWriteBit(val & 1)
		val >>= 1
		n--
		bitCount--
	}
}

func (d *Driver) SwdWriteSequence(bitCount uint32, data []byte) {
	inIdx := 0
	for bitCount > 0 {
		val := uint32(data[inIdx])
		inIdx++

		k := uint32(8)
		for k > 0 && bitCount > 0 {
			d.swWriteBit(val & 1)
			val >>= 1
			k--
			bitCount--
		}
	}
}

func (d *Driver) SwdReadSequence(bitCount uint32, data []byte) {
	outIdx := 0
	for bitCount > 0 {
		var val uint32
		k := uint32(8)

		for k > 0 && bitCount > 0 {
			bit := d.swReadBit()
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

// ---------------- SWD transfer ----------------

// SwdTransfer runs one SWD transfer. There is one copy of it here, where
// drivers/bcm2835 has three; see "Why one type here" in the package comment for
// what changed to make that the right call.
func (d *Driver) SwdTransfer(req uint8, data *uint32, requestTimestamp func()) uint8 {
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
