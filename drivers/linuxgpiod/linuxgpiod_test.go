//go:build linux

package linuxgpiod

import (
	"errors"
	"fmt"
	"os"
	"strconv"
	"strings"
	"testing"
	"unsafe"
)

// The tests stand drivers up over a fake gpiochip rather than a real one, so
// they run on any machine and claim nothing. The fake is not a stub: it tracks
// each line's direction and level, applies a SET_CONFIG payload the way
// linereq_set_config() does -- including leaving lines with no direction flag
// alone -- and refuses a SET_VALUES naming a line that is an input, which is
// the rule this driver's writeMask exists to keep.
//
// What that buys over drivers/bcm2835's fake register block is countable
// edges. A write-only GPSET0 mask cannot say how many times SWCLK moved; a
// recorded ioctl trace can, so the waveform assertions below are about the
// actual number of clocks, and not merely about the ACK that came back.

const (
	lineSWCLK    = 16
	lineSWDIOIn  = 12
	lineSWDIOOut = 20
	lineSWDIODir = 21

	// lineSWDIO is the single data line the bidirectional wirings use.
	lineSWDIO = 20
)

var (
	testChip = Chip{Path: "/dev/gpiochip0", Name: "gpiochip0", Label: "fake", Lines: 32}

	// testExpander is a second, smaller chip -- an I2C GPIO expander, say --
	// for the wirings whose pins are not all in one place.
	testExpander = Chip{Path: "/dev/gpiochip9", Name: "gpiochip9", Label: "fake-expander", Lines: 8}
)

// ---------------- The fake chip ----------------

type opKind int

const (
	opSet opKind = iota
	opGet
	opConfig
)

type op struct {
	kind       opKind
	bits, mask uint64

	// at orders this op against every other chip's, for the wirings whose
	// ioctls are spread over more than one request.
	at int
}

type fakeChip struct {
	offsets []uint32

	out map[uint32]bool // is this line an output
	val map[uint32]bool // the level an output is driving
	in  map[uint32]bool // the level the outside world presents on an input

	ops []op

	// seq is shared by every chip in one test world, so that ops on different
	// chips can be put in order against each other.
	seq *int

	// script is what the target puts on SWDIO, one entry per sample, for
	// driving a transfer down a particular ACK path. An exhausted script falls
	// back to the steady level in `in`.
	script []bool

	// failFrom makes every call from the nth onwards fail, for the error path.
	failFrom int
	calls    int
}

func newFake(offsets []uint32) *fakeChip {
	f := &fakeChip{
		offsets: append([]uint32(nil), offsets...),
		out:     map[uint32]bool{},
		val:     map[uint32]bool{},
		in:      map[uint32]bool{},
		seq:     new(int),
	}
	// requestLines asks for them all as inputs.
	for _, o := range f.offsets {
		f.out[o] = false
	}
	return f
}

var errInjected = errors.New("injected ioctl failure")

func (f *fakeChip) failing() error {
	f.calls++
	if f.failFrom > 0 && f.calls >= f.failFrom {
		return errInjected
	}
	return nil
}

func (f *fakeChip) setValues(bits, mask uint64) error {
	if err := f.failing(); err != nil {
		return err
	}
	if mask == 0 {
		// linereq_set_values() answers -EINVAL for a mask naming no line.
		return fmt.Errorf("SET_VALUES with an empty mask")
	}
	for i, o := range f.offsets {
		if mask&(1<<uint(i)) == 0 {
			continue
		}
		if !f.out[o] {
			return fmt.Errorf("SET_VALUES named line %d, which is an input", o)
		}
		f.val[o] = bits&(1<<uint(i)) != 0
	}
	f.ops = append(f.ops, op{opSet, bits, mask, f.next()})
	return nil
}

func (f *fakeChip) getValues(mask uint64) (uint64, error) {
	if err := f.failing(); err != nil {
		return 0, err
	}
	if mask == 0 {
		return 0, fmt.Errorf("GET_VALUES with an empty mask")
	}
	var bits uint64
	for i, o := range f.offsets {
		if mask&(1<<uint(i)) == 0 {
			continue
		}
		level := f.in[o]
		if f.out[o] {
			// An output reads back what it is driving.
			level = f.val[o]
		} else if len(f.script) > 0 {
			level = f.script[0]
			f.script = f.script[1:]
		}
		if level {
			bits |= 1 << uint(i)
		}
	}
	f.ops = append(f.ops, op{opGet, bits, mask, f.next()})
	return bits, nil
}

// setConfig applies a payload the way linereq_set_config() does: per line, take
// the flags from the first matching attribute or from the default, and skip the
// line entirely when they carry no direction.
func (f *fakeChip) setConfig(cfg *lineConfig) error {
	if err := f.failing(); err != nil {
		return err
	}
	for i, o := range f.offsets {
		mask := uint64(1) << uint(i)

		flags := cfg.flags
		for a := uint32(0); a < cfg.numAttrs; a++ {
			if cfg.attrs[a].attr.id == attrIDFlags && cfg.attrs[a].mask&mask != 0 {
				flags = cfg.attrs[a].attr.value
				break
			}
		}
		if flags&(lineFlagInput|lineFlagOutput) == 0 {
			continue
		}

		if flags&lineFlagOutput != 0 {
			value := false
			for a := uint32(0); a < cfg.numAttrs; a++ {
				if cfg.attrs[a].attr.id == attrIDOutputValues && cfg.attrs[a].mask&mask != 0 {
					value = cfg.attrs[a].attr.value&mask != 0
					break
				}
			}
			f.out[o] = true
			f.val[o] = value
		} else {
			f.out[o] = false
		}
	}
	f.ops = append(f.ops, op{kind: opConfig, at: f.next()})
	return nil
}

func (f *fakeChip) close() error { return nil }

// next stamps an op with its place in the whole world's order.
func (f *fakeChip) next() int {
	*f.seq++
	return *f.seq
}

func (f *fakeChip) reset() { f.ops = nil }

func (f *fakeChip) counts() (sets, gets, configs int) {
	for _, o := range f.ops {
		switch o.kind {
		case opSet:
			sets++
		case opGet:
			gets++
		case opConfig:
			configs++
		}
	}
	return
}

// ---------------- Standing a driver up ----------------

// fakeHooks makes a world of chips the driver can be built against. Every chip
// in it answers to its number, name, path and label, exactly as openChip
// resolves a real one, so the "same chip under two names" case is reachable.
//
// The files handed back are /dev/null: build closes a duplicate, and Close
// closes them all, and both should be real file operations rather than
// something the test has to special-case.
func fakeHooks(t *testing.T, chips ...Chip) (hooks, map[string]*fakeChip) {
	t.Helper()
	fakes := map[string]*fakeChip{}

	find := func(name string) (Chip, bool) {
		for i, c := range chips {
			if name == c.Name || name == c.Path || name == c.Label ||
				name == strconv.Itoa(i) || name == strings.TrimPrefix(c.Name, "gpiochip") {
				return c, true
			}
		}
		return Chip{}, false
	}

	return hooks{
		openChip: func(name string) (*os.File, Chip, error) {
			c, ok := find(name)
			if !ok {
				return nil, Chip{}, fmt.Errorf("no gpiochip named %q", name)
			}
			f, err := os.Open(os.DevNull)
			if err != nil {
				t.Fatal(err)
			}
			return f, c, nil
		},
		claim: func(c Chip, _ *os.File, offsets []uint32) (lineIO, error) {
			f := newFake(offsets)
			for _, other := range fakes {
				f.seq = other.seq
				break
			}
			fakes[c.Name] = f
			return f, nil
		},
	}, fakes
}

func openFake(t *testing.T, w wiring) (*Driver, *fakeChip) {
	t.Helper()
	h, fakes := fakeHooks(t, testChip)
	d, err := build(w, h)
	if err != nil {
		t.Fatalf("build: %v", err)
	}
	return d, fakes[testChip.Name]
}

// on names a pin on the single test chip.
func on(line int) Pin { return Pin{testChip.Name, line} }

func newBidir(t *testing.T) (*Driver, *fakeChip) {
	return openFake(t, wiring{swclk: on(lineSWCLK), swdioIn: on(lineSWDIO),
		swdioOut: on(lineSWDIO), swdioDir: Pin{Line: -1}, shared: true})
}

func newBidirBuffered(t *testing.T) (*Driver, *fakeChip) {
	return openFake(t, wiring{swclk: on(lineSWCLK), swdioIn: on(lineSWDIO),
		swdioOut: on(lineSWDIO), swdioDir: on(lineSWDIODir), shared: true})
}

func newSplit(t *testing.T) (*Driver, *fakeChip) {
	return openFake(t, wiring{swclk: on(lineSWCLK), swdioIn: on(lineSWDIOIn),
		swdioOut: on(lineSWDIOOut), swdioDir: on(lineSWDIODir)})
}

// ---------------- The uAPI ----------------

// The structs go to the kernel by raw pointer, so their layout is the
// interface. A field of the wrong width here is an EINVAL on a Pi and nothing
// at all on a development machine, which is the worst way round.
func TestStructLayoutMatchesTheKernel(t *testing.T) {
	for _, c := range []struct {
		name string
		got  uintptr
		want uintptr
	}{
		{"gpiochip_info", unsafe.Sizeof(chipInfo{}), 68},
		{"gpio_v2_line_values", unsafe.Sizeof(lineValues{}), 16},
		{"gpio_v2_line_attribute", unsafe.Sizeof(lineAttribute{}), 16},
		{"gpio_v2_line_config_attribute", unsafe.Sizeof(lineConfigAttribute{}), 24},
		{"gpio_v2_line_config", unsafe.Sizeof(lineConfig{}), 272},
		{"gpio_v2_line_request", unsafe.Sizeof(lineRequest{}), 592},
	} {
		if c.got != c.want {
			t.Errorf("sizeof(%s) = %d, kernel says %d", c.name, c.got, c.want)
		}
	}

	for _, c := range []struct {
		name string
		got  uintptr
		want uintptr
	}{
		{"line_config.attrs", unsafe.Offsetof(lineConfig{}.attrs), 32},
		{"line_request.consumer", unsafe.Offsetof(lineRequest{}.consumer), 256},
		{"line_request.config", unsafe.Offsetof(lineRequest{}.config), 288},
		{"line_request.num_lines", unsafe.Offsetof(lineRequest{}.numLines), 560},
		{"line_request.fd", unsafe.Offsetof(lineRequest{}.fd), 588},
	} {
		if c.got != c.want {
			t.Errorf("offsetof(%s) = %d, kernel says %d", c.name, c.got, c.want)
		}
	}
}

// The ioctl numbers are hand-written constants, so recompute them from the
// _IOC encoding and the struct sizes rather than trusting the transcription.
func TestIoctlNumbers(t *testing.T) {
	const (
		dirWrite = 1
		dirRead  = 2
	)
	ioc := func(dir, typ, nr, size uintptr) uintptr {
		return dir<<30 | size<<16 | typ<<8 | nr
	}

	for _, c := range []struct {
		name string
		got  uintptr
		want uintptr
	}{
		{"GPIO_GET_CHIPINFO", ioctlChipInfo, ioc(dirRead, 0xB4, 0x01, unsafe.Sizeof(chipInfo{}))},
		{"GPIO_V2_GET_LINE", ioctlGetLine, ioc(dirRead|dirWrite, 0xB4, 0x07, unsafe.Sizeof(lineRequest{}))},
		{"GPIO_V2_LINE_SET_CONFIG", ioctlSetConfig, ioc(dirRead|dirWrite, 0xB4, 0x0D, unsafe.Sizeof(lineConfig{}))},
		{"GPIO_V2_LINE_GET_VALUES", ioctlGetValues, ioc(dirRead|dirWrite, 0xB4, 0x0E, unsafe.Sizeof(lineValues{}))},
		{"GPIO_V2_LINE_SET_VALUES", ioctlSetValues, ioc(dirRead|dirWrite, 0xB4, 0x0F, unsafe.Sizeof(lineValues{}))},
	} {
		if c.got != c.want {
			t.Errorf("%s = %#x, computed %#x", c.name, c.got, c.want)
		}
	}
}

// makeConfig's whole job is to say nothing about the lines it was not asked
// about, because that is what keeps a turnaround from disturbing SWCLK.
func TestConfigLeavesUnnamedLinesAlone(t *testing.T) {
	d, f := newBidir(t)
	d.PortOn()

	f.val[lineSWCLK] = true // mid-transfer, SWCLK is high
	d.SwdioOutDisable()

	if !f.val[lineSWCLK] {
		t.Error("turning SWDIO round pulled SWCLK low; the target would see an extra clock")
	}
	if !f.out[lineSWCLK] {
		t.Error("turning SWDIO round stopped SWCLK being an output")
	}
	if f.out[lineSWDIO] {
		t.Error("SWDIO is still an output after OutDisable")
	}
}

// ---------------- How a pin is written ----------------

// ParsePin is the one place that knows how a pin is spelled on a command line,
// so that two programs cannot disagree about it and drive different lines.
func TestParsePin(t *testing.T) {
	for _, c := range []struct {
		spec    string
		want    Pin
		wantErr bool
	}{
		// A bare number has no chip. What that means is the caller's to decide.
		{spec: "16", want: Pin{"", 16}},
		{spec: " 16 ", want: Pin{"", 16}},
		{spec: "-1", want: Pin{"", -1}},

		// The chip half is whatever the chip can be called.
		{spec: "gpiochip0:16", want: Pin{"gpiochip0", 16}},
		{spec: "0:16", want: Pin{"0", 16}},
		{spec: "pinctrl-rp1:16", want: Pin{"pinctrl-rp1", 16}},
		// Split at the last colon, so a path survives.
		{spec: "/dev/gpiochip0:16", want: Pin{"/dev/gpiochip0", 16}},
		{spec: " gpiochip2 : 5 ", want: Pin{"gpiochip2", 5}},

		{spec: ":16", wantErr: true},
		{spec: "gpiochip0:", wantErr: true},
		{spec: "sixteen", wantErr: true},
		{spec: "gpiochip0:sixteen", wantErr: true},
		{spec: "", wantErr: true},
	} {
		got, err := ParsePin(c.spec)
		if c.wantErr {
			if err == nil {
				t.Errorf("ParsePin(%q) = %v, want an error", c.spec, got)
			}
			continue
		}
		if err != nil {
			t.Errorf("ParsePin(%q): %v", c.spec, err)
			continue
		}
		if got != c.want {
			t.Errorf("ParsePin(%q) = %#v, want %#v", c.spec, got, c.want)
		}
	}
}

// ---------------- Constructors ----------------

func TestConstructorsRejectBadPins(t *testing.T) {
	for _, c := range []struct {
		name string
		w    wiring
	}{
		{"bidir: clock and data on one line",
			wiring{swclk: on(16), swdioIn: on(16), swdioOut: on(16), swdioDir: Pin{Line: -1}, shared: true}},
		{"bidir: line off the end of the chip",
			wiring{swclk: on(16), swdioIn: on(99), swdioOut: on(99), swdioDir: Pin{Line: -1}, shared: true}},
		{"bidir: negative line",
			wiring{swclk: on(16), swdioIn: on(-3), swdioOut: on(-3), swdioDir: Pin{Line: -1}, shared: true}},
		{"buffered: direction line doubling as data",
			wiring{swclk: on(16), swdioIn: on(20), swdioOut: on(20), swdioDir: on(20), shared: true}},
		{"buffered: clock and data on one line",
			wiring{swclk: on(16), swdioIn: on(16), swdioOut: on(16), swdioDir: on(21), shared: true}},
		{"split: direction line doubling as clock",
			wiring{swclk: on(16), swdioIn: on(12), swdioOut: on(20), swdioDir: on(16)}},
		{"split: in and out on one line",
			wiring{swclk: on(16), swdioIn: on(20), swdioOut: on(20), swdioDir: on(21)}},
		{"a chip that does not exist",
			wiring{swclk: Pin{"nope", 16}, swdioIn: on(12), swdioOut: on(20), swdioDir: on(21)}},
	} {
		h, _ := fakeHooks(t, testChip)
		claimed := false
		h.claim = func(Chip, *os.File, []uint32) (lineIO, error) {
			claimed = true
			return newFake(nil), nil
		}
		_, err := build(c.w, h)
		if claimed {
			t.Errorf("%s: claimed lines before validating them", c.name)
		}
		if err == nil {
			t.Errorf("%s: accepted, want an error", c.name)
		}
	}
}

func TestOpenNamesTheChipItCouldNotFind(t *testing.T) {
	_, err := OpenSplit("definitely-not-a-chip-label", 16, 12, 20, 21)
	if err == nil {
		t.Fatal("opening a chip that does not exist succeeded")
	}
	if !strings.Contains(err.Error(), "definitely-not-a-chip-label") {
		t.Errorf("error does not name the chip asked for: %v", err)
	}
}

// One line with two jobs is claimed once: the kernel refuses a request naming
// the same offset twice.
func TestSharedPinIsClaimedOnce(t *testing.T) {
	var claimed []uint32
	h, _ := fakeHooks(t, testChip)
	inner := h.claim
	h.claim = func(c Chip, f *os.File, offsets []uint32) (lineIO, error) {
		claimed = offsets
		return inner(c, f, offsets)
	}
	if _, err := build(wiring{swclk: on(lineSWCLK), swdioIn: on(lineSWDIO),
		swdioOut: on(lineSWDIO), swdioDir: on(lineSWDIODir), shared: true}, h); err != nil {
		t.Fatal(err)
	}

	seen := map[uint32]bool{}
	for _, o := range claimed {
		if seen[o] {
			t.Fatalf("line %d claimed twice in %v", o, claimed)
		}
		seen[o] = true
	}
	if len(claimed) != 3 {
		t.Fatalf("claimed %v, want three lines: clock, the shared data line, direction", claimed)
	}
}

// ---------------- Port and direction ----------------

func TestPortOnDrivesTheIdleLevels(t *testing.T) {
	t.Run("bidir idles SWDIO high", func(t *testing.T) {
		// With one pin there is no buffer to hold the line, so the host has to
		// drive the SWD idle level itself.
		d, f := newBidir(t)
		d.PortOn()
		if !f.out[lineSWDIO] || !f.val[lineSWDIO] {
			t.Errorf("SWDIO out=%v level=%v after PortOn, want an output driving high",
				f.out[lineSWDIO], f.val[lineSWDIO])
		}
		if !f.out[lineSWCLK] || f.val[lineSWCLK] {
			t.Errorf("SWCLK out=%v level=%v after PortOn, want an output driving low",
				f.out[lineSWCLK], f.val[lineSWCLK])
		}
	})

	t.Run("split keeps SWDIO in an input", func(t *testing.T) {
		d, f := newSplit(t)
		d.PortOn()
		if f.out[lineSWDIOIn] {
			t.Error("the SWDIO input line was made an output")
		}
		if !f.out[lineSWDIOOut] {
			t.Error("the SWDIO output line is not an output")
		}
		if !f.out[lineSWDIODir] || f.val[lineSWDIODir] {
			t.Error("the direction line should be an output, low, with the host driving")
		}
	})
}

func TestPortOffReleasesEveryLine(t *testing.T) {
	for _, c := range []struct {
		name  string
		open  func(*testing.T) (*Driver, *fakeChip)
		lines []uint32
	}{
		{"bidir", newBidir, []uint32{lineSWCLK, lineSWDIO}},
		{"bidir buffered", newBidirBuffered, []uint32{lineSWCLK, lineSWDIO, lineSWDIODir}},
		{"split", newSplit, []uint32{lineSWCLK, lineSWDIOIn, lineSWDIOOut, lineSWDIODir}},
	} {
		t.Run(c.name, func(t *testing.T) {
			d, f := c.open(t)
			d.PortOn()
			d.PortOff()
			for _, line := range c.lines {
				if f.out[line] {
					t.Errorf("line %d is still an output after PortOff", line)
				}
			}
		})
	}
}

// The order is what keeps the host pin and the buffer output from ever both
// driving the wire: point the buffer away from whichever side is about to stop
// driving, before it does.
func TestBufferedTurnaroundOrder(t *testing.T) {
	d, f := newBidirBuffered(t)
	d.PortOn()

	f.reset()
	d.SwdioOutDisable()
	if len(f.ops) != 2 || f.ops[0].kind != opConfig || f.ops[1].kind != opSet {
		t.Fatalf("OutDisable: %v, want the SWDIO line reconfigured first and the buffer turned after", f.ops)
	}
	if !f.val[lineSWDIODir] {
		t.Error("the direction line should be high while the target drives")
	}

	f.reset()
	d.SwdioOutEnable()
	if len(f.ops) != 2 || f.ops[0].kind != opSet || f.ops[1].kind != opConfig {
		t.Fatalf("OutEnable: %v, want the buffer turned first and the SWDIO line reconfigured after", f.ops)
	}
	if f.val[lineSWDIODir] {
		t.Error("the direction line should be low while the host drives")
	}
}

// The split wiring's data lines never move; turning the buffer round is the
// whole of a turnaround, which is one ioctl and no reconfiguration at all.
func TestSplitTurnaroundIsOneWrite(t *testing.T) {
	d, f := newSplit(t)
	d.PortOn()

	for _, c := range []struct {
		name string
		act  func()
		dir  bool
	}{
		{"OutDisable", d.SwdioOutDisable, true},
		{"OutEnable", d.SwdioOutEnable, false},
	} {
		f.reset()
		c.act()
		sets, gets, configs := f.counts()
		if sets != 1 || gets != 0 || configs != 0 {
			t.Errorf("%s took %d sets, %d gets, %d configs; want one set and nothing else",
				c.name, sets, gets, configs)
		}
		if f.val[lineSWDIODir] != c.dir {
			t.Errorf("%s left the direction line %v, want %v", c.name, f.val[lineSWDIODir], c.dir)
		}
		if !f.out[lineSWDIOOut] || f.out[lineSWDIOIn] {
			t.Errorf("%s moved a data line; on this wiring they never move", c.name)
		}
	}
}

// ---------------- Syscalls per bit ----------------

// This is the claim the whole driver is built around: SWDIO's level and SWCLK's
// falling edge travel in one ioctl, so a written bit costs two syscalls and not
// three. A driver claiming each signal as its own line cannot do this; if a
// change here ever splits them apart again, the clock drops by a third and
// nothing else would notice.
func TestWriteBitCostsTwoIoctls(t *testing.T) {
	for _, c := range []struct {
		name string
		open func(*testing.T) (*Driver, *fakeChip)
	}{
		{"bidir", newBidir},
		{"bidir buffered", newBidirBuffered},
		{"split", newSplit},
	} {
		t.Run(c.name, func(t *testing.T) {
			d, f := c.open(t)
			d.PortOn()

			f.reset()
			d.swWriteBit(1)
			sets, gets, configs := f.counts()
			if sets != 2 || gets != 0 || configs != 0 {
				t.Fatalf("a written bit took %d sets, %d gets, %d configs; want two sets",
					sets, gets, configs)
			}
			// The first of them has to carry both lines, or they are not fused.
			if want := d.clk.mask | d.dout.mask; f.ops[0].mask != want {
				t.Errorf("the first ioctl's mask is %#b, want %#b -- SWDIO and SWCLK are not moving together",
					f.ops[0].mask, want)
			}

			f.reset()
			d.SwdioOutDisable()
			f.reset()
			d.swReadBit()
			sets, gets, _ = f.counts()
			if sets != 2 || gets != 1 {
				t.Fatalf("a read bit took %d sets and %d gets; want two clock edges and one sample",
					sets, gets)
			}
		})
	}
}

// A SET_VALUES may not name a line that is an input. The fake refuses one, so
// this asserts the invariant by running a whole transfer -- every turnaround
// included -- and requiring that nothing was refused.
func TestNoWriteEverNamesAnInputLine(t *testing.T) {
	for _, c := range []struct {
		name string
		open func(*testing.T) (*Driver, *fakeChip)
	}{
		{"bidir", newBidir},
		{"bidir buffered", newBidirBuffered},
		{"split", newSplit},
	} {
		t.Run(c.name, func(t *testing.T) {
			d, f := c.open(t)
			d.PortOn()
			f.in[lineSWDIOIn] = true
			f.in[lineSWDIO] = true

			var data uint32
			d.SwdTransfer(0x8D, &data, nil)
			d.SwdWriteSequence(8, []byte{0xA5})
			d.SwdioOutDisable()
			d.SwdReadSequence(8, make([]byte, 1))
			d.SwdioOutEnable()
			d.SwjSequence(8, []byte{0xFF})

			if d.Err() != nil {
				t.Fatalf("the fake chip refused an ioctl: %v", d.Err())
			}
		})
	}
}

// ---------------- The waveform ----------------

// TestEveryWiringClocksTheSameTransfer is the cross-wiring comparison, and this
// is the package that can make it: the fake chip records every ioctl, so the
// number of clock edges is countable. drivers/bcm2835 has the same three copies
// of SwdTransfer and no way to check them -- GPSET0 and GPCLR0 are write-only
// masks its fake only remembers the last write to, so a copy that clocked the
// wrong number of recovery cycles would pass there unnoticed.
//
// Here, three wirings that disagree about how many times SWCLK moved is a
// failure, which is the only reason one shared copy of SwdTransfer is safe to
// have in the first place.
func TestEveryWiringClocksTheSameTransfer(t *testing.T) {
	const request = 0x8D // a DP read of register 0x04, with the wire's parity bit

	for _, level := range []bool{false, true} {
		type result struct {
			name   string
			ack    uint8
			edges  int
			stamps int
		}
		var results []result

		for _, c := range []struct {
			name string
			open func(*testing.T) (*Driver, *fakeChip)
			in   uint32
		}{
			{"bidir", newBidir, lineSWDIO},
			{"bidir buffered", newBidirBuffered, lineSWDIO},
			{"split", newSplit, lineSWDIOIn},
		} {
			d, f := c.open(t)
			d.PortOn()
			d.SwdConfigure(1, false)
			f.in[c.in] = level

			f.reset()
			stamped := 0
			var data uint32
			ack := d.SwdTransfer(request, &data, func() { stamped++ })

			results = append(results, result{c.name, ack, countClockEdges(f, d.clk.mask), stamped})
		}

		for i := 1; i < len(results); i++ {
			a, b := results[0], results[i]
			if a.ack != b.ack {
				t.Errorf("with SWDIO reading %v, %s answered ack %d and %s answered %d",
					level, a.name, a.ack, b.name, b.ack)
			}
			if a.edges != b.edges {
				t.Errorf("with SWDIO reading %v, %s clocked %d SWCLK edges and %s clocked %d",
					level, a.name, a.edges, b.name, b.edges)
			}
			if a.stamps != b.stamps {
				t.Errorf("with SWDIO reading %v, %s took %d timestamps and %s took %d",
					level, a.name, a.stamps, b.name, b.stamps)
			}
		}
		if results[0].edges == 0 {
			t.Fatal("no SWCLK edges were recorded at all; the trace is not being captured")
		}
	}
}

// countClockEdges counts the transitions of SWCLK in a recorded trace, which is
// what a scope would count.
func countClockEdges(f *fakeChip, clkMask uint64) int {
	edges, level := 0, -1
	for _, o := range f.ops {
		if o.kind != opSet || o.mask&clkMask == 0 {
			continue
		}
		next := 0
		if o.bits&clkMask != 0 {
			next = 1
		}
		if next != level {
			edges++
			level = next
		}
	}
	return edges
}

// TestTransferClocksExactlyTheRightCycles is the assertion drivers/bcm2835's
// package comment says its own fake cannot make: with the ioctl trace recorded,
// the number of SWCLK cycles a transfer puts on the wire is countable, so each
// path through SwdTransfer can be held to the count the SWD specification says
// it has. A recovery loop that ran 32 times instead of 33 would pass every
// other test here and fail this one.
//
// The counts are the protocol arithmetic, with a one-cycle turnaround:
//
//	request 8 + turnaround 1 + ack 3   = 12 before anything branches
//	a data phase, read or write        = 33 (32 bits and a parity bit)
//	a turnaround back to the host      = 1
func TestTransferClocksExactlyTheRightCycles(t *testing.T) {
	// The ACK arrives least significant bit first.
	ackOK := []bool{true, false, false}   // 0b001
	ackWAIT := []bool{false, true, false} // 0b010
	ackNone := []bool{true, true, true}   // not an ACK at all

	// A read's data phase: 32 bits and a parity bit, all zero.
	dataPhaseBits := make([]bool, 33)

	for _, c := range []struct {
		name       string
		request    uint8
		script     []bool
		dataPhase  bool
		wantCycles int
	}{
		{"read, OK", 0x02, append(append([]bool{}, ackOK...), dataPhaseBits...), false, 12 + 33 + 1},
		{"write, OK", 0x00, ackOK, false, 12 + 1 + 33},
		{"read, WAIT, no data phase", 0x02, ackWAIT, false, 12 + 1},
		{"read, WAIT, data phase", 0x02, ackWAIT, true, 12 + 33 + 1},
		{"write, WAIT, data phase", 0x00, ackWAIT, true, 12 + 1 + 33},
		{"no ACK at all", 0x02, ackNone, false, 12 + 1 + 33},
	} {
		t.Run(c.name, func(t *testing.T) {
			d, f := newSplit(t)
			d.PortOn()
			d.SwdConfigure(1, c.dataPhase)
			f.script = append([]bool{}, c.script...)

			f.reset()
			data := uint32(0)
			d.SwdTransfer(c.request, &data, nil)

			// Two transitions per cycle: SWCLK low, then high.
			if got := countClockEdges(f, d.clk.mask); got != 2*c.wantCycles {
				t.Errorf("clocked %d SWCLK cycles, want %d", got/2, c.wantCycles)
			}
			if d.Err() != nil {
				t.Errorf("Err() = %v", d.Err())
			}
		})
	}
}

// A read's 32 bits arrive least significant first and have to be reassembled
// that way; the fake's script is what lets a particular word be sent.
func TestReadTransferReassemblesTheWord(t *testing.T) {
	const want = uint32(0xDEADBEEF)

	script := []bool{true, false, false} // ACK OK
	parity := false
	for i := 0; i < 32; i++ {
		bit := want>>uint(i)&1 != 0
		script = append(script, bit)
		parity = parity != bit
	}
	script = append(script, parity)

	d, f := newSplit(t)
	d.PortOn()
	d.SwdConfigure(1, false)
	f.script = script

	var got uint32
	if ack := d.SwdTransfer(0x02, &got, nil); ack != dapTransferOK {
		t.Fatalf("ack = %d, want OK -- the parity bit was rejected", ack)
	}
	if got != want {
		t.Errorf("read %#x, want %#x", got, want)
	}
}

// Whatever happened, a transfer hands the line back to the host driving it
// high: the SWD idle level, and where the next transfer expects to start.
func TestTransferLeavesSWDIOIdleHigh(t *testing.T) {
	for _, c := range []struct {
		name string
		open func(*testing.T) (*Driver, *fakeChip)
		out  uint32
	}{
		{"bidir", newBidir, lineSWDIO},
		{"bidir buffered", newBidirBuffered, lineSWDIO},
		{"split", newSplit, lineSWDIOOut},
	} {
		t.Run(c.name, func(t *testing.T) {
			d, f := c.open(t)
			d.PortOn()
			var data uint32
			d.SwdTransfer(0x8D, &data, nil)

			if !f.out[c.out] {
				t.Fatal("SWDIO was not left with the host driving")
			}
			if !f.val[c.out] {
				t.Error("SWDIO was not left high")
			}
		})
	}
}

// ---------------- Sequences ----------------

// A run that is not a whole number of bytes has to end up right-aligned in the
// last byte, which is what the trailing shift in SwdReadSequence is for.
func TestReadSequencePacksBits(t *testing.T) {
	d, f := newSplit(t)
	d.PortOn()
	d.SwdioOutDisable()
	f.in[lineSWDIOIn] = true

	buf := make([]byte, 2)
	d.SwdReadSequence(12, buf)
	if buf[0] != 0xFF || buf[1] != 0x0F {
		t.Errorf("twelve high bits packed to %#x %#x, want 0xff 0x0f", buf[0], buf[1])
	}
}

func TestWriteSequenceSendsBitsLSBFirst(t *testing.T) {
	d, f := newSplit(t)
	d.PortOn()

	f.reset()
	d.SwdWriteSequence(4, []byte{0b1010})

	var got []int
	for _, o := range f.ops {
		if o.kind == opSet && o.mask&d.dout.mask != 0 && o.mask&d.clk.mask != 0 {
			// The fused write: SWDIO's level with SWCLK going low.
			bit := 0
			if o.bits&d.dout.mask != 0 {
				bit = 1
			}
			got = append(got, bit)
		}
	}
	want := []int{0, 1, 0, 1}
	if fmt.Sprint(got) != fmt.Sprint(want) {
		t.Errorf("clocked out %v, want %v -- bits go out least significant first", got, want)
	}
}

// ---------------- Errors ----------------

// An ioctl can fail where a register store cannot, and swd.BitBanger has
// nowhere to say so. The failure has to be kept, and the transfer has to come
// back rather than wedge.
func TestFailingIoctlIsRecordedAndTheTransferReturns(t *testing.T) {
	d, f := newSplit(t)
	d.PortOn()
	if d.Err() != nil {
		t.Fatalf("an error before anything failed: %v", d.Err())
	}

	f.failFrom = 1
	var data uint32
	ack := d.SwdTransfer(0x8D, &data, nil)

	if d.Err() == nil {
		t.Fatal("a failing ioctl was not recorded")
	}
	if !errors.Is(d.Err(), errInjected) {
		t.Errorf("Err() = %v, want the injected failure wrapped", d.Err())
	}
	if ack == dapTransferOK {
		t.Error("a transfer over a broken chip reported OK")
	}
}

func TestEmptyMaskIsNotSentToTheKernel(t *testing.T) {
	// The split wiring's SwdioSet while the target drives masks down to
	// nothing, and the kernel answers -EINVAL for a SET_VALUES naming no line.
	d, f := newBidir(t)
	d.PortOn()
	d.SwdioOutDisable()

	f.reset()
	d.SwdioSet()
	d.SwdioClr()
	if len(f.ops) != 0 {
		t.Errorf("%d ioctls sent for a write with nothing to write", len(f.ops))
	}
	if d.Err() != nil {
		t.Errorf("Err() = %v, want nothing: a write with no lines is not a failure", d.Err())
	}
}

// ---------------- Clock pacing ----------------

func TestUncalibratedReportsNothing(t *testing.T) {
	d, _ := newSplit(t)
	if coeff, offset := d.SpeedCoeffs(); coeff != 0 || offset != 0 {
		t.Fatalf("SpeedCoeffs() = %d, %d before any measurement, want 0, 0", coeff, offset)
	}
	d.SetFrequencyHz(2_000_000)
	if d.clockHz != 2_000_000 {
		t.Fatalf("clockHz = %d, want the requested 2000000 to be remembered", d.clockHz)
	}
}

func TestMeasureProducesPositiveCosts(t *testing.T) {
	d, _ := newSplit(t)
	d.PortOn()
	d.measure()

	if d.loopNs <= 0 {
		t.Errorf("loopNs = %v, want > 0", d.loopNs)
	}
	if d.toggleNs <= 0 {
		t.Errorf("toggleNs = %v, want > 0", d.toggleNs)
	}
}

// Calibrating before PortOn is the ordinary case -- it is the safe moment for
// it, and it is the order the gateway uses -- so the measurement has to drive a
// real output rather than an edge the kernel refuses or drops early. An edge on
// a floating line is cheaper than a real one, which would come out as a maximum
// clock the hardware cannot reach.
func TestCalibratingBeforePortOnDrivesRealEdges(t *testing.T) {
	d, f := newSplit(t)
	if d.portOn {
		t.Fatal("a driver arrives with its port on")
	}

	coeff, offset, err := d.Calibrate()
	if err != nil {
		t.Fatalf("Calibrate before PortOn: %v", err)
	}
	if d.Err() != nil {
		t.Fatalf("calibrating drove a line the kernel would have refused: %v", d.Err())
	}
	if coeff == 0 || offset == 0 {
		t.Fatalf("Calibrate() = %d, %d, want both non-zero", coeff, offset)
	}

	sets, _, _ := f.counts()
	if sets == 0 {
		t.Error("no SWCLK edges reached the chip at all")
	}
	// The line has to go back exactly as it was found: floating, for a target
	// that is not attached yet.
	if f.out[lineSWCLK] {
		t.Error("SWCLK was left an output by a measurement taken before PortOn")
	}
}

func TestFasterClockNeedsFewerIterations(t *testing.T) {
	d, _ := newSplit(t)
	d.loopNs, d.toggleNs = 2, 1000

	d.SetFrequencyHz(100_000)
	slow := d.iterations
	d.SetFrequencyHz(200_000)
	fast := d.iterations
	if fast > slow {
		t.Fatalf("200kHz needs %d iterations and 100kHz needs %d; a faster clock must need fewer", fast, slow)
	}
}

// On this backend an edge costs a syscall, so any clock near what the hardware
// would give has nothing left to pace. That is not an error -- it is the
// driver saying the kernel is the limit.
func TestClockAboveTheSyscallCostClampsToZero(t *testing.T) {
	d, _ := newSplit(t)
	d.loopNs, d.toggleNs = 2, 1000 // a microsecond an edge, which is realistic here

	d.SetFrequencyHz(1_000_000)
	if d.iterations != 0 {
		t.Fatalf("iterations = %d, want 0: a 500ns half period is below the cost of one ioctl", d.iterations)
	}
}

func TestDelayLoopIsNotOptimizedAway(t *testing.T) {
	d, _ := newSplit(t)
	d.delayLoop(1000)
	if d.sink != 1000 {
		t.Fatalf("sink = %d after delayLoop(1000), want 1000 -- the loop body did not run", d.sink)
	}
}

func TestSpeedCoeffRoundTrip(t *testing.T) {
	d, _ := newSplit(t)
	d.loopNs, d.toggleNs = 4.4, 1100
	d.SetFrequencyHz(200_000)

	coeff, offset := d.SpeedCoeffs()
	if coeff == 0 || offset == 0 {
		t.Fatalf("SpeedCoeffs() = %d, %d, want both non-zero", coeff, offset)
	}
	if offset >= coeff {
		t.Fatalf("offset %d >= coeff %d, which claims an unbounded maximum clock", offset, coeff)
	}
	want := d.iterations

	other, _ := newSplit(t)
	other.SetFrequencyHz(200_000)
	if err := other.SetSpeedCoeffs(coeff, offset); err != nil {
		t.Fatalf("SetSpeedCoeffs(%d, %d): %v", coeff, offset, err)
	}
	if diff := int(other.iterations) - int(want); diff > 2 || diff < -2 {
		t.Fatalf("round trip gave %d delay iterations, want about %d", other.iterations, want)
	}
}

func TestSetSpeedCoeffsRejectsNonsense(t *testing.T) {
	d, _ := newSplit(t)
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

// ---------------- Pins on more than one chip ----------------
//
// Each pin names its own chip, so a wiring with its data pins on the SoC and
// its buffer direction line on an I2C expander is supported. These are that.
//
// What it costs depends entirely on which signal moved. The direction line is
// already its own ioctl, so putting it on another chip costs nothing at all.
// SWDIO is not: it rides along with SWCLK's falling edge in one ioctl, and
// lines in different requests cannot share one.

// openWorld builds a driver over two chips.
func openWorld(t *testing.T, w wiring) (*Driver, map[string]*fakeChip) {
	t.Helper()
	h, fakes := fakeHooks(t, testChip, testExpander)
	d, err := build(w, h)
	if err != nil {
		t.Fatalf("build: %v", err)
	}
	return d, fakes
}

// expander names a pin on the second chip.
func expander(line int) Pin { return Pin{testExpander.Name, line} }

// A direction line on another chip is free: a turnaround was always an ioctl of
// its own, so it lands on the other chip and nothing else changes.
func TestDirectionLineOnAnotherChipCostsNothing(t *testing.T) {
	d, fakes := openWorld(t, wiring{
		swclk: on(lineSWCLK), swdioIn: on(lineSWDIOIn), swdioOut: on(lineSWDIOOut),
		swdioDir: expander(5),
	})
	if !d.Fused() {
		t.Fatal("SWDIO and SWCLK are still on one chip; they should still be fused")
	}
	d.PortOn()

	main, exp := fakes[testChip.Name], fakes[testExpander.Name]
	if main == nil || exp == nil {
		t.Fatalf("wanted a request on each chip, got %v", fakes)
	}
	if !exp.out[5] || exp.val[5] {
		t.Error("the direction line on the expander was not driven low by PortOn")
	}

	main.reset()
	exp.reset()
	d.swWriteBit(1)
	if sets, _, _ := main.counts(); sets != 2 {
		t.Errorf("a written bit took %d ioctls on the data chip, want 2", sets)
	}
	if len(exp.ops) != 0 {
		t.Errorf("a written bit touched the expander %d times, want 0", len(exp.ops))
	}

	main.reset()
	exp.reset()
	d.SwdioOutDisable()
	if len(main.ops) != 0 {
		t.Errorf("a turnaround touched the data chip %d times, want 0", len(main.ops))
	}
	if sets, _, _ := exp.counts(); sets != 1 {
		t.Errorf("a turnaround took %d ioctls on the expander, want 1", sets)
	}
	if !exp.val[5] {
		t.Error("the direction line should be high while the target drives")
	}
}

// SWDIO on another chip is not free, and the driver has to say so rather than
// quietly sending a SET_VALUES to the wrong request. The bit costs three
// ioctls instead of two, and SWDIO still has to be settled before SWCLK falls.
func TestSwdioOnAnotherChipCostsAnExtraIoctl(t *testing.T) {
	d, fakes := openWorld(t, wiring{
		swclk: on(lineSWCLK), swdioIn: on(lineSWDIOIn), swdioOut: expander(3),
		swdioDir: on(lineSWDIODir),
	})
	if d.Fused() {
		t.Fatal("SWDIO and SWCLK are on different chips and cannot be fused")
	}
	d.PortOn()

	main, exp := fakes[testChip.Name], fakes[testExpander.Name]
	main.reset()
	exp.reset()
	d.swWriteBit(1)

	if sets, _, _ := exp.counts(); sets != 1 {
		t.Errorf("SWDIO took %d ioctls, want 1", sets)
	}
	if sets, _, _ := main.counts(); sets != 2 {
		t.Errorf("SWCLK took %d ioctls, want 2", sets)
	}
	if !exp.val[3] {
		t.Error("SWDIO was not driven high")
	}
	// SWCLK's falling edge is what the target latches on, so SWDIO has to be
	// there first. This is the one thing fusing them used to guarantee for
	// free.
	if exp.ops[0].at > main.ops[0].at {
		t.Errorf("SWDIO was written at %d, after SWCLK fell at %d",
			exp.ops[0].at, main.ops[0].at)
	}

	// While the target owns SWDIO, its line is not written at all -- the same
	// rule the fused path keeps with writeMask.
	main.reset()
	exp.reset()
	d.SwdioOutDisable()
	exp.reset()
	d.swReadBit()
	if len(exp.ops) != 0 {
		t.Errorf("SWDIO's chip was written %d times during a read bit, want 0", len(exp.ops))
	}
	if d.Err() != nil {
		t.Fatalf("the fake chip refused an ioctl: %v", d.Err())
	}
}

// A transfer has to put the same thing on the wire however the pins are spread
// about. The cycle counts are the ones TestTransferClocksExactlyTheRightCycles
// pins down for a single chip.
func TestSpreadingThePinsDoesNotChangeTheWaveform(t *testing.T) {
	for _, c := range []struct {
		name string
		w    wiring
	}{
		{"one chip", wiring{swclk: on(lineSWCLK), swdioIn: on(lineSWDIOIn),
			swdioOut: on(lineSWDIOOut), swdioDir: on(lineSWDIODir)}},
		{"direction on an expander", wiring{swclk: on(lineSWCLK), swdioIn: on(lineSWDIOIn),
			swdioOut: on(lineSWDIOOut), swdioDir: expander(5)}},
		{"swdio out on an expander", wiring{swclk: on(lineSWCLK), swdioIn: on(lineSWDIOIn),
			swdioOut: expander(3), swdioDir: on(lineSWDIODir)}},
	} {
		t.Run(c.name, func(t *testing.T) {
			d, fakes := openWorld(t, c.w)
			d.PortOn()
			d.SwdConfigure(1, false)
			fakes[testChip.Name].script = []bool{true, false, false} // ACK OK

			fakes[testChip.Name].reset()
			var data uint32
			if ack := d.SwdTransfer(0x02, &data, nil); ack != dapTransferOK {
				t.Fatalf("ack = %d, want OK", ack)
			}
			// request 8 + turnaround 1 + ack 3 + data 33 + turnaround 1.
			if got := countClockEdges(fakes[testChip.Name], d.clk.mask); got != 2*46 {
				t.Errorf("clocked %d SWCLK cycles, want 46", got/2)
			}
			if d.Err() != nil {
				t.Errorf("Err() = %v", d.Err())
			}
		})
	}
}

// Two spellings of one chip must not become two requests: the second would be
// refused for any line the first already holds, and splitting a wiring across
// two requests on one chip would give up the fusion for nothing.
func TestOneChipUnderTwoNamesIsClaimedOnce(t *testing.T) {
	h, fakes := fakeHooks(t, testChip, testExpander)
	claims := 0
	inner := h.claim
	h.claim = func(c Chip, f *os.File, offsets []uint32) (lineIO, error) {
		claims++
		return inner(c, f, offsets)
	}

	d, err := build(wiring{
		swclk:    Pin{"gpiochip0", lineSWCLK},
		swdioIn:  Pin{"/dev/gpiochip0", lineSWDIOIn},
		swdioOut: Pin{"fake", lineSWDIOOut}, // the label
		swdioDir: Pin{"0", lineSWDIODir},    // the number
	}, h)
	if err != nil {
		t.Fatalf("build: %v", err)
	}
	if claims != 1 {
		t.Errorf("claimed %d requests for four names of one chip, want 1", claims)
	}
	if len(fakes) != 1 {
		t.Errorf("opened %d chips, want 1", len(fakes))
	}
	if !d.Fused() {
		t.Error("pins on one chip spelled four ways did not end up fused")
	}
}

// Line numbers only collide within a chip. The same number on two chips is two
// different pins, and rejecting it would rule out most expander wirings.
func TestSameLineNumberOnDifferentChipsIsFine(t *testing.T) {
	if _, err := build(wiring{
		swclk: on(5), swdioIn: on(12), swdioOut: on(20), swdioDir: expander(5),
	}, func() hooks { h, _ := fakeHooks(t, testChip, testExpander); return h }()); err != nil {
		t.Errorf("line 5 on two different chips was rejected: %v", err)
	}
}

func TestWiringReportsWhereThePinsLanded(t *testing.T) {
	d, _ := openWorld(t, wiring{
		swclk: on(lineSWCLK), swdioIn: on(lineSWDIOIn), swdioOut: on(lineSWDIOOut),
		swdioDir: expander(5),
	})
	want := "swclk gpiochip0:16, swdio in gpiochip0:12, out gpiochip0:20, dir gpiochip9:5"
	if got := d.Wiring(); got != want {
		t.Errorf("Wiring() = %q, want %q", got, want)
	}
}
