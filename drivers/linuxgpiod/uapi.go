//go:build linux

package linuxgpiod

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
	"syscall"
	"unsafe"
)

// The Linux GPIO character device, v2 uAPI -- <linux/gpio.h>, the interface
// libgpiod v2 is a wrapper over. It is spoken here directly rather than through
// that library for three reasons:
//
//   - cgo. Everything in this module except the USB HID transport is pure Go,
//     which is what lets `make pi` cross-build for a Raspberry Pi on any
//     machine with no toolchain at all. Linking libgpiod would take that away
//     from the one binary that most wants it.
//   - Version skew. libgpiod v1 and v2 are different APIs, and distributions
//     ship both, and a driver on top of it needs shims to paper over the
//     difference. The kernel uAPI underneath is one interface with a
//     stability guarantee, so there is nothing to paper over.
//   - Control of the payload. libgpiod's line-value calls take one line at a
//     time. GPIO_V2_LINE_SET_VALUES takes a bitmap, so SWCLK and SWDIO can
//     move in a single ioctl -- see [core.swWriteBit], which is where a third
//     of this driver's syscalls went.
//
// The v1 ioctls (GPIO_GET_LINEHANDLE_IOCTL and friends) are deliberately not
// here. v2 landed in Linux 5.10, in November 2020; a kernel old enough to need
// v1 is old enough not to be running a Pi 5, which is the board this driver
// exists for.

const (
	gpioMaxNameSize       = 32
	gpioV2LinesMax        = 64
	gpioV2LineNumAttrsMax = 10

	// gpio_v2_line_flag
	lineFlagInput  = 1 << 2
	lineFlagOutput = 1 << 3

	// gpio_v2_line_attr_id
	attrIDFlags        = 1
	attrIDOutputValues = 2
)

// The ioctl request numbers, which encode the size of the struct they carry.
// TestIoctlNumbers recomputes them from the _IOC encoding and the struct sizes
// rather than trusting the transcription.
//
// They are the same on every architecture this module is built for -- arm,
// arm64, amd64 and riscv64 all use the asm-generic _IOC layout. Alpha, MIPS,
// PowerPC and SPARC do not, and would need a per-architecture table; nothing
// here runs on one, and TestIoctlNumbers would pass on one regardless, since it
// recomputes with the same encoding it is checking. Worth knowing before
// porting, not worth a table nobody would exercise.
const (
	ioctlChipInfo  = 0x8044B401 // GPIO_GET_CHIPINFO_IOCTL
	ioctlGetLine   = 0xC250B407 // GPIO_V2_GET_LINE_IOCTL
	ioctlSetConfig = 0xC110B40D // GPIO_V2_LINE_SET_CONFIG_IOCTL
	ioctlGetValues = 0xC010B40E // GPIO_V2_LINE_GET_VALUES_IOCTL
	ioctlSetValues = 0xC010B40F // GPIO_V2_LINE_SET_VALUES_IOCTL
)

// struct gpiochip_info
type chipInfo struct {
	name  [gpioMaxNameSize]byte
	label [gpioMaxNameSize]byte
	lines uint32
}

// struct gpio_v2_line_values
type lineValues struct {
	bits uint64
	mask uint64
}

// struct gpio_v2_line_attribute. The kernel's union of flags, output values and
// a debounce period is one 64-bit field here, because all three are read and
// written as one.
type lineAttribute struct {
	id      uint32
	padding uint32
	value   uint64
}

// struct gpio_v2_line_config_attribute
type lineConfigAttribute struct {
	attr lineAttribute
	mask uint64
}

// struct gpio_v2_line_config
type lineConfig struct {
	flags    uint64
	numAttrs uint32
	padding  [5]uint32
	attrs    [gpioV2LineNumAttrsMax]lineConfigAttribute
}

// struct gpio_v2_line_request
type lineRequest struct {
	offsets         [gpioV2LinesMax]uint32
	consumer        [gpioMaxNameSize]byte
	config          lineConfig
	numLines        uint32
	eventBufferSize uint32
	padding         [5]uint32
	fd              int32
}

// Every field above is fixed-width and the kernel's 64-bit ones are
// __aligned_u64, so these sizes hold on a 32-bit build as well as a 64-bit one.
// A mistyped field stops the build here rather than sending the kernel a
// malformed ioctl, which it would answer with a plain EINVAL.
const (
	_ = uint(unsafe.Sizeof(chipInfo{}) - 68)
	_ = uint(68 - unsafe.Sizeof(chipInfo{}))
	_ = uint(unsafe.Sizeof(lineValues{}) - 16)
	_ = uint(16 - unsafe.Sizeof(lineValues{}))
	_ = uint(unsafe.Sizeof(lineConfig{}) - 272)
	_ = uint(272 - unsafe.Sizeof(lineConfig{}))
	_ = uint(unsafe.Sizeof(lineRequest{}) - 592)
	_ = uint(592 - unsafe.Sizeof(lineRequest{}))
)

func ioctl(fd int, req uintptr, arg unsafe.Pointer) error {
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, uintptr(fd), req, uintptr(arg)); errno != 0 {
		return errno
	}
	return nil
}

func cstr(b []byte) string {
	if i := strings.IndexByte(string(b), 0); i >= 0 {
		return string(b[:i])
	}
	return string(b)
}

// ---------------- Chips ----------------

// Chip is one /dev/gpiochipN, as it describes itself.
type Chip struct {
	Path  string // /dev/gpiochip0
	Name  string // gpiochip0
	Label string // pinctrl-rp1 on a Pi 5, pinctrl-bcm2711 on a Pi 4
	Lines uint32 // how many GPIOs it has
}

func (c Chip) String() string {
	return fmt.Sprintf("%s (%s, %d lines)", c.Name, c.Label, c.Lines)
}

// Chips lists the GPIO chips this machine has.
//
// It exists because the header pins are not always on the same chip: they are
// gpiochip0 labelled pinctrl-bcm2711 on a Pi 4 and gpiochip0 labelled
// pinctrl-rp1 on a Pi 5, but a Pi 5 running an older kernel put RP1 at
// gpiochip4, and a board with an I2C expander has chips that are not the header
// at all. Guessing a number is how a driver ends up wiggling something else, so
// this is here to be looked at.
func Chips() ([]Chip, error) {
	paths, err := filepath.Glob("/dev/gpiochip*")
	if err != nil {
		return nil, err
	}
	sort.Strings(paths)

	var out []Chip
	for _, p := range paths {
		// Read-only: describing a chip does not need the write access
		// claiming lines on it does, so a listing shows more than a driver
		// could use rather than less.
		f, err := os.Open(p)
		if err != nil {
			// A chip this user cannot even open is one they cannot drive;
			// leaving it out is the honest answer, and openChip's error says
			// what the listing showed.
			continue
		}
		c, err := describe(f, p)
		f.Close()
		if err != nil {
			continue
		}
		out = append(out, c)
	}
	return out, nil
}

func describe(f *os.File, path string) (Chip, error) {
	var info chipInfo
	if err := ioctl(int(f.Fd()), ioctlChipInfo, unsafe.Pointer(&info)); err != nil {
		return Chip{}, fmt.Errorf("GPIO_GET_CHIPINFO on %s: %w", path, err)
	}
	return Chip{
		Path:  path,
		Name:  cstr(info.name[:]),
		Label: cstr(info.label[:]),
		Lines: info.lines,
	}, nil
}

// openChip resolves a chip given as a number ("0"), a name ("gpiochip0"), a
// path ("/dev/gpiochip0") or a label ("pinctrl-rp1"), and opens it.
func openChip(name string) (*os.File, Chip, error) {
	path := ""
	switch {
	case name == "":
		return nil, Chip{}, fmt.Errorf("linuxgpiod: no gpiochip named")
	case strings.HasPrefix(name, "/"):
		path = name
	case strings.HasPrefix(name, "gpiochip"):
		path = "/dev/" + name
	default:
		if _, err := strconv.Atoi(name); err == nil {
			path = "/dev/gpiochip" + name
		}
	}

	if path == "" {
		// A label, then. Look for it.
		chips, err := Chips()
		if err != nil {
			return nil, Chip{}, err
		}
		for _, c := range chips {
			if c.Label == name {
				path = c.Path
				break
			}
		}
		if path == "" {
			return nil, Chip{}, fmt.Errorf("linuxgpiod: no gpiochip is named or labelled %q; this machine has %s",
				name, describeAll())
		}
	}

	f, err := os.OpenFile(path, os.O_RDWR, 0)
	if err != nil {
		// os.OpenFile's error already names the path and the operation, so
		// only the part it cannot know is added here.
		return nil, Chip{}, fmt.Errorf("%w (need root, or membership of the gpio group)", err)
	}
	c, err := describe(f, path)
	if err != nil {
		f.Close()
		return nil, Chip{}, err
	}
	return f, c, nil
}

func describeAll() string {
	chips, err := Chips()
	if err != nil || len(chips) == 0 {
		return "none it can open"
	}
	parts := make([]string, 0, len(chips))
	for _, c := range chips {
		parts = append(parts, c.String())
	}
	return strings.Join(parts, ", ")
}

// ---------------- Line requests ----------------

// lineIO is one claimed group of GPIOs: the three calls the driver makes on the
// kernel, and the seam the tests replace with an in-memory chip.
//
// It is an interface, which everything else in this module's bit-banging path
// is careful not to be. It is affordable exactly here and nowhere else: the
// implementation behind it performs a syscall, so the indirect call is a
// couple of nanoseconds in front of a microsecond of kernel work.
// BenchmarkDispatch and BenchmarkIoctlFloor measure both halves of that claim.
type lineIO interface {
	// setValues drives the lines selected by mask to the levels in bits. Both
	// are indexed by position in the request, not by GPIO number.
	setValues(bits, mask uint64) error
	// getValues samples the lines selected by mask.
	getValues(mask uint64) (uint64, error)
	// setConfig changes line directions. Lines whose flags carry no direction
	// are left exactly as they are, which is what makes a turnaround a
	// one-line operation on a request holding four.
	setConfig(cfg *lineConfig) error
	close() error
}

// kernelLines is a lineIO over a real GPIO_V2_GET_LINE file descriptor.
type kernelLines struct {
	fd int

	// Reused across calls so that a clock edge allocates nothing. The driver
	// is single-threaded by construction -- swd.BitBanger is a pin-wiggling
	// interface, not a concurrent one.
	vals lineValues
}

// requestLines claims offsets on chip as one request. They come back as inputs;
// PortOn is what makes any of them an output.
//
// One request for every pin, rather than one per pin the way libgpiod's API
// leads you to, is the whole reason this driver is worth writing: the lines in
// a single request share one GPIO_V2_LINE_SET_VALUES bitmap, so SWDIO and SWCLK
// move together in one syscall instead of two.
func requestLines(chip *os.File, consumer string, offsets []uint32) (*kernelLines, error) {
	if len(offsets) == 0 || len(offsets) > gpioV2LinesMax {
		return nil, fmt.Errorf("linuxgpiod: %d lines requested, want 1..%d", len(offsets), gpioV2LinesMax)
	}

	req := lineRequest{
		numLines: uint32(len(offsets)),
		config:   lineConfig{flags: lineFlagInput},
	}
	copy(req.consumer[:gpioMaxNameSize-1], consumer)
	copy(req.offsets[:], offsets)

	if err := ioctl(int(chip.Fd()), ioctlGetLine, unsafe.Pointer(&req)); err != nil {
		return nil, fmt.Errorf("GPIO_V2_GET_LINE for lines %v: %w "+
			"(EBUSY means something else holds one of them -- a device-tree overlay, "+
			"a running OpenOCD, or another copy of this program)", offsets, err)
	}
	return &kernelLines{fd: int(req.fd)}, nil
}

func (k *kernelLines) setValues(bits, mask uint64) error {
	k.vals = lineValues{bits: bits, mask: mask}
	return ioctl(k.fd, ioctlSetValues, unsafe.Pointer(&k.vals))
}

func (k *kernelLines) getValues(mask uint64) (uint64, error) {
	k.vals = lineValues{mask: mask}
	if err := ioctl(k.fd, ioctlGetValues, unsafe.Pointer(&k.vals)); err != nil {
		return 0, err
	}
	return k.vals.bits, nil
}

func (k *kernelLines) setConfig(cfg *lineConfig) error {
	return ioctl(k.fd, ioctlSetConfig, unsafe.Pointer(cfg))
}

func (k *kernelLines) close() error {
	if k.fd < 0 {
		return nil
	}
	fd := k.fd
	k.fd = -1
	return syscall.Close(fd)
}

// ---------------- Config payloads ----------------

// makeConfig builds a GPIO_V2_LINE_SET_CONFIG payload that makes the lines in
// outputs outputs at the levels in values, makes the lines in inputs inputs,
// and says nothing at all about the rest.
//
// Saying nothing is the point. The kernel skips any line whose flags carry no
// direction bit -- "Lines not explicitly reconfigured as input or output are
// left unchanged", linereq_set_config() in gpiolib-cdev.c -- so a turnaround
// that flips SWDIO cannot disturb SWCLK, even though both are in the same
// request. Without that guarantee a turnaround would have to restate SWCLK's
// level, and getting it wrong would put an extra clock on the wire.
func makeConfig(outputs, inputs, values uint64) lineConfig {
	cfg := lineConfig{}
	if outputs != 0 {
		cfg.attrs[cfg.numAttrs] = lineConfigAttribute{
			attr: lineAttribute{id: attrIDFlags, value: lineFlagOutput},
			mask: outputs,
		}
		cfg.numAttrs++
		// An output line comes up at whatever OUTPUT_VALUES says, and at zero
		// when nothing says anything: a direction change is also a level
		// change, so the level has to travel with it.
		cfg.attrs[cfg.numAttrs] = lineConfigAttribute{
			attr: lineAttribute{id: attrIDOutputValues, value: values},
			mask: outputs,
		}
		cfg.numAttrs++
	}
	if inputs != 0 {
		cfg.attrs[cfg.numAttrs] = lineConfigAttribute{
			attr: lineAttribute{id: attrIDFlags, value: lineFlagInput},
			mask: inputs,
		}
		cfg.numAttrs++
	}
	return cfg
}
