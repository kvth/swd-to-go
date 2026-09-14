//go:build linux

package linuxgpiod

import (
	"os"
	"syscall"
	"testing"
	"unsafe"
)

// What an edge costs here, and what that buys back.
//
// The question this package exists to answer is whether a GPIO driver can be
// written generically -- one body of code over any pins on any board -- without
// giving up the speed drivers/bcm2835 goes to such lengths for. These
// benchmarks are the evidence for the answer, which is that the two questions
// turn out not to be connected: what abstraction costs and what this driver
// costs are three orders of magnitude apart.
//
// BenchmarkIoctlFloor is the floor: one ioctl that the kernel rejects
// immediately with EBADF, so it measures a syscall round trip and nothing else.
// A real GPIO_V2_LINE_SET_VALUES is that plus the gpiolib work and, on a Pi 5,
// a PCIe write to RP1 -- the floor is a lower bound, not an estimate.
//
// BenchmarkDispatch* is the cost of getting to a pin three ways: a direct call
// on a concrete type, a call through an interface, and a call through a generic
// type parameter. Compare them against the floor above before reaching for any
// of it.
//
// Run: go test ./drivers/linuxgpiod -run XXX -bench .

// BenchmarkIoctlFloor is one syscall, with no driver work behind it.
func BenchmarkIoctlFloor(b *testing.B) {
	var vals lineValues
	for i := 0; i < b.N; i++ {
		// Fd -1 fails in the kernel's fd lookup, which is as early as an
		// ioctl can fail: entry, lookup, EBADF, exit.
		syscall.Syscall(syscall.SYS_IOCTL, ^uintptr(0), ioctlSetValues, uintptr(unsafe.Pointer(&vals)))
	}
}

// The three ways to reach a pin.

type pinStore interface{ store(uint32) }

type concretePin struct{ v uint32 }

func (p *concretePin) store(x uint32) { p.v = x }

// ifacePin is package-level and interface-typed so the compiler cannot see the
// concrete type at the call site and devirtualise the call away. A benchmark
// that lets it do so measures the direct call twice.
var ifacePin pinStore = &concretePin{}

func viaGeneric[T pinStore](p T, x uint32) { p.store(x) }

func BenchmarkDispatchDirect(b *testing.B) {
	p := &concretePin{}
	for i := 0; i < b.N; i++ {
		p.store(uint32(i))
	}
}

func BenchmarkDispatchInterface(b *testing.B) {
	p := ifacePin
	for i := 0; i < b.N; i++ {
		p.store(uint32(i))
	}
}

func BenchmarkDispatchGeneric(b *testing.B) {
	p := &concretePin{}
	for i := 0; i < b.N; i++ {
		viaGeneric(p, uint32(i))
	}
}

// nullLines is a lineIO that does nothing, so that BenchmarkWriteBit measures
// the driver rather than a test double. The recording fake the tests use keeps
// its state in maps, which would be most of what got measured here.
type nullLines struct{ sink uint64 }

func (n *nullLines) setValues(bits, mask uint64) error     { n.sink = bits | mask; return nil }
func (n *nullLines) getValues(mask uint64) (uint64, error) { return mask & 1, nil }
func (n *nullLines) setConfig(*lineConfig) error           { return nil }
func (n *nullLines) close() error                          { return nil }

// BenchmarkWriteBit is everything a written SWD bit costs except the syscalls:
// the mask arithmetic, the error check, the interface call into lineIO, and the
// delay loop doing nothing.
//
// Two of BenchmarkIoctlFloor is what the same bit costs in the kernel. The
// ratio between them is the honest answer to "is the abstraction here
// affordable" -- and it is also why there is no point optimising anything in
// this file: the syscalls are the driver.
func BenchmarkWriteBit(b *testing.B) {
	d, err := build(
		wiring{swclk: on(lineSWCLK), swdioIn: on(lineSWDIOIn),
			swdioOut: on(lineSWDIOOut), swdioDir: on(lineSWDIODir)},
		hooks{
			openChip: func(string) (*os.File, Chip, error) { return nil, testChip, nil },
			claim:    func(Chip, *os.File, []uint32) (lineIO, error) { return &nullLines{}, nil },
		})
	if err != nil {
		b.Fatal(err)
	}
	d.PortOn()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		d.swWriteBit(uint32(i) & 1)
	}
}
