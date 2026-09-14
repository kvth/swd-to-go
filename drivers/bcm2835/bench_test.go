package bcm2835

import (
	"sync/atomic"
	"testing"
	"unsafe"
)

// What a clock edge costs, and where the cost is.
//
// These exist because the obvious optimisation here is not one, and without
// the numbers somebody will keep reaching for it. The driver resolves each pin
// to its registers once at open rather than deriving them from a pin number on
// every edge (see the [pin] type); Resolved and Computed below are those two
// ways of doing exactly the same store, and they measure the same. The whole
// address computation -- a divide, a modulo, a shift and the slice bounds
// check -- is under a nanosecond.
//
// The Atomic/Plain pair is where the time actually goes. The barrier is around
// ten times everything else in an edge put together, and it is not removable:
// without it the CPU's write buffer may merge two stores to GPSET0 or GPCLR0
// into one and swallow a clock edge. So the ceiling on this driver's SWD clock
// is set by how fast the machine can retire a release store, and no amount of
// tidying the code around it moves that.
//
// Run: go test ./drivers/bcm2835 -run XXX -bench .

var benchSink uint32

func benchPin(b *testing.B) (*core, pin) {
	b.Helper()
	c := testCore(pinSWCLK, pinSWDIO, pinSWDIO)
	return &c, c.clk
}

// BenchmarkEdge is the real thing: one SWCLK low/high pair, barrier included.
func BenchmarkEdge(b *testing.B) {
	c, _ := benchPin(b)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		c.SwclkClr()
		c.SwclkSet()
	}
}

// BenchmarkWriteBit is one SWD bit at the maximum clock: the data pin, both
// clock edges, and two delay loops that are doing nothing.
func BenchmarkWriteBit(b *testing.B) {
	c, _ := benchPin(b)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		c.swWriteBit(uint32(i) & 1)
	}
}

func BenchmarkResolvedAtomic(b *testing.B) {
	_, p := benchPin(b)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		atomic.StoreUint32(p.set, p.mask)
	}
}

func BenchmarkComputedAtomic(b *testing.B) {
	c, _ := benchPin(b)
	g, n := c.regs, uint(pinSWCLK)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		atomic.StoreUint32((*uint32)(unsafe.Pointer(&g.mem[gpset0+uintptr(n/32)*4])), 1<<(n%32))
	}
}

func BenchmarkResolvedPlain(b *testing.B) {
	_, p := benchPin(b)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		*p.set = p.mask
	}
	benchSink = *p.set
}

func BenchmarkComputedPlain(b *testing.B) {
	c, _ := benchPin(b)
	g, n := c.regs, uint(pinSWCLK)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		*(*uint32)(unsafe.Pointer(&g.mem[gpset0+uintptr(n/32)*4])) = 1 << (n % 32)
	}
	benchSink = *(*uint32)(unsafe.Pointer(&g.mem[gpset0]))
}
