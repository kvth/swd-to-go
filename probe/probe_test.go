package probe_test

import (
	"context"
	"testing"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/cmsis/client"
	"github.com/kvth/swd-to-go/cmsis/server"
	"github.com/kvth/swd-to-go/cmsis/transport/inproc"
	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
)

// The point of swd.Probe is that a bitbang engine and a CMSIS-DAP client are
// interchangeable. These tests drive the same simulated target both ways and
// require the same answers: directly through the engine, and through the engine
// with a CMSIS-DAP server and client sandwiched in between.
//
// That sandwich is not a contrivance. It is exactly the path the old code took
// for locally wired hardware, where every in-process transfer was encoded into
// a packet and decoded again; the first case is what that path costs nothing to
// become now.

const (
	ramBase = 0x20000000
	ramSize = 0x2000
)

type probeFactory struct {
	name string
	open func(t *testing.T, target *simtarget.Target) swd.Probe
}

func factories() []probeFactory {
	return []probeFactory{
		{
			name: "bitbang",
			open: func(t *testing.T, target *simtarget.Target) swd.Probe {
				return bitbang.New(target)
			},
		},
		{
			name: "cmsis-dap",
			open: func(t *testing.T, target *simtarget.Target) swd.Probe {
				t.Helper()
				srv := server.New(context.Background(), bitbang.New(target), server.Options{})
				p, err := inproc.Open(context.Background(), srv)
				if err != nil {
					t.Fatalf("open CMSIS-DAP probe: %v", err)
				}
				return p
			},
		},
	}
}

// attach brings a probe up to the point where target memory can be reached.
func attach(t *testing.T, p swd.Probe) *mem.Client {
	t.Helper()
	ctx := context.Background()

	if err := p.Connect(ctx); err != nil {
		t.Fatalf("connect: %v", err)
	}
	if err := p.Configure(ctx, swd.Config{WaitRetry: 100, Turnaround: 1}); err != nil {
		t.Fatalf("configure: %v", err)
	}

	dpc := dp.New(p)
	if err := dpc.Init(ctx); err != nil {
		t.Fatalf("DP init: %v", err)
	}
	if err := dpc.SetDbgPower(ctx, true, true); err != nil {
		t.Fatalf("power up: %v", err)
	}

	apc := mem.New(dpc, 0)
	if err := apc.Init(ctx); err != nil {
		t.Fatalf("MEM-AP init: %v", err)
	}
	return apc
}

func newTarget() *simtarget.Target {
	return simtarget.New(simtarget.Options{},
		&simtarget.Region{Base: ramBase, Data: make([]byte, ramSize)})
}

func TestSingleWordRoundTrip(t *testing.T) {
	for _, f := range factories() {
		t.Run(f.name, func(t *testing.T) {
			ctx := context.Background()
			apc := attach(t, f.open(t, newTarget()))

			if err := apc.WriteTargetReg(ctx, ramBase+0x40, 0xDEADBEEF); err != nil {
				t.Fatalf("write: %v", err)
			}
			got, err := apc.ReadTargetReg(ctx, ramBase+0x40)
			if err != nil {
				t.Fatalf("read: %v", err)
			}
			if got != 0xDEADBEEF {
				t.Fatalf("read back 0x%08x, want 0xdeadbeef", got)
			}
		})
	}
}

// A run long enough to be split across several block transfers, which is where
// the posted-read pipeline and the chunking both have to be right.
func TestBlockRoundTrip(t *testing.T) {
	const words = 512

	want := make([]uint32, words)
	for i := range want {
		want[i] = uint32(0xA5A50000 + i)
	}

	for _, f := range factories() {
		t.Run(f.name, func(t *testing.T) {
			ctx := context.Background()
			apc := attach(t, f.open(t, newTarget()))

			if err := apc.WriteTargetMem(ctx, ramBase+0x100, want); err != nil {
				t.Fatalf("block write: %v", err)
			}
			got, err := apc.ReadTargetMem(ctx, ramBase+0x100, words)
			if err != nil {
				t.Fatalf("block read: %v", err)
			}
			if len(got) != words {
				t.Fatalf("read %d words, want %d", len(got), words)
			}
			for i := range want {
				if got[i] != want[i] {
					t.Fatalf("word %d = 0x%08x, want 0x%08x", i, got[i], want[i])
				}
			}
		})
	}
}

// A transfer list mixing reads and writes exercises the posted-read pipeline in
// the one place it is easy to get wrong: a DP read following an AP read has to
// drain RDBUFF first, and both values have to come back in request order.
func TestMixedTransferListReturnsValuesInOrder(t *testing.T) {
	for _, f := range factories() {
		t.Run(f.name, func(t *testing.T) {
			ctx := context.Background()
			target := newTarget()
			p := f.open(t, target)
			apc := attach(t, p)

			if err := apc.WriteTargetReg(ctx, ramBase+0x80, 0x11112222); err != nil {
				t.Fatalf("seed memory: %v", err)
			}
			// Point the MEM-AP at the word just written, so the AP read below
			// returns it.
			if err := apc.WriteReg(ctx, mem.TAR, ramBase+0x80); err != nil {
				t.Fatalf("set TAR: %v", err)
			}

			const drw = 0x0c
			const dpidr = 0x00
			_, values, err := p.Transfer(ctx, []swd.Request{
				{Op: swd.OpRead, AP: true, Reg: drw},    // posted
				{Op: swd.OpRead, AP: false, Reg: dpidr}, // forces an RDBUFF drain
			})
			if err != nil {
				t.Fatalf("transfer: %v", err)
			}
			if len(values) != 2 {
				t.Fatalf("got %d values, want 2 -- one per read", len(values))
			}
			if values[0] != 0x11112222 {
				t.Errorf("AP read = 0x%08x, want 0x11112222", values[0])
			}
			if values[1] == 0 {
				t.Errorf("DP read returned 0, want a DPIDR")
			}
		})
	}
}

// A read of a region nothing is mapped at must fail rather than return zeros,
// and it must fail the same way through both probes.
func TestFaultIsReportedNotSwallowed(t *testing.T) {
	for _, f := range factories() {
		t.Run(f.name, func(t *testing.T) {
			ctx := context.Background()
			apc := attach(t, f.open(t, newTarget()))

			if _, err := apc.ReadTargetReg(ctx, 0x90000000); err == nil {
				t.Fatal("read of unmapped memory succeeded")
			}
		})
	}
}

// swd.Cmsis is optional because only a CMSIS-DAP probe can answer it. The pins
// are not: they are on swd.Probe, and a probe that cannot reach them says so.
func TestCmsisIsOnlyClaimedByADapProbe(t *testing.T) {
	ctx := context.Background()

	bb := bitbang.New(newTarget())
	if _, ok := any(bb).(swd.Cmsis); ok {
		t.Error("bitbang probe should not claim swd.Cmsis -- there is no DAP to ask")
	}
	if _, err := bb.ReadPins(ctx); err != nil {
		t.Errorf("bitbang ReadPins: %v", err)
	}

	srv := server.New(ctx, bitbang.New(newTarget()), server.Options{})
	dapc, err := inproc.Open(ctx, srv)
	if err != nil {
		t.Fatalf("open CMSIS-DAP probe: %v", err)
	}
	dap, ok := any(dapc).(swd.Cmsis)
	if !ok {
		t.Fatal("CMSIS-DAP probe should offer swd.Cmsis")
	}

	product, err := dap.ProductID(ctx)
	if err != nil {
		t.Fatalf("ProductID: %v", err)
	}
	if product == "" {
		t.Error("ProductID is empty")
	}

	// DAP_SWJ_Pins carries the pins across the link, so the client answers the
	// same question the bitbang probe behind the server does.
	if _, err := dapc.ReadPins(ctx); err != nil {
		t.Errorf("CMSIS-DAP ReadPins: %v", err)
	}

	var _ *client.Probe = dapc
}
