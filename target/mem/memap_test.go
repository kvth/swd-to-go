package mem_test

import (
	"bytes"
	"context"
	"testing"

	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
)

const (
	ramBase = 0x20000000
	ramSize = 0x4000
)

func attach(t *testing.T, opts simtarget.Options) (*mem.Client, *simtarget.Target) {
	t.Helper()
	ctx := context.Background()

	target := simtarget.New(opts, &simtarget.Region{Base: ramBase, Data: make([]byte, ramSize)})
	probe := bitbang.New(target)

	if err := probe.Connect(ctx); err != nil {
		t.Fatalf("connect: %v", err)
	}
	dpc := dp.New(probe)
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
	return apc, target
}

// The shadow exists to keep transfers off the wire, so what it is worth is
// measured in transfers. Reading one register over and over is the case that
// matters -- it is what waiting for a core to halt looks like -- and with TAR
// already pointing at it, the TAR write is pure overhead.
//
// Note what a write costs below the probe: the write itself, and the RDBUFF
// read that confirms it landed, since an SWD write is only known to have
// worked once a following transfer comes back OK. So a TAR write skipped is
// two wire transfers saved, not one.
func TestRepeatedRegisterReadDoesNotReloadTAR(t *testing.T) {
	apc, target := attach(t, simtarget.Options{})
	ctx := context.Background()

	const at = ramBase + 0x40
	if _, err := apc.ReadTargetReg(ctx, at); err != nil { // warm the shadow
		t.Fatalf("read: %v", err)
	}

	before := target.Transfers
	for i := 0; i < 10; i++ {
		if _, err := apc.ReadTargetReg(ctx, at); err != nil {
			t.Fatalf("read %d: %v", i, err)
		}
	}
	perRead := float64(target.Transfers-before) / 10

	// An AP read is the posted read plus the RDBUFF that collects it. Anything
	// beyond that would be TAR being written again.
	if perRead != 2 {
		t.Errorf("a repeated register read costs %.1f transfers, want 2", perRead)
	}
}

func TestReadsAtDifferentAddressesDoReloadTAR(t *testing.T) {
	apc, target := attach(t, simtarget.Options{})
	ctx := context.Background()

	if _, err := apc.ReadTargetReg(ctx, ramBase); err != nil {
		t.Fatalf("read: %v", err)
	}
	before := target.Transfers
	for i := 0; i < 10; i++ {
		if _, err := apc.ReadTargetReg(ctx, ramBase+uint32(i*4)); err != nil {
			t.Fatalf("read %d: %v", i, err)
		}
	}
	// The TAR write and its confirming RDBUFF, then the read and the RDBUFF
	// that collects it. The first iteration reuses the warmed TAR, so it is
	// two transfers rather than four.
	const want = (9*4 + 2) / 10.0
	if perRead := float64(target.Transfers-before) / 10; perRead != want {
		t.Errorf("a moving register read costs %.1f transfers, want %.1f", perRead, want)
	}
}

// Anything that writes TAR behind this client's back has to say so, or the
// client will skip a TAR write it needed. target/rp2040's Flasher is exactly
// such a caller.
func TestInvalidateForcesTARToBeWrittenAgain(t *testing.T) {
	apc, target := attach(t, simtarget.Options{})
	ctx := context.Background()

	const at = ramBase + 0x40
	if _, err := apc.ReadTargetReg(ctx, at); err != nil {
		t.Fatalf("read: %v", err)
	}

	apc.Invalidate()
	before := target.Transfers
	if _, err := apc.ReadTargetReg(ctx, at); err != nil {
		t.Fatalf("read after Invalidate: %v", err)
	}
	// CSW and TAR both have to go out again -- two transfers each, with their
	// confirming RDBUFF reads -- on top of the read and the RDBUFF that
	// collects it.
	if n := target.Transfers - before; n != 6 {
		t.Errorf("a read after Invalidate cost %d transfers, want 6 with CSW and TAR rewritten", n)
	}
}

// --- unaligned access -------------------------------------------------------

// The two AP flavours, since the whole point of probing for byte access is
// that some APs do not have it and have to be driven differently.
func forEachAP(t *testing.T, f func(t *testing.T, opts simtarget.Options)) {
	t.Helper()
	t.Run("byte-capable", func(t *testing.T) { f(t, simtarget.Options{}) })
	t.Run("word-only", func(t *testing.T) { f(t, simtarget.Options{NoByteAccess: true}) })
}

func TestUnalignedRoundTrip(t *testing.T) {
	forEachAP(t, func(t *testing.T, opts simtarget.Options) {
		apc, _ := attach(t, opts)
		ctx := context.Background()

		// Every combination of a ragged start and a ragged length, including
		// the ones that fit entirely inside one word.
		for _, off := range []uint32{0, 1, 2, 3} {
			for _, n := range []int{1, 2, 3, 4, 5, 7, 8, 9, 13} {
				at := uint32(ramBase+0x100) + off
				want := make([]byte, n)
				for i := range want {
					want[i] = byte(int(off)*29 + i*7 + 1)
				}

				if err := apc.WriteTargetMemBytes(ctx, at, want); err != nil {
					t.Fatalf("write %d bytes at +%d: %v", n, off, err)
				}
				got, err := apc.ReadTargetMemBytes(ctx, at, n)
				if err != nil {
					t.Fatalf("read %d bytes at +%d: %v", n, off, err)
				}
				if !bytes.Equal(got, want) {
					t.Fatalf("at +%d, %d bytes: read %x, wrote %x", off, n, got, want)
				}
			}
		}
	})
}

// A byte-capable AP writes exactly the bytes it was asked to. The word-only
// fallback cannot: it widens the range to whole words and puts the surrounding
// bytes back as it found them, which is only correct so long as nothing else
// changed them. Both have to leave the neighbours reading the same.
func TestUnalignedWriteLeavesItsNeighboursAlone(t *testing.T) {
	forEachAP(t, func(t *testing.T, opts simtarget.Options) {
		apc, target := attach(t, opts)
		ctx := context.Background()

		const at = ramBase + 0x200
		filler := make([]byte, 64)
		for i := range filler {
			filler[i] = byte(0xA0 + i)
		}
		if !target.WriteMem(at, filler) {
			t.Fatal("could not seed the region")
		}

		if err := apc.WriteTargetMemBytes(ctx, at+9, []byte{0x11, 0x22, 0x33}); err != nil {
			t.Fatalf("write: %v", err)
		}

		want := append([]byte(nil), filler...)
		copy(want[9:], []byte{0x11, 0x22, 0x33})
		if got := target.ReadMem(at, len(want)); !bytes.Equal(got, want) {
			t.Errorf("the region reads %x, want %x", got, want)
		}
	})
}

// TAR only counts within a 1 kB window and wraps rather than carrying, so a
// block transfer crossing one has to stop and reload it. The shadow tracks
// where the wrap left TAR, and getting that wrong puts the next run a
// kilobyte away from where it belongs.
func TestBlockTransferAcrossTheAutoIncrementBoundary(t *testing.T) {
	apc, _ := attach(t, simtarget.Options{})
	ctx := context.Background()

	// Start a few words short of a 1 kB boundary and run well past it.
	at := uint32(ramBase + 1024 - 16)
	want := make([]byte, 3*1024)
	for i := range want {
		want[i] = byte(i*31 + 3)
	}

	if err := apc.WriteTargetMemBytes(ctx, at, want); err != nil {
		t.Fatalf("write: %v", err)
	}
	got, err := apc.ReadTargetMemBytes(ctx, at, len(want))
	if err != nil {
		t.Fatalf("read: %v", err)
	}
	if !bytes.Equal(got, want) {
		for i := range want {
			if got[i] != want[i] {
				t.Fatalf("first difference at +%d (0x%08x): read 0x%02x, wrote 0x%02x",
					i, at+uint32(i), got[i], want[i])
			}
		}
	}
}

// A run of single-word accesses and a run of block accesses want different
// auto-increment settings, so switching between them costs a CSW write. What
// must not happen is a CSW write inside either run.
func TestBlockReadsDoNotRewriteCSW(t *testing.T) {
	apc, target := attach(t, simtarget.Options{})
	ctx := context.Background()

	if _, err := apc.ReadTargetMem(ctx, ramBase, 4); err != nil { // warm the shadow
		t.Fatalf("read: %v", err)
	}
	before := target.Transfers
	for i := 0; i < 8; i++ {
		if _, err := apc.ReadTargetMem(ctx, ramBase, 4); err != nil {
			t.Fatalf("read %d: %v", i, err)
		}
	}
	// Per read: the TAR write and its confirming RDBUFF, then four DRW reads
	// pipelined into each other and the RDBUFF that drains the last. A ninth
	// would be CSW going out again inside the run.
	if n := target.Transfers - before; n != 8*7 {
		t.Errorf("eight four-word reads cost %d transfers, want %d", n, 8*7)
	}
}

// An unaligned range used to be moved a byte at a time -- a TAR write and a
// DRW access each -- so its ends cost several transfers per byte where the
// words containing them cost a handful in total. RTT rings sit at arbitrary
// offsets, so this is the common case and not the corner one.
//
// The numbers are deliberately compared against the aligned read of the same
// length rather than written down: what matters is that a ragged end costs
// about the same as a tidy one, not what either costs this month.
func TestUnalignedAccessCostsAboutWhatAnAlignedOneDoes(t *testing.T) {
	forEachAP(t, func(t *testing.T, opts simtarget.Options) {
		apc, target := attach(t, opts)
		ctx := context.Background()

		const at = ramBase + 0x300
		cost := func(f func() error) int {
			before := target.Transfers
			if err := f(); err != nil {
				t.Fatalf("access: %v", err)
			}
			return target.Transfers - before
		}

		data := make([]byte, 64)
		aligned := cost(func() error {
			_, err := apc.ReadTargetMemBytes(ctx, at, len(data))
			return err
		})
		ragged := cost(func() error {
			_, err := apc.ReadTargetMemBytes(ctx, at+2, len(data)-3)
			return err
		})
		if ragged > aligned+4 {
			t.Errorf("a ragged read costs %d transfers against an aligned one's %d; "+
				"the ends are being moved a byte at a time again", ragged, aligned)
		}

		alignedW := cost(func() error { return apc.WriteTargetMemBytes(ctx, at, data) })
		raggedW := cost(func() error { return apc.WriteTargetMemBytes(ctx, at+2, data[:len(data)-3]) })
		// A ragged write also reads the two boundary words back before it can
		// put them, which an aligned write does not.
		if raggedW > alignedW*2+4 {
			t.Errorf("a ragged write costs %d transfers against an aligned one's %d; "+
				"the ends are being moved a byte at a time again", raggedW, alignedW)
		}
	})
}
