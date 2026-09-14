package rp2040_test

import (
	"bytes"
	"context"
	"strings"
	"testing"

	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
	"github.com/kvth/swd-to-go/target/rp2040"
)

// These run against internal/simtarget's RP2040: a bootrom with the real
// function table layout, SRAM, an XIP-mapped flash array and the ARMv6-M debug
// registers. There is no CPU in it, so "resume" means "carry out the routine
// named in r7 and halt at the trampoline's breakpoint" -- which is exactly the
// contract the flashing sequence depends on, and the only one worth simulating.
//
// Flash programming in the simulation can only clear bits, the way the real
// part does. That is what makes a missing erase show up here as corruption
// rather than as a clean write, and it is what the EraseNone test below turns
// into an assertion.

const flashSize = 512 * 1024

func newChip(t *testing.T, opts simtarget.Options) (*rp2040.Flasher, *simtarget.RP2040, *simtarget.Target) {
	t.Helper()

	if opts.TargetSel == 0 {
		opts.TargetSel = rp2040.TARGET_CORE_0
	}
	target, chip := simtarget.NewRP2040(opts, flashSize)

	ctx := context.Background()
	probe := bitbang.New(target)
	if _, err := rp2040.Attach(ctx, probe, rp2040.TARGET_CORE_0); err != nil {
		t.Fatalf("attach: %v", err)
	}
	dpc := dp.New(probe)
	if err := dpc.Init(ctx); err != nil {
		t.Fatalf("init DP: %v", err)
	}
	apc := mem.New(dpc, 0)
	if err := apc.Init(ctx); err != nil {
		t.Fatalf("init MEM-AP: %v", err)
	}
	return rp2040.NewFlasher(probe, dpc, apc, 0, 0, 0, 0), chip, target
}

// testImage is deterministic and page-aligned in nothing in particular, so the
// padding path gets exercised too.
func testImage(n int) []byte {
	img := make([]byte, n)
	for i := range img {
		img[i] = byte(i*13 + 5)
	}
	return img
}

func TestFlashImageWritesAndVerifies(t *testing.T) {
	f, chip, _ := newChip(t, simtarget.Options{})
	image := testImage(9 * 1024)

	res, err := f.FlashImage(context.Background(), image, rp2040.Options{Verify: true})
	if err != nil {
		t.Fatalf("FlashImage: %v", err)
	}

	if !bytes.Equal(chip.Flash[:len(image)], image) {
		t.Error("what is in flash is not the image")
	}
	if !chip.XIPMapped {
		t.Error("flash was left out of the memory map, so the target would not boot")
	}
	if !res.Verified {
		t.Error("Verified is not set on a run that verified")
	}
	// 9 kB spans three 4 kB sectors, and the image is padded up to a page.
	if res.Sectors != 3 || res.Programmed != 3 || res.Skipped != 0 {
		t.Errorf("planned %d sectors, programmed %d, skipped %d; want 3, 3, 0",
			res.Sectors, res.Programmed, res.Skipped)
	}
	if res.Total != 9*1024 {
		t.Errorf("Total is %d, want the image rounded up to a page", res.Total)
	}
}

func TestFlashImagePadsTheLastPage(t *testing.T) {
	f, chip, _ := newChip(t, simtarget.Options{})
	image := testImage(300) // one page and a bit

	res, err := f.FlashImage(context.Background(), image, rp2040.Options{})
	if err != nil {
		t.Fatalf("FlashImage: %v", err)
	}
	if res.Total != 512 {
		t.Errorf("Total is %d, want 512", res.Total)
	}
	// Erased flash reads as 0xFF, so the padding has to be indistinguishable
	// from never having been written.
	for i := len(image); i < 512; i++ {
		if chip.Flash[i] != 0xFF {
			t.Fatalf("the tail of the last page is 0x%02x at %d, want 0xFF", chip.Flash[i], i)
			return
		}
	}
}

func TestPreVerifySkipsAnAlreadyCurrentTarget(t *testing.T) {
	f, chip, _ := newChip(t, simtarget.Options{})
	image := testImage(8 * 1024)
	ctx := context.Background()

	if _, err := f.FlashImage(ctx, image, rp2040.Options{}); err != nil {
		t.Fatalf("first write: %v", err)
	}

	before := chip.Calls
	res, err := f.FlashImage(ctx, image, rp2040.Options{PreVerify: true})
	if err != nil {
		t.Fatalf("second write: %v", err)
	}

	if !res.UpToDate {
		t.Error("UpToDate is not set on a target that was already current")
	}
	if !res.Verified {
		t.Error("a preverify that matched should count as verified")
	}
	if res.Skipped != res.Sectors || res.Programmed != 0 || res.Erased != 0 {
		t.Errorf("skipped %d of %d sectors, programmed %d, erased %d; want everything skipped",
			res.Skipped, res.Sectors, res.Programmed, res.Erased)
	}
	// The point of preverify is that nothing is written, and a ROM call is how
	// anything gets written -- so not one of them should have run.
	if chip.Calls != before {
		t.Errorf("%d bootrom calls ran on a target that needed no writing", chip.Calls-before)
	}
}

func TestEraseAutoOnlyTouchesTheSectorsThatChanged(t *testing.T) {
	f, chip, _ := newChip(t, simtarget.Options{})
	image := testImage(16 * 1024) // four sectors
	ctx := context.Background()

	if _, err := f.FlashImage(ctx, image, rp2040.Options{}); err != nil {
		t.Fatalf("first write: %v", err)
	}

	// Change one byte, in the third sector.
	next := append([]byte(nil), image...)
	next[9*1024] ^= 0xFF

	res, err := f.FlashImage(ctx, next, rp2040.Options{Erase: rp2040.EraseAuto, Verify: true})
	if err != nil {
		t.Fatalf("second write: %v", err)
	}

	if res.Sectors != 4 {
		t.Fatalf("the image covers %d sectors, want 4", res.Sectors)
	}
	if res.Programmed != 1 || res.Erased != 1 || res.Skipped != 3 {
		t.Errorf("programmed %d, erased %d, skipped %d; want 1, 1, 3 for a one-byte change",
			res.Programmed, res.Erased, res.Skipped)
	}
	if !bytes.Equal(chip.Flash[:len(next)], next) {
		t.Error("what is in flash is not the second image")
	}
}

func TestEraseAutoSkipsTheEraseOfABlankSector(t *testing.T) {
	f, chip, _ := newChip(t, simtarget.Options{})
	image := testImage(8 * 1024)

	// Nothing has been written, so every sector is blank and none of them
	// needs clearing before it is programmed.
	res, err := f.FlashImage(context.Background(), image, rp2040.Options{Erase: rp2040.EraseAuto})
	if err != nil {
		t.Fatalf("FlashImage: %v", err)
	}
	if res.Erased != 0 {
		t.Errorf("erased %d sectors on a blank part, want 0", res.Erased)
	}
	if res.Programmed != res.Sectors {
		t.Errorf("programmed %d of %d sectors, want all of them", res.Programmed, res.Sectors)
	}
	if !bytes.Equal(chip.Flash[:len(image)], image) {
		t.Error("what is in flash is not the image")
	}
}

func TestEraseAllRewritesEverything(t *testing.T) {
	f, _, _ := newChip(t, simtarget.Options{})
	image := testImage(16 * 1024)
	ctx := context.Background()

	if _, err := f.FlashImage(ctx, image, rp2040.Options{}); err != nil {
		t.Fatalf("first write: %v", err)
	}
	res, err := f.FlashImage(ctx, image, rp2040.Options{Erase: rp2040.EraseAll, Verify: true})
	if err != nil {
		t.Fatalf("second write: %v", err)
	}
	if res.Skipped != 0 || res.Erased != res.Sectors || res.Programmed != res.Sectors {
		t.Errorf("erased %d, programmed %d, skipped %d of %d sectors; want everything rewritten",
			res.Erased, res.Programmed, res.Skipped, res.Sectors)
	}
}

func TestVerifyCatchesAWriteIntoAnUnerasedSector(t *testing.T) {
	f, _, _ := newChip(t, simtarget.Options{})
	ctx := context.Background()

	if _, err := f.FlashImage(ctx, testImage(4*1024), rp2040.Options{}); err != nil {
		t.Fatalf("first write: %v", err)
	}

	// Programming can only clear bits, so writing different data over a sector
	// that was never erased leaves the AND of the two -- which is what Verify
	// is for.
	other := bytes.Repeat([]byte{0x5A}, 4*1024)
	_, err := f.FlashImage(ctx, other, rp2040.Options{Erase: rp2040.EraseNone, Verify: true})
	if err == nil {
		t.Fatal("a write into an unerased sector verified clean")
	}
}

func TestFlashImageResetsAndRuns(t *testing.T) {
	f, chip, _ := newChip(t, simtarget.Options{})

	_, err := f.FlashImage(context.Background(), testImage(2048),
		rp2040.Options{Reset: true, Run: true})
	if err != nil {
		t.Fatalf("FlashImage: %v", err)
	}
	if chip.Halted() {
		t.Error("the core is still halted after a run was asked for")
	}
}

func TestFlashImageResetsAndHalts(t *testing.T) {
	f, chip, _ := newChip(t, simtarget.Options{})

	_, err := f.FlashImage(context.Background(), testImage(2048),
		rp2040.Options{Reset: true})
	if err != nil {
		t.Fatalf("FlashImage: %v", err)
	}
	if !chip.Halted() {
		t.Error("the core is running after a reset that should have caught it")
	}
}

func TestFlashImageRefusesAnOffsetAnEraseWouldOverrun(t *testing.T) {
	f, _, _ := newChip(t, simtarget.Options{})

	// Page-aligned but not sector-aligned: erasing it would clear the start of
	// the sector as well, which the caller did not ask for.
	_, err := f.FlashImage(context.Background(), testImage(1024),
		rp2040.Options{Offset: rp2040.FlashPage})
	if err == nil {
		t.Fatal("an offset partway into a sector was accepted with erasing on")
	}
}

func TestFlashImageAtAnOffset(t *testing.T) {
	f, chip, _ := newChip(t, simtarget.Options{})
	image := testImage(4 * 1024)
	const at = 64 * 1024

	res, err := f.FlashImage(context.Background(), image,
		rp2040.Options{Offset: at, Verify: true})
	if err != nil {
		t.Fatalf("FlashImage: %v", err)
	}
	if !res.Verified {
		t.Error("the run did not verify")
	}
	if !bytes.Equal(chip.Flash[at:at+len(image)], image) {
		t.Error("the image is not at the offset it was written to")
	}
	// Nothing outside the range it was given.
	if !allBlank(chip.Flash[:at]) {
		t.Error("flash below the offset was disturbed")
	}
}

func TestFlashImageOfNothing(t *testing.T) {
	f, _, _ := newChip(t, simtarget.Options{})

	res, err := f.FlashImage(context.Background(), nil, rp2040.Options{Verify: true})
	if err != nil {
		t.Fatalf("FlashImage: %v", err)
	}
	if res.Total != 0 || !res.UpToDate {
		t.Errorf("an empty image gave %+v", res)
	}
}

func TestFlashImageReportsProgress(t *testing.T) {
	f, _, _ := newChip(t, simtarget.Options{})

	seen := map[string]bool{}
	_, err := f.FlashImage(context.Background(), testImage(8*1024), rp2040.Options{
		PreVerify: true,
		Verify:    true,
		Progress:  func(phase string, done, total int) { seen[phase] = true },
	})
	if err != nil {
		t.Fatalf("FlashImage: %v", err)
	}
	for _, phase := range []string{"preverify", "program", "verify"} {
		if !seen[phase] {
			t.Errorf("no progress was reported for the %q phase", phase)
		}
	}
}

// --- rescue -----------------------------------------------------------------

func TestRescueRunsTheResetSequence(t *testing.T) {
	target, _ := simtarget.NewRP2040(simtarget.Options{
		Multidrop: true,
		TargetSel: rp2040.TARGET_RESCUE,
	}, flashSize)

	if err := rp2040.Rescue(context.Background(), bitbang.New(target)); err != nil {
		t.Fatalf("Rescue: %v", err)
	}
	// The power-up request and its acknowledge are both gone, which is what
	// says the reset was released rather than left on.
	if got := target.CtrlStat(); got != 0 {
		t.Errorf("CTRL/STAT is 0x%08x after a rescue, want every power bit clear", got)
	}
}

// The rescue port is instance 0xF, not either core's. A wire where only core 0
// answers has to leave a rescue with nothing to talk to -- which is what says
// the TARGETSEL being written is the rescue port's and not the core's.
func TestRescueAddressesTheRescuePort(t *testing.T) {
	target, _ := simtarget.NewRP2040(simtarget.Options{
		Multidrop: true,
		TargetSel: rp2040.TARGET_CORE_0,
	}, flashSize)

	if err := rp2040.Rescue(context.Background(), bitbang.New(target)); err == nil {
		t.Fatal("a rescue succeeded on a wire where only core 0 answers")
	}
}

func TestRescueFailsWhenTheResetIsNotReleased(t *testing.T) {
	target, _ := simtarget.NewRP2040(simtarget.Options{
		Multidrop:    true,
		TargetSel:    rp2040.TARGET_RESCUE,
		PowerLatched: true,
	}, flashSize)

	err := rp2040.Rescue(context.Background(), bitbang.New(target))
	if err == nil || !strings.Contains(err.Error(), "did not release the reset") {
		t.Fatalf("Rescue gave %v, want the reset-not-released failure", err)
	}
}

func TestRescueAndAttachLeavesAUsableTarget(t *testing.T) {
	// One simulated DP answering for both the rescue port and core 0, since
	// the point here is that the two bring-ups run back to back on one wire.
	ctx := context.Background()
	target, chip := simtarget.NewRP2040(simtarget.Options{}, flashSize)

	probe := bitbang.New(target)
	if _, err := rp2040.RescueAndAttach(ctx, probe, rp2040.TARGET_CORE_0); err != nil {
		t.Fatalf("RescueAndAttach: %v", err)
	}
	dpc := dp.New(probe)
	if err := dpc.Init(ctx); err != nil {
		t.Fatalf("init DP: %v", err)
	}
	apc := mem.New(dpc, 0)
	if err := apc.Init(ctx); err != nil {
		t.Fatalf("init MEM-AP: %v", err)
	}

	image := testImage(2048)
	f := rp2040.NewFlasher(probe, dpc, apc, 0, 0, 0, 0)
	if _, err := f.FlashImage(ctx, image, rp2040.Options{Verify: true}); err != nil {
		t.Fatalf("FlashImage after a rescue: %v", err)
	}
	if !bytes.Equal(chip.Flash[:len(image)], image) {
		t.Error("what is in flash is not the image")
	}
}

// --- helpers ----------------------------------------------------------------

func allBlank(b []byte) bool {
	for _, v := range b {
		if v != 0xFF {
			return false
		}
	}
	return true
}
