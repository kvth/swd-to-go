package rp2040_test

import (
	"bytes"
	"context"
	"testing"

	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/rp2040"
)

func openSession(t *testing.T) (*rp2040.Session, *simtarget.RP2040, *simtarget.Target) {
	t.Helper()
	target, chip := simtarget.NewRP2040(simtarget.Options{TargetSel: rp2040.TARGET_CORE_0}, flashSize)
	s, err := rp2040.Open(context.Background(), bitbang.New(target), rp2040.TARGET_CORE_0)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	t.Cleanup(func() { s.Close(context.Background()) })
	return s, chip, target
}

func TestSessionProgramsAndReadsBack(t *testing.T) {
	s, chip, _ := openSession(t)
	ctx := context.Background()
	image := testImage(5 * 1024)

	// program <file> 0x10001000 preverify verify reset
	_, err := s.Program(ctx, image, rp2040.Options{
		Offset: 0x1000, PreVerify: true, Verify: true, Reset: true, Run: true,
	})
	if err != nil {
		t.Fatalf("Program: %v", err)
	}
	if chip.Halted() {
		t.Error("the core is halted after a program with reset and run")
	}

	got, err := s.ReadMemory(ctx, rp2040.FlashBase+0x1000, len(image))
	if err != nil {
		t.Fatalf("ReadMemory: %v", err)
	}
	if !bytes.Equal(got, image) {
		t.Error("what ReadMemory returns is not the image")
	}
}

// dump_image does not stop the target, and neither does a read of flash that
// is already mapped.
func TestSessionReadMemoryLeavesAMappedChipRunning(t *testing.T) {
	s, chip, _ := openSession(t)
	copy(chip.Flash, []byte{1, 2, 3, 4, 5, 6, 7})

	got, err := s.ReadMemory(context.Background(), rp2040.FlashBase+1, 5)
	if err != nil {
		t.Fatalf("ReadMemory: %v", err)
	}
	if !bytes.Equal(got, []byte{2, 3, 4, 5, 6}) {
		t.Errorf("read % x, want 02 03 04 05 06", got)
	}
	if chip.Halted() || chip.Calls != 0 {
		t.Errorf("halted %v after %d ROM calls; a read of mapped flash wants neither",
			chip.Halted(), chip.Calls)
	}
}

func TestSessionReadMemoryMapsFlashFirst(t *testing.T) {
	s, chip, _ := openSession(t)
	copy(chip.Flash, []byte{0xde, 0xad, 0xbe, 0xef})
	chip.UnmapXIP()

	got, err := s.ReadMemory(context.Background(), rp2040.FlashBase, 4)
	if err != nil {
		t.Fatalf("ReadMemory with flash unmapped: %v", err)
	}
	if !bytes.Equal(got, []byte{0xde, 0xad, 0xbe, 0xef}) {
		t.Errorf("read % x, want de ad be ef", got)
	}
}

// Mapping flash runs ROM calls on a stack in SRAM, so a read that does not
// reach flash must not do it: SRAM after a crash is worth reading as it was.
func TestSessionReadMemoryOfSRAMLeavesTheSSIAlone(t *testing.T) {
	s, chip, target := openSession(t)
	target.WriteMem(simtarget.SRAMBase, []byte{9, 8, 7, 6})
	chip.UnmapXIP()

	got, err := s.ReadMemory(context.Background(), simtarget.SRAMBase, 4)
	if err != nil {
		t.Fatalf("ReadMemory: %v", err)
	}
	if !bytes.Equal(got, []byte{9, 8, 7, 6}) {
		t.Errorf("read % x, want 09 08 07 06", got)
	}
	if chip.Calls != 0 || chip.XIPMapped {
		t.Errorf("%d ROM calls ran and mapped is %v for a read of SRAM", chip.Calls, chip.XIPMapped)
	}
}

func TestSessionResetHalts(t *testing.T) {
	s, chip, _ := openSession(t)
	if err := s.Reset(context.Background(), true); err != nil {
		t.Fatalf("reset halt: %v", err)
	}
	if !chip.Halted() {
		t.Error("the core is running after reset halt")
	}
	if err := s.Reset(context.Background(), false); err != nil {
		t.Fatalf("reset run: %v", err)
	}
	if chip.Halted() {
		t.Error("the core is halted after reset run")
	}
}
