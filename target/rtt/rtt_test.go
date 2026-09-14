package rtt_test

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"strings"
	"testing"

	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
	"github.com/kvth/swd-to-go/target/rtt"
)

const (
	ramBase = 0x20000000
	ramSize = 0x4000
)

var layout = simtarget.RTTLayout{
	CBAddr:   ramBase + 0x100,
	NameAddr: ramBase + 0x080,
	UpAddr:   ramBase + 0x400,
	DownAddr: ramBase + 0x800,
	BufSize:  64,
}

func attach(t *testing.T, regions ...*simtarget.Region) (*mem.Client, *simtarget.Target) {
	t.Helper()
	ctx := context.Background()

	if len(regions) == 0 {
		regions = []*simtarget.Region{{Base: ramBase, Data: make([]byte, ramSize)}}
	}
	target := simtarget.New(simtarget.Options{}, regions...)
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

func withBlock(t *testing.T) (*rtt.RTT, *simtarget.RTTBlock, *mem.Client, *simtarget.Target) {
	t.Helper()
	apc, target := attach(t)
	block := simtarget.NewRTTBlock(target, layout, "Terminal")

	r, err := rtt.New(context.Background(), apc, layout.CBAddr)
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	return r, block, apc, target
}

// --- finding the block ------------------------------------------------------

func TestFindLocatesTheBlock(t *testing.T) {
	apc, target := attach(t)
	simtarget.NewRTTBlock(target, layout, "Terminal")

	got, err := rtt.FindRTTControlBlock(context.Background(), apc, ramBase, ramSize, 0x200)
	if err != nil {
		t.Fatalf("FindRTTControlBlock: %v", err)
	}
	if got != layout.CBAddr {
		t.Errorf("found 0x%08x, want 0x%08x", got, layout.CBAddr)
	}
}

// The scan reads a chunk at a time, and a block whose signature lands across a
// chunk boundary is the case that gets missed: without an overlap between
// chunks the scan has a blind spot every blockSize bytes.
func TestFindLocatesABlockStraddlingAChunkBoundary(t *testing.T) {
	const chunk = 0x200

	// Put the ID so that it starts a few bytes before a chunk boundary and
	// finishes after it.
	at := uint32(ramBase) + chunk - 4
	straddling := layout
	straddling.CBAddr = at

	apc, target := attach(t)
	simtarget.NewRTTBlock(target, straddling, "Terminal")

	got, err := rtt.FindRTTControlBlock(context.Background(), apc, ramBase, ramSize, chunk)
	if err != nil {
		t.Fatalf("a control block across a chunk boundary was not found: %v", err)
	}
	if got != at {
		t.Errorf("found 0x%08x, want 0x%08x", got, at)
	}
}

// The useful thing to hand this is a whole RAM window, which means ranges that
// are partly unmapped. A fault has to be stepped over, not treated as the end
// of the search -- and a sticky error left behind would fail every transfer
// after it, so it has to be cleared too.
func TestFindStepsOverUnmappedMemory(t *testing.T) {
	// Two regions with a hole between them, and the block in the far one.
	const holeAt = ramBase + 0x1000
	low := &simtarget.Region{Base: ramBase, Data: make([]byte, 0x1000)}
	high := &simtarget.Region{Base: holeAt + 0x1000, Data: make([]byte, 0x1000)}

	apc, target := attach(t, low, high)

	far := layout
	far.CBAddr = high.Base + 0x100
	far.NameAddr = high.Base + 0x080
	far.UpAddr = high.Base + 0x400
	far.DownAddr = high.Base + 0x800
	simtarget.NewRTTBlock(target, far, "Terminal")

	got, err := rtt.FindRTTControlBlock(context.Background(), apc, ramBase, 0x3000, 0x200)
	if err != nil {
		t.Fatalf("the search gave up at the unmapped hole: %v", err)
	}
	if got != far.CBAddr {
		t.Errorf("found 0x%08x, want 0x%08x", got, far.CBAddr)
	}
	if target.Sticky() {
		t.Error("the search left a sticky DP error behind")
	}
}

// The ID string is in the target's own binary too, wherever SEGGER's code put
// it. Matching it is necessary but not sufficient: the header has to hold up,
// and the scan has to carry on past one that does not.
func TestFindSkipsTheIDStringOnItsOwn(t *testing.T) {
	apc, target := attach(t)

	// A decoy: the signature, followed by buffer counts nothing would declare.
	decoy := make([]byte, 24)
	copy(decoy, "SEGGER RTT\x00")
	binary.LittleEndian.PutUint32(decoy[16:], 9999)
	binary.LittleEndian.PutUint32(decoy[20:], 9999)
	if !target.WriteMem(ramBase+0x20, decoy) {
		t.Fatal("could not plant the decoy")
	}

	simtarget.NewRTTBlock(target, layout, "Terminal")

	got, err := rtt.FindRTTControlBlock(context.Background(), apc, ramBase, ramSize, 0x200)
	if err != nil {
		t.Fatalf("FindRTTControlBlock: %v", err)
	}
	if got != layout.CBAddr {
		t.Errorf("found 0x%08x -- the decoy at 0x%08x was taken for a control block",
			got, ramBase+0x20)
	}
}

func TestNewRejectsWhatIsNotAControlBlock(t *testing.T) {
	apc, _ := attach(t)

	_, err := rtt.New(context.Background(), apc, ramBase+0x100)
	if !errors.Is(err, rtt.ErrNotFound) {
		t.Fatalf("New on empty RAM gave %v, want ErrNotFound", err)
	}
}

// --- channels ---------------------------------------------------------------

func TestChannelReportsTheDescriptor(t *testing.T) {
	r, _, _, _ := withBlock(t)

	info, err := r.Channel(context.Background(), rtt.Up, 0)
	if err != nil {
		t.Fatalf("Channel: %v", err)
	}
	if info.Name != "Terminal" {
		t.Errorf("channel name is %q, want %q", info.Name, "Terminal")
	}
	if info.Buffer != layout.UpAddr || info.Size != layout.BufSize {
		t.Errorf("buffer 0x%08x size %d, want 0x%08x size %d",
			info.Buffer, info.Size, layout.UpAddr, layout.BufSize)
	}
	if up, down := r.Channels(); up != 1 || down != 1 {
		t.Errorf("the block declares %d up and %d down, want 1 and 1", up, down)
	}
}

func TestChannelNumberOutOfRange(t *testing.T) {
	r, _, _, _ := withBlock(t)

	if _, err := r.Channel(context.Background(), rtt.Up, 7); !isNoSuchChannel(err) {
		t.Errorf("channel 7 gave %v, want no-such-channel", err)
	}
	if _, err := r.Read(context.Background(), 7); !isNoSuchChannel(err) {
		t.Errorf("reading channel 7 gave %v, want no-such-channel", err)
	}
}

// The out-of-range and corrupt-descriptor failures are not exported: a caller
// has nothing to do with either but report it. Match on the message, which is
// the contract that is left.
func isNoSuchChannel(err error) bool {
	return err != nil && strings.Contains(err.Error(), "no such channel")
}

// SEGGER's default control block reserves slots the target may never give
// storage to. Reaching one has to say so, not divide by its zero size.
func TestChannelWithNoBuffer(t *testing.T) {
	r, block, _, target := withBlock(t)

	// Take the up channel's buffer away, and put something in the ring
	// pointers so the "nothing pending" shortcut does not hide it.
	zero := make([]byte, 4)
	if !target.WriteMem(block.UpDesc()+8, zero) { // SizeOfBuffer
		t.Fatal("could not clear the buffer size")
	}
	var wr [4]byte
	binary.LittleEndian.PutUint32(wr[:], 8)
	if !target.WriteMem(block.UpDesc()+12, wr[:]) { // WrOff
		t.Fatal("could not set the write offset")
	}

	_, err := r.Read(context.Background(), 0)
	if !errors.Is(err, rtt.ErrNoBuffer) {
		t.Fatalf("reading an unbacked channel gave %v, want ErrNoBuffer", err)
	}
}

func TestChannelWithOffsetsOutsideItsBuffer(t *testing.T) {
	r, block, _, target := withBlock(t)

	var wr [4]byte
	binary.LittleEndian.PutUint32(wr[:], layout.BufSize+1)
	if !target.WriteMem(block.UpDesc()+12, wr[:]) { // WrOff past the end
		t.Fatal("could not set the write offset")
	}

	_, err := r.Read(context.Background(), 0)
	if err == nil || !strings.Contains(err.Error(), "descriptor is corrupt") {
		t.Fatalf("reading a corrupt descriptor gave %v, want the corrupt-descriptor failure", err)
	}
}

// --- the ring ---------------------------------------------------------------

func TestReadReturnsNothingWhenTheRingIsEmpty(t *testing.T) {
	r, _, _, _ := withBlock(t)

	got, err := r.Read(context.Background(), 0)
	if err != nil {
		t.Fatalf("Read: %v", err)
	}
	if len(got) != 0 {
		t.Errorf("read %q from an empty ring", got)
	}
}

func TestWriteThenTargetReads(t *testing.T) {
	r, block, _, _ := withBlock(t)

	want := []byte("hello")
	n, err := r.Write(context.Background(), 0, want)
	if err != nil {
		t.Fatalf("Write: %v", err)
	}
	if n != len(want) {
		t.Fatalf("wrote %d of %d bytes", n, len(want))
	}
	if got := block.DrainDown(); !bytes.Equal(got, want) {
		t.Errorf("the target read %q, want %q", got, want)
	}
}

// A ring that has wrapped needs two reads, from the read offset to the end and
// then from the start. Draining only the first run per call leaves the caller
// to notice and come back, which is a round trip it should not need.
func TestReadDrainsAWrappedRingInOneCall(t *testing.T) {
	r, block, _, _ := withBlock(t)
	ctx := context.Background()

	// Fill most of the ring and take it out again, to push both offsets near
	// the end without wrapping yet.
	first := bytes.Repeat([]byte{'a'}, int(layout.BufSize)-8)
	block.FillUp(first)
	if got, err := r.Read(ctx, 0); err != nil {
		t.Fatalf("Read: %v", err)
	} else if len(got) != len(first) {
		t.Fatalf("first read got %d bytes, want %d", len(got), len(first))
	}

	// Now a run that crosses the end of the ring.
	want := []byte("0123456789ABCDEF")
	block.FillUp(want)

	got, err := r.Read(ctx, 0)
	if err != nil {
		t.Fatalf("Read: %v", err)
	}
	if !bytes.Equal(got, want) {
		t.Errorf("read %q in one call, want %q -- the wrapped run was not drained", got, want)
	}
}

// The same on the way out: a write that crosses the end of the ring goes in two
// runs, and the second starts at the buffer's base.
func TestWriteWrapsAroundTheRing(t *testing.T) {
	r, block, _, _ := withBlock(t)
	ctx := context.Background()

	// Push the down ring's offsets near the end.
	filler := bytes.Repeat([]byte{'x'}, int(layout.BufSize)-8)
	if _, err := r.Write(ctx, 0, filler); err != nil {
		t.Fatalf("filling: %v", err)
	}
	if got := block.DrainDown(); len(got) != len(filler) {
		t.Fatalf("the target read %d bytes, want %d", len(got), len(filler))
	}

	want := []byte("0123456789ABCDEF")
	n, err := r.Write(ctx, 0, want)
	if err != nil {
		t.Fatalf("Write: %v", err)
	}
	if n != len(want) {
		t.Fatalf("wrote %d of %d bytes across the wrap", n, len(want))
	}
	if got := block.DrainDown(); !bytes.Equal(got, want) {
		t.Errorf("the target read %q, want %q", got, want)
	}
}

// The ring keeps one slot empty so that full and empty are distinguishable, so
// a write of more than fits takes what it can and says how much.
func TestWriteTakesWhatFits(t *testing.T) {
	r, _, _, _ := withBlock(t)

	tooMuch := bytes.Repeat([]byte{'y'}, int(layout.BufSize)*2)
	n, err := r.Write(context.Background(), 0, tooMuch)
	if err != nil {
		t.Fatalf("Write: %v", err)
	}
	if want := int(layout.BufSize) - 1; n != want {
		t.Errorf("wrote %d bytes into a %d-byte ring, want %d", n, layout.BufSize, want)
	}
}

// --- what a poll costs ------------------------------------------------------

// The buffer counts are fixed when the target initialises the block, so reading
// them again on every poll is a round trip spent confirming something already
// known. Two polls should cost exactly twice one.
func TestPollingDoesNotRereadTheHeader(t *testing.T) {
	r, _, _, target := withBlock(t)
	ctx := context.Background()

	if _, err := r.Read(ctx, 0); err != nil { // warm the MEM-AP shadow
		t.Fatalf("Read: %v", err)
	}

	before := target.Transfers
	if _, err := r.Read(ctx, 0); err != nil {
		t.Fatalf("Read: %v", err)
	}
	one := target.Transfers - before

	before = target.Transfers
	for i := 0; i < 4; i++ {
		if _, err := r.Read(ctx, 0); err != nil {
			t.Fatalf("Read: %v", err)
		}
	}
	if four := target.Transfers - before; four != 4*one {
		t.Errorf("one empty poll costs %d transfers and four cost %d, want %d",
			one, four, 4*one)
	}

	// And the descriptor read is the whole of it. Spelled out: the TAR write
	// and the RDBUFF that confirms it landed, then six DRW reads for the
	// 24-byte descriptor pipelined into each other, and the RDBUFF that drains
	// the last. Nine. Re-reading the header would be four more.
	if one != 9 {
		t.Errorf("an empty poll costs %d transfers, want 9 -- the descriptor read and nothing else", one)
	}
}
