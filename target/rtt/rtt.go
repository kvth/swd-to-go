// Package rtt is SEGGER RTT against a control block in target memory.
//
// RTT is a pair of ring buffers in the target's RAM. A control block names
// them; the target writes into an "up" buffer and reads from a "down" buffer,
// and the debugger moves the offsets along by reading and writing target memory
// over the MEM-AP while the CPU keeps running. Nothing about it needs the
// target halted, which is the whole appeal.
//
// The control block is read once, at [New], and what does not change afterwards
// is kept: the buffer counts are fixed when the target initialises the block,
// so re-reading them would be a round trip per poll spent confirming something
// already known. What is re-read every time is the pair of offsets, because
// those are the thing that moves.
package rtt

import (
	"bytes"
	"context"
	"errors"
	"fmt"
)

// The control block, as SEGGER lays it out:
//
//	struct SEGGER_RTT_CB {
//	  char     acID[16];              // "SEGGER RTT" and NUL padding
//	  int      MaxNumUpBuffers;
//	  int      MaxNumDownBuffers;
//	  RTT_BUFFER aUp[MaxNumUpBuffers];
//	  RTT_BUFFER aDown[MaxNumDownBuffers];
//	};
//
//	struct RTT_BUFFER {
//	  const char *sName;
//	  char       *pBuffer;
//	  unsigned    SizeOfBuffer;
//	  unsigned    WrOff;              // up: written by the target, down: by us
//	  unsigned    RdOff;              // up: written by us,         down: by the target
//	  unsigned    Flags;
//	};
const (
	// cbID includes the terminating NUL. Ten characters of it can turn up in a
	// string table or in a stale copy of the struct; eleven, aligned, is a good
	// deal less likely to.
	cbID = "SEGGER RTT\x00"

	cbOffMaxUp   = 16
	cbOffMaxDown = 20
	cbOffBuffers = 24

	descSize     = 24
	descOffName  = 0
	descOffAddr  = 4
	descOffSize  = 8
	descOffWrOff = 12
	descOffRdOff = 16
	descOffFlags = 20

	// maxBuffers is the sanity bound on the counts in the header. A block
	// claiming more than this is not one we found, it is whatever else happened
	// to spell out the ID. SEGGER's own default is three of each.
	maxBuffers = 64

	// nameMax is the longest channel name read back.
	nameMax = 32
)

// The two failures a caller is meant to tell apart from a broken link: the
// block is not there yet, and the channel is not backed yet. Both are states a
// target passes through on its way up, so a caller that polls has to be able to
// recognise them. Everything else this package can fail with is a plain error,
// because there is nothing to do with it but report it.
var (
	// ErrNotFound means no control block was found, or what is at the address
	// given is not one.
	ErrNotFound = errors.New("rtt: no SEGGER RTT control block")
	// ErrNoBuffer means the channel exists but the target never gave it
	// storage. SEGGER's default block reserves spare slots, so this is a
	// normal state rather than damage.
	ErrNoBuffer = errors.New("rtt: the channel has no buffer")
)

var (
	// errCorrupt means a descriptor's offsets are outside its own buffer.
	errCorrupt = errors.New("rtt: the channel descriptor is corrupt")
	// errBadChannel means the channel number is beyond what the block declares.
	errBadChannel = errors.New("rtt: no such channel")
)

// Direction picks which set of buffers a channel number refers to.
type Direction int

const (
	// Up is target to host.
	Up Direction = iota
	// Down is host to target.
	Down
)

func (d Direction) String() string {
	if d == Down {
		return "down"
	}
	return "up"
}

// Memory is the part of a MEM-AP client this package needs: target memory at
// byte granularity, one word for a ring pointer, and the sticky-error clear
// the scan depends on to step over unmapped addresses. target/mem's Client is
// one.
//
// It is declared here, and narrowed to these four calls, so that RTT depends
// on a way of reaching target memory rather than on target/mem's
// implementation of one.
type Memory interface {
	ReadTargetMemBytes(ctx context.Context, addr uint32, length int) ([]byte, error)
	WriteTargetMemBytes(ctx context.Context, addr uint32, data []byte) error
	WriteTargetReg(ctx context.Context, addr uint32, value uint32) error

	// ClearErrors clears the debug port's sticky error bits. A read of
	// unmapped memory faults, and every later access fails until this is
	// called -- which is what [FindRTTControlBlock] does on its way past a
	// hole.
	ClearErrors(ctx context.Context) error
}

// RTT is an attached control block.
//
// It is not safe for concurrent use: the ring-pointer dance is read-then-write
// against a MEM-AP client that is itself single-threaded.
type RTT struct {
	memap  Memory
	cbAddr uint32

	// Fixed at target init, so read once and kept. See the package comment.
	numUp, numDown uint32
}

// New attaches to a control block already located in target memory, and fails
// if what is there is not one.
func New(ctx context.Context, memap Memory, cbAddr uint32) (*RTT, error) {
	numUp, numDown, err := readHeader(ctx, memap, cbAddr)
	if err != nil {
		return nil, err
	}
	return &RTT{memap: memap, cbAddr: cbAddr, numUp: numUp, numDown: numDown}, nil
}

// Addr is where the control block was found.
func (r *RTT) Addr() uint32 { return r.cbAddr }

// Channels is how many up and down channels the block declares. Some of them
// may have no buffer; see [ErrNoBuffer].
func (r *RTT) Channels() (up, down int) { return int(r.numUp), int(r.numDown) }

// CheckControlBlock reports whether a control block is at cbAddr.
func CheckControlBlock(ctx context.Context, memap Memory, cbAddr uint32) error {
	_, _, err := readHeader(ctx, memap, cbAddr)
	return err
}

// readHeader is both halves of recognising a control block: the ID string, and
// buffer counts that could plausibly be real. The ID alone is not enough --
// the string is in the target's own binary too, wherever SEGGER's code put it.
func readHeader(ctx context.Context, memap Memory, cbAddr uint32) (numUp, numDown uint32, err error) {
	// The ID and both counts in one read: the counts are at offsets 16 and 20,
	// so the whole of what identifies a block is the first 24 bytes.
	b, err := memap.ReadTargetMemBytes(ctx, cbAddr, cbOffBuffers)
	if err != nil {
		return 0, 0, fmt.Errorf("read control block at 0x%08x: %w", cbAddr, err)
	}
	if !bytes.HasPrefix(b, []byte(cbID)) {
		return 0, 0, fmt.Errorf("%w at 0x%08x", ErrNotFound, cbAddr)
	}

	numUp = le32(b[cbOffMaxUp:])
	numDown = le32(b[cbOffMaxDown:])
	if numUp > maxBuffers || numDown > maxBuffers || numUp+numDown == 0 {
		return 0, 0, fmt.Errorf("%w at 0x%08x: it claims %d up and %d down channels",
			ErrNotFound, cbAddr, numUp, numDown)
	}
	return numUp, numDown, nil
}

// FindRTTControlBlock scans target memory from startAddr for a control block
// and returns its address.
//
// searchSize is how far to look and blockSize how much to read at a time.
// Ranges that are partly unmapped are fine, which matters because the useful
// thing to pass is a whole RAM window: a read that faults is stepped over
// rather than ending the search.
func FindRTTControlBlock(ctx context.Context, memap Memory,
	startAddr uint32, searchSize int, blockSize int) (uint32, error) {

	if searchSize < len(cbID) {
		return 0, fmt.Errorf("%w: a %d-byte range cannot hold one", ErrNotFound, searchSize)
	}
	if blockSize < len(cbID) {
		blockSize = 512
	}

	if found, ok, err := scan(ctx, memap, startAddr, searchSize, blockSize); err != nil {
		return 0, err
	} else if ok {
		return found, nil
	}
	return 0, fmt.Errorf("%w in 0x%08x-0x%08x",
		ErrNotFound, startAddr, startAddr+uint32(searchSize))
}

// scan walks [start, start+size) looking for the signature.
//
// The reads are of one whole block each, starting at the range's own start and
// stepping by exactly the block size, and that is deliberate. A caller hands
// this a RAM window, which means ranges that are partly unmapped; a read that
// faults is stepped over rather than ending the search, and stepping in whole
// blocks is what keeps the fault confined to blocks that really are
// unmapped. Chunks staggered to overlap each other would instead take mapped
// memory down with a fault, and a control block a few bytes past the end of a
// hole would be invisible: inside the chunk that faulted, and before the one
// after it.
//
// An ID lying across a block boundary is still found, without re-reading
// anything: the last few bytes of each block are carried forward and searched
// together with the next one.
func scan(ctx context.Context, memap Memory,
	start uint32, size int, block int) (uint32, bool, error) {

	id := []byte(cbID)
	overlap := len(id) - 1

	var (
		tail     []byte // trailing bytes of the previous block
		tailAddr uint32
	)

	for off := 0; off < size; off += block {
		n := size - off
		if n > block {
			n = block
		}
		at := start + uint32(off)

		data, err := memap.ReadTargetMemBytes(ctx, at, n)
		if err != nil {
			// Not memory the AP can reach. Shake the sticky error off -- every
			// later transfer would fail with it set -- and carry on. Nothing
			// carries across a hole.
			if clearErr := memap.ClearErrors(ctx); clearErr != nil {
				return 0, false, fmt.Errorf("scan 0x%08x: %w", at, err)
			}
			tail = nil
			continue
		}

		buf, bufAddr := data, at
		if tail != nil && tailAddr+uint32(len(tail)) == at {
			buf = append(tail, data...)
			bufAddr = tailAddr
		}

		for i := 0; i+len(id) <= len(buf); i++ {
			if !bytes.HasPrefix(buf[i:], id) {
				continue
			}
			// The ID is necessary but not sufficient -- the string is in the
			// target's own binary too, wherever SEGGER's code put it. Check the
			// header before believing it, and carry on if it does not hold up.
			candidate := bufAddr + uint32(i)
			if _, _, err := readHeader(ctx, memap, candidate); err == nil {
				return candidate, true, nil
			}
		}

		// Carry the tail forward. The copy matters: `data` is about to be
		// appended to on the next pass, and a subslice of it would be written
		// through.
		if len(data) >= overlap {
			tail = append([]byte(nil), data[len(data)-overlap:]...)
			tailAddr = at + uint32(n-overlap)
		} else {
			tail = nil
		}
	}
	return 0, false, nil
}

// Info is one channel's descriptor as it stood when it was read.
type Info struct {
	// Desc is where the descriptor itself lives, which is what the offsets are
	// written back through.
	Desc uint32
	// Name is the channel name the target set, empty if it set none or if it
	// could not be read.
	Name string
	// Buffer and Size are the ring itself.
	Buffer, Size uint32
	// WrOff and RdOff are the ring's two offsets. For an up channel the target
	// moves WrOff and we move RdOff; for a down channel it is the other way
	// round.
	WrOff, RdOff uint32
	Flags        uint32
}

// Pending is how many bytes are waiting to be read out of an up channel.
func (i Info) Pending() uint32 {
	if i.Size == 0 {
		return 0
	}
	return (i.WrOff - i.RdOff + i.Size) % i.Size
}

// Free is how much room is left in a down channel. One slot is always kept
// empty, so that a full ring and an empty one do not both have the two offsets
// equal.
func (i Info) Free() uint32 {
	if i.Size == 0 {
		return 0
	}
	return i.Size - 1 - i.Pending()
}

// Channel reads one channel's descriptor, including its name. It is what a
// caller lists channels with; [RTT.Read] and [RTT.Write] read the descriptor
// themselves and do not need it.
func (r *RTT) Channel(ctx context.Context, dir Direction, channel int) (Info, error) {
	d, err := r.readDesc(ctx, dir, channel)
	if err != nil {
		return Info{}, err
	}
	if d.nameAddr != 0 {
		// A name that will not read is not worth failing over: the offsets are
		// what a caller is here for.
		if raw, err := r.memap.ReadTargetMemBytes(ctx, d.nameAddr, nameMax); err == nil {
			if end := bytes.IndexByte(raw, 0); end >= 0 {
				d.info.Name = string(raw[:end])
			} else {
				d.info.Name = string(raw)
			}
		} else {
			_ = r.memap.ClearErrors(ctx)
		}
	}
	return d.info, nil
}

// desc is a descriptor plus the bits of it only this package needs.
type desc struct {
	info     Info
	nameAddr uint32
}

func (r *RTT) readDesc(ctx context.Context, dir Direction, channel int) (desc, error) {
	limit, base := r.numUp, uint32(0)
	if dir == Down {
		limit, base = r.numDown, r.numUp
	}
	if channel < 0 || uint32(channel) >= limit {
		return desc{}, fmt.Errorf("%w: %s channel %d, the block declares %d",
			errBadChannel, dir, channel, limit)
	}

	at := r.cbAddr + cbOffBuffers + (base+uint32(channel))*descSize
	b, err := r.memap.ReadTargetMemBytes(ctx, at, descSize)
	if err != nil {
		return desc{}, fmt.Errorf("read %s channel %d descriptor: %w", dir, channel, err)
	}

	return desc{
		nameAddr: le32(b[descOffName:]),
		info: Info{
			Desc:   at,
			Buffer: le32(b[descOffAddr:]),
			Size:   le32(b[descOffSize:]),
			WrOff:  le32(b[descOffWrOff:]),
			RdOff:  le32(b[descOffRdOff:]),
			Flags:  le32(b[descOffFlags:]),
		},
	}, nil
}

// usable rejects a channel that cannot be worked with, separating "the target
// never gave this one storage" from "the offsets are nonsense". Without this an
// unbacked channel divides by a zero buffer size.
func usable(i Info) error {
	if i.Buffer == 0 || i.Size == 0 {
		return ErrNoBuffer
	}
	if i.WrOff >= i.Size || i.RdOff >= i.Size {
		return fmt.Errorf("%w: offsets %d/%d in a %d-byte buffer", errCorrupt, i.WrOff, i.RdOff, i.Size)
	}
	return nil
}

// Read drains an up channel and advances the target's read offset by what it
// took. It returns nil with no error when the channel had nothing pending.
//
// The ring is drained in one call rather than one run per call: up to two
// reads, from the read offset to whichever comes first of the write offset and
// the end of the ring, and then from the start of the ring.
func (r *RTT) Read(ctx context.Context, channel int) ([]byte, error) {
	d, err := r.readDesc(ctx, Up, channel)
	if err != nil {
		return nil, err
	}
	if err := usable(d.info); err != nil {
		return nil, fmt.Errorf("up channel %d: %w", channel, err)
	}

	i := d.info
	if i.WrOff == i.RdOff {
		return nil, nil
	}

	out := make([]byte, 0, i.Pending())
	readOff := i.RdOff
	for readOff != i.WrOff {
		end := i.Size
		if i.WrOff > readOff {
			end = i.WrOff
		}
		chunk, err := r.memap.ReadTargetMemBytes(ctx, i.Buffer+readOff, int(end-readOff))
		if err != nil {
			return nil, fmt.Errorf("read up channel %d: %w", channel, err)
		}
		out = append(out, chunk...)

		readOff = end
		if readOff == i.Size {
			readOff = 0
		}
	}

	// Publish the new read offset only once the data is in hand, so a transfer
	// that failed leaves the ring where it was and the bytes still there.
	if err := r.memap.WriteTargetReg(ctx, i.Desc+descOffRdOff, readOff); err != nil {
		return nil, fmt.Errorf("advance up channel %d read offset: %w", channel, err)
	}
	return out, nil
}

// Write appends to a down channel, as much of data as fits, and returns how
// much went in. A caller with more to send retries with the remainder: the
// buffer only drains as the target reads it.
func (r *RTT) Write(ctx context.Context, channel int, data []byte) (int, error) {
	d, err := r.readDesc(ctx, Down, channel)
	if err != nil {
		return 0, err
	}
	if err := usable(d.info); err != nil {
		return 0, fmt.Errorf("down channel %d: %w", channel, err)
	}

	i := d.info
	want := uint32(len(data))
	if free := i.Free(); want > free {
		want = free
	}
	if want == 0 {
		return 0, nil
	}

	writeOff := i.WrOff
	written := uint32(0)
	for written < want {
		// Stop at the read offset or the end of the ring, whichever comes
		// first. The slot before RdOff is the one kept empty.
		end := i.Size
		if i.RdOff > writeOff {
			end = i.RdOff - 1
		}
		run := end - writeOff
		if run > want-written {
			run = want - written
		}
		if run == 0 {
			break
		}

		if err := r.memap.WriteTargetMemBytes(ctx, i.Buffer+writeOff, data[written:written+run]); err != nil {
			return int(written), fmt.Errorf("write down channel %d: %w", channel, err)
		}
		written += run
		writeOff += run
		if writeOff == i.Size {
			writeOff = 0
		}
	}

	// Publish the new write offset only once the data is in memory, or the
	// target reads bytes that are not there yet.
	if err := r.memap.WriteTargetReg(ctx, i.Desc+descOffWrOff, writeOff); err != nil {
		return int(written), fmt.Errorf("advance down channel %d write offset: %w", channel, err)
	}
	return int(written), nil
}

func le32(b []byte) uint32 {
	return uint32(b[0]) | uint32(b[1])<<8 | uint32(b[2])<<16 | uint32(b[3])<<24
}
