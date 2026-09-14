package simtarget

import (
	"encoding/binary"
	"sync"
	"time"
)

// A simulated target running SEGGER RTT: a control block in RAM whose down
// channel is drained into its up channel in upper case, the way sim_probe's
// does. It stands in for firmware that answers what it is sent, which is what a
// fused node query's write-then-poll-read cycle needs something on the other end
// to do.

// RTT control block layout, repeated here rather than imported so the simulated
// target does not depend on the code under test for what the wire looks like.
const (
	rttCBOffMaxUp   = 16
	rttCBOffMaxDown = 20
	rttCBOffBuffers = 24

	rttBufDescSize = 24
	rttBufOffName  = 0
	rttBufOffAddr  = 4
	rttBufOffSize  = 8
	rttBufOffWrOff = 12
	rttBufOffRdOff = 16
	rttBufOffFlags = 20
)

// RTTLayout says where a simulated control block and its two ring buffers live.
type RTTLayout struct {
	CBAddr   uint32
	NameAddr uint32
	UpAddr   uint32
	DownAddr uint32
	BufSize  uint32
}

// RTTBlock is a control block laid out in a target's memory, and the echo that
// makes it answer.
type RTTBlock struct {
	target *Target
	layout RTTLayout

	stop chan struct{}
	done chan struct{}
	once sync.Once
}

// NewRTTBlock writes a control block with one up and one down channel into the
// target's memory, both empty.
func NewRTTBlock(t *Target, layout RTTLayout, name string) *RTTBlock {
	b := &RTTBlock{target: t, layout: layout}

	id := make([]byte, 16)
	copy(id, "SEGGER RTT")
	t.WriteMem(layout.CBAddr, id)
	b.writeU32(layout.CBAddr+rttCBOffMaxUp, 1)
	b.writeU32(layout.CBAddr+rttCBOffMaxDown, 1)

	t.WriteMem(layout.NameAddr, append([]byte(name), 0))

	for i, bufAddr := range []uint32{layout.UpAddr, layout.DownAddr} {
		desc := layout.CBAddr + rttCBOffBuffers + uint32(i)*rttBufDescSize
		b.writeU32(desc+rttBufOffName, layout.NameAddr)
		b.writeU32(desc+rttBufOffAddr, bufAddr)
		b.writeU32(desc+rttBufOffSize, layout.BufSize)
		b.writeU32(desc+rttBufOffWrOff, 0)
		b.writeU32(desc+rttBufOffRdOff, 0)
		b.writeU32(desc+rttBufOffFlags, 0)
	}

	return b
}

// UpDesc and DownDesc are where the two channel descriptors live, for a test
// that wants to inspect or move the offsets itself.
func (b *RTTBlock) UpDesc() uint32 {
	return b.layout.CBAddr + rttCBOffBuffers
}

func (b *RTTBlock) DownDesc() uint32 {
	return b.layout.CBAddr + rttCBOffBuffers + rttBufDescSize
}

func (b *RTTBlock) writeU32(addr, value uint32) {
	var raw [4]byte
	binary.LittleEndian.PutUint32(raw[:], value)
	b.target.WriteMem(addr, raw[:])
}

func (b *RTTBlock) readU32(addr uint32) uint32 {
	raw := b.target.ReadMem(addr, 4)
	if raw == nil {
		return 0
	}
	return binary.LittleEndian.Uint32(raw)
}

// StartEcho runs the simulated firmware: whatever turns up on the down channel
// is read out and written back on the up channel, in upper case, the way a
// target answering a request would. Stop it with Stop.
func (b *RTTBlock) StartEcho(transform func([]byte) []byte) {
	if transform == nil {
		transform = upperCase
	}
	b.stop = make(chan struct{})
	b.done = make(chan struct{})

	go func() {
		defer close(b.done)
		for {
			select {
			case <-b.stop:
				return
			default:
			}
			if data := b.DrainDown(); len(data) > 0 {
				b.FillUp(transform(data))
			}
			time.Sleep(100 * time.Microsecond)
		}
	}()
}

// Stop ends the echo.
func (b *RTTBlock) Stop() {
	b.once.Do(func() {
		if b.stop == nil {
			return
		}
		close(b.stop)
		<-b.done
	})
}

// DrainDown is the target's side of the down ring: read everything the host put
// there and move the read offset past it. Exported so that a test driving the
// host side by hand can play the target without running the echo.
func (b *RTTBlock) DrainDown() []byte {
	desc := b.DownDesc()
	bufAddr := b.readU32(desc + rttBufOffAddr)
	size := b.readU32(desc + rttBufOffSize)
	wr := b.readU32(desc + rttBufOffWrOff)
	rd := b.readU32(desc + rttBufOffRdOff)
	if size == 0 || wr == rd {
		return nil
	}

	var out []byte
	for rd != wr {
		end := size
		if wr > rd {
			end = wr
		}
		chunk := b.target.ReadMem(bufAddr+rd, int(end-rd))
		if chunk == nil {
			return out
		}
		out = append(out, chunk...)
		rd = end % size
	}

	b.writeU32(desc+rttBufOffRdOff, rd)
	return out
}

// FillUp is the target's side of the up ring: append as much as fits, leaving
// the one empty slot that tells a full ring from an empty one.
func (b *RTTBlock) FillUp(data []byte) {
	desc := b.UpDesc()
	bufAddr := b.readU32(desc + rttBufOffAddr)
	size := b.readU32(desc + rttBufOffSize)
	wr := b.readU32(desc + rttBufOffWrOff)
	rd := b.readU32(desc + rttBufOffRdOff)
	if size == 0 {
		return
	}

	free := size - wr + rd - 1
	if rd > wr {
		free = rd - wr - 1
	}
	if uint32(len(data)) > free {
		data = data[:free]
	}

	for len(data) > 0 {
		end := size
		if rd > wr {
			end = rd - 1
		}
		run := int(end - wr)
		if run > len(data) {
			run = len(data)
		}
		if run <= 0 {
			break
		}
		b.target.WriteMem(bufAddr+wr, data[:run])
		data = data[run:]
		wr = (wr + uint32(run)) % size
	}

	b.writeU32(desc+rttBufOffWrOff, wr)
}

func upperCase(in []byte) []byte {
	out := make([]byte, len(in))
	for i, c := range in {
		if c >= 'a' && c <= 'z' {
			c -= 'a' - 'A'
		}
		out[i] = c
	}
	return out
}
