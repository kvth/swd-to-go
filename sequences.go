package swd

import (
	"context"
	"encoding/binary"
)

// The switching sequences below are fixed bit patterns from the ARM Debug
// Interface spec. They are free functions rather than Probe methods because
// they are pure protocol: every probe drives them the same way, through
// Sequence, and none of them has a reason to override one.

// LineReset holds SWDIO high for at least 50 SWCLK cycles and then idles it
// low, which returns the debug port to its reset state.
func LineReset(ctx context.Context, p Probe) error {
	seq := []byte{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00}
	return p.Sequence(ctx, len(seq)*8, seq)
}

// JTAGToDormant takes a port that came up in JTAG mode to the dormant state.
func JTAGToDormant(ctx context.Context, p Probe) error {
	seq := []byte{
		// At least 5 TCK cycles with TMS high.
		0xff,
		// One more TCK cycle with TMS high, then the 31-bit JTAG-to-DS select
		// sequence 0xba 0xbb 0xbb 0x33 shifted up by one to make room for it.
		0x75,
		0x77,
		0x77,
		0x67,
	}
	return p.Sequence(ctx, len(seq)*8, seq)
}

// DormantToSWD takes a dormant port to SWD.
func DormantToSWD(ctx context.Context, p Probe) error {
	seq := []byte{
		// At least 8 SWCLK cycles with SWDIO high.
		0xff,
		// Selection alert sequence.
		0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86,
		0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e, 0xbc, 0x19,
		// 4 SWCLK cycles low, the SWD activation code 0x1a, then at least 8
		// cycles high.
		0xa0,
		0xf1,
		0xff,
		// At least 50 SWCLK cycles with SWDIO high.
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		// At least 2 idle cycles.
		0x00,
	}
	return p.Sequence(ctx, len(seq)*8, seq)
}

// SWDToDormant takes an SWD port back to dormant, which is what you owe the
// bus before handing it to anyone else on a multidrop target.
func SWDToDormant(ctx context.Context, p Probe) error {
	seq := []byte{
		// At least 50 SWCLK cycles with SWDIO high.
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		// Switching sequence from SWD to dormant.
		0xbc, 0xe3,
	}
	return p.Sequence(ctx, len(seq)*8, seq)
}

// SelectTarget writes TARGETSEL on a multidrop bus, which picks which of the
// targets sharing the wire answers from here on.
//
// It has to go out as a raw sequence rather than an ordinary transfer: every
// target on the bus sees the request, all but one of them stay quiet, and so
// the ACK phase has to be clocked past and ignored. A transfer would call that
// a protocol error and retry it.
func SelectTarget(ctx context.Context, p Probe, target uint32) error {
	_, err := p.SWDSequence(ctx, []Sequence{
		// The SWD header for a DP write to TARGETSEL, 8 clocks out.
		{NumCycles: 8, Data: []byte{0x99}},
		// Turnaround plus the ACK phase, 5 clocks in, thrown away.
		{NumCycles: 5, Input: true},
		// The 32-bit value and its parity bit, 33 clocks out.
		{NumCycles: 33, Data: packTargetSEL(target)},
	})
	return err
}

func packTargetSEL(v uint32) []byte {
	b := make([]byte, 5)
	binary.LittleEndian.PutUint32(b[:4], v)
	b[4] = parity32(v)
	return b
}

func parity32(v uint32) uint8 {
	v ^= v >> 16
	v ^= v >> 8
	v ^= v >> 4
	v ^= v >> 2
	v ^= v >> 1
	return uint8(v & 1)
}
