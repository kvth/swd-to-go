package swd

import (
	"context"
	"encoding/binary"
)

// The switching sequences below are fixed bit patterns from the ARM Debug
// Interface spec. They are free functions rather than Probe methods because
// they are pure protocol: every probe drives them the same way, through
// Sequence, and none of them has a reason to override one.

// The patterns themselves, shared by the one-step functions below and by
// [SwitchAndSelect], so that the combined form cannot drift from the separate
// ones.
var (
	// At least 50 SWCLK cycles with SWDIO high, then idle it low.
	seqLineReset = []byte{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00}

	seqJTAGToDormant = []byte{
		// At least 5 TCK cycles with TMS high.
		0xff,
		// One more TCK cycle with TMS high, then the 31-bit JTAG-to-DS select
		// sequence 0xba 0xbb 0xbb 0x33 shifted up by one to make room for it.
		0x75,
		0x77,
		0x77,
		0x67,
	}

	seqDormantToSWD = []byte{
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

	seqSWDToDormant = []byte{
		// At least 50 SWCLK cycles with SWDIO high.
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		// Switching sequence from SWD to dormant.
		0xbc, 0xe3,
	}
)

// LineReset holds SWDIO high for at least 50 SWCLK cycles and then idles it
// low, which returns the debug port to its reset state.
func LineReset(ctx context.Context, p Probe) error {
	return p.Sequence(ctx, len(seqLineReset)*8, seqLineReset)
}

// JTAGToDormant takes a port that came up in JTAG mode to the dormant state.
func JTAGToDormant(ctx context.Context, p Probe) error {
	return p.Sequence(ctx, len(seqJTAGToDormant)*8, seqJTAGToDormant)
}

// DormantToSWD takes a dormant port to SWD, which is where every bring-up
// here starts and the only way onto a multidrop bus.
func DormantToSWD(ctx context.Context, p Probe) error {
	return p.Sequence(ctx, len(seqDormantToSWD)*8, seqDormantToSWD)
}

// SWDToDormant takes an SWD port back to dormant, which is what you owe the
// bus before handing it to anyone else on a multidrop target.
func SWDToDormant(ctx context.Context, p Probe) error {
	return p.Sequence(ctx, len(seqSWDToDormant)*8, seqSWDToDormant)
}

// SelectTarget writes TARGETSEL on a multidrop bus, which picks which of the
// targets sharing the wire answers from here on.
//
// It has to go out as a raw sequence rather than an ordinary transfer: every
// target on the bus sees the request, all but one of them stay quiet, and so
// the ACK phase has to be clocked past and ignored. A transfer would call that
// a protocol error and retry it.
func SelectTarget(ctx context.Context, p Probe, target uint32) error {
	_, err := p.SWDSequence(ctx, targetSelSequences(target))
	return err
}

// SwitchAndSelect is the whole bring-up dance as a single call to the probe:
// JTAG to dormant, dormant to SWD, a line reset, and the multidrop TARGETSEL
// that picks which target answers. It clocks exactly what [JTAGToDormant],
// [DormantToSWD], [LineReset] and [SelectTarget] clock in that order, off the
// same patterns.
//
// The point is the call count rather than the cycles: the four separate calls
// are four exchanges with a probe on the far end of a socket, where this is
// one. On a multiplexed wire that is three round trips saved every time a
// position is visited, and the bits on the wire are the same either way.
//
// A target that is not on a multidrop bus wants the first three and not this:
// there is nothing to select, and TARGETSEL would go unanswered.
func SwitchAndSelect(ctx context.Context, p Probe, target uint32) error {
	seqs := make([]Sequence, 0, 8)
	seqs = appendOut(seqs, seqJTAGToDormant)
	seqs = appendOut(seqs, seqDormantToSWD)
	seqs = appendOut(seqs, seqLineReset)
	seqs = append(seqs, targetSelSequences(target)...)

	_, err := p.SWDSequence(ctx, seqs)
	return err
}

// appendOut splits a pattern into output sequences, since one carries at most
// 64 cycles and the switching patterns are longer than that.
func appendOut(dst []Sequence, pattern []byte) []Sequence {
	for len(pattern) > 0 {
		n := len(pattern)
		if n > 8 {
			n = 8
		}
		dst = append(dst, Sequence{NumCycles: uint8(n * 8), Data: pattern[:n]})
		pattern = pattern[n:]
	}
	return dst
}

// targetSelSequences is the TARGETSEL write, which [SelectTarget] sends on its
// own and [SwitchAndSelect] sends after the switching patterns.
func targetSelSequences(target uint32) []Sequence {
	return []Sequence{
		// The SWD header for a DP write to TARGETSEL, 8 clocks out.
		{NumCycles: 8, Data: []byte{0x99}},
		// Turnaround plus the ACK phase, 5 clocks in, thrown away.
		{NumCycles: 5, Input: true},
		// The 32-bit value and its parity bit, 33 clocks out.
		{NumCycles: 33, Data: packTargetSEL(target)},
	}
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
