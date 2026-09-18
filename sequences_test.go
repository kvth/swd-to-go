package swd_test

import (
	"context"
	"testing"

	"github.com/kvth/swd-to-go"
)

// recorder is a Probe that records the switching traffic and nothing else. The
// sequences are pure protocol -- fixed patterns, no target involved -- so what
// there is to check is that the bits and the order are right, and how many
// calls it took to send them.
type recorder struct {
	swd.Probe // nil: anything this does not implement is not called here

	calls int
	bits  []byte // every cycle sent or sampled, one byte per cycle, in order
}

func (r *recorder) Sequence(_ context.Context, numBits int, data []byte) error {
	r.calls++
	r.appendOut(numBits, data)
	return nil
}

func (r *recorder) SWDSequence(_ context.Context, seqs []swd.Sequence) ([]byte, error) {
	r.calls++
	var in []byte
	for _, s := range seqs {
		if s.Input {
			// An input sequence clocks the same cycles and samples them; the
			// values are the target's, so record only that they happened.
			for i := 0; i < int(s.NumCycles); i++ {
				r.bits = append(r.bits, 'i')
			}
			in = append(in, make([]byte, (int(s.NumCycles)+7)/8)...)
			continue
		}
		r.appendOut(int(s.NumCycles), s.Data)
	}
	return in, nil
}

// appendOut records numBits of data, LSB first, one byte per cycle.
func (r *recorder) appendOut(numBits int, data []byte) {
	for i := 0; i < numBits; i++ {
		r.bits = append(r.bits, '0'+(data[i/8]>>(i%8))&1)
	}
}

// SwitchAndSelect exists to save round trips, so both halves of that are worth
// pinning: it has to put exactly the same cycles on the wire as the four calls
// it replaces, and it has to do it in one call rather than four.
func TestSwitchAndSelectClocksTheSameCyclesInOneCall(t *testing.T) {
	ctx := context.Background()
	const target = 0x01002927

	var separate recorder
	for _, step := range []struct {
		name string
		fn   func(context.Context, swd.Probe) error
	}{
		{"JTAGToDormant", swd.JTAGToDormant},
		{"DormantToSWD", swd.DormantToSWD},
		{"LineReset", swd.LineReset},
	} {
		if err := step.fn(ctx, &separate); err != nil {
			t.Fatalf("%s: %v", step.name, err)
		}
	}
	if err := swd.SelectTarget(ctx, &separate, target); err != nil {
		t.Fatalf("SelectTarget: %v", err)
	}

	var combined recorder
	if err := swd.SwitchAndSelect(ctx, &combined, target); err != nil {
		t.Fatalf("SwitchAndSelect: %v", err)
	}

	if string(combined.bits) != string(separate.bits) {
		t.Errorf("SwitchAndSelect clocks %d cycles, the four separate calls clock %d, and they differ",
			len(combined.bits), len(separate.bits))
	}
	if separate.calls != 4 {
		t.Errorf("the four separate calls took %d probe calls, want 4", separate.calls)
	}
	if combined.calls != 1 {
		t.Errorf("SwitchAndSelect took %d probe calls, want 1 -- saving those is the whole point",
			combined.calls)
	}
}

// One sequence carries at most 64 cycles, and the switching patterns are
// longer than that, so the splitting is where this would break.
func TestSwitchAndSelectSplitsWithinTheSequenceLimit(t *testing.T) {
	var got recorder
	captured := &captureSeqs{recorder: &got}
	if err := swd.SwitchAndSelect(context.Background(), captured, 0x01002927); err != nil {
		t.Fatalf("SwitchAndSelect: %v", err)
	}
	for i, s := range captured.seqs {
		if s.NumCycles < 1 || s.NumCycles > 64 {
			t.Errorf("sequence %d asks for %d cycles, outside the 1..64 a probe accepts",
				i, s.NumCycles)
		}
		if !s.Input && len(s.Data) < (int(s.NumCycles)+7)/8 {
			t.Errorf("sequence %d has %d data bytes for %d cycles", i, len(s.Data), s.NumCycles)
		}
	}
}

type captureSeqs struct {
	*recorder
	seqs []swd.Sequence
}

func (c *captureSeqs) SWDSequence(ctx context.Context, seqs []swd.Sequence) ([]byte, error) {
	c.seqs = append(c.seqs, seqs...)
	return c.recorder.SWDSequence(ctx, seqs)
}
