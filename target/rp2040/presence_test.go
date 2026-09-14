package rp2040_test

import (
	"context"
	"errors"
	"strings"
	"testing"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/rp2040"
)

// Attach is the presence check, so what it rejects is the whole of what
// "nothing is there" means. Everything here is a DPIDR a real wire can produce.
func TestAttachChecksDPIDR(t *testing.T) {
	cases := []struct {
		name  string
		dpidr uint32
		want  string // a distinguishing fragment of the error, "" for success
	}{
		{"rp2040", 0x0BC12477, ""},
		// An ARM debug port, but not an RP2040's. Attach answers "an RP2040 is
		// there", so this is a failure and not a bring-up.
		{"other ARM DP", 0x5BA02477, "an RP2040 reports"},
		// A wire with nothing driving it floats to one rail, and the read
		// succeeds carrying that. Only the high one is expressible here:
		// simtarget reads a zero DPIDR option as "use the default".
		{"floating high", 0xFFFFFFFF, "an RP2040 reports"},
		// One bit off, which is what a near miss looks like.
		{"one bit off", 0x0BC12476, "an RP2040 reports"},
		// One nibble off an RP2040's, which is what a near miss looks like.
		{"wrong DP version", 0x0BC11477, "an RP2040 reports"},
	}

	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			target := simtarget.New(simtarget.Options{
				DPIDR:     c.dpidr,
				Multidrop: true,
				TargetSel: rp2040.TARGET_CORE_0,
			})
			_, err := rp2040.Attach(context.Background(), bitbang.New(target), rp2040.TARGET_CORE_0)

			if c.want == "" {
				if err != nil {
					t.Fatalf("DPIDR 0x%08x: %v", c.dpidr, err)
				}
				return
			}
			if err == nil {
				t.Fatalf("DPIDR 0x%08x was accepted", c.dpidr)
			}
			if !strings.Contains(err.Error(), c.want) {
				t.Errorf("error %q does not mention %q", err, c.want)
			}
		})
	}
}

// A position with nothing plugged into it: the TARGETSEL picks a target that
// is not on the wire, so the DPIDR read gets no acknowledge at all. That is an
// ordinary answer for a bus being polled, not a probe failure, and a caller
// telling the two apart needs the TransferError left in the chain to do it.
func TestAttachAbsentTargetIsNotPresent(t *testing.T) {
	target := simtarget.New(simtarget.Options{
		Multidrop: true,
		TargetSel: rp2040.TARGET_CORE_1, // ...but we ask for core 0
	})
	_, err := rp2040.Attach(context.Background(), bitbang.New(target), rp2040.TARGET_CORE_0)
	if err == nil {
		t.Fatal("attached to a target that is not on the wire")
	}
	var te *swd.TransferError
	if !errors.As(err, &te) {
		t.Fatalf("the TransferError underneath was discarded: %v", err)
	}
	if te.Status.Ok() {
		t.Errorf("TransferError reports %s, want a failure", te.Status)
	}
}

// Attach must still hand back a usable detach when it fails, so that a caller
// polling positions can defer it before checking the error.
func TestAttachAlwaysReturnsDetach(t *testing.T) {
	target := simtarget.New(simtarget.Options{
		DPIDR: 0xFFFFFFFF, Multidrop: true, TargetSel: rp2040.TARGET_CORE_0,
	})
	detach, err := rp2040.Attach(context.Background(), bitbang.New(target), rp2040.TARGET_CORE_0)
	if err == nil {
		t.Fatal("want a failure to detach from")
	}
	if detach == nil {
		t.Fatal("detach is nil after a failed attach")
	}
	if err := detach(context.Background()); err != nil {
		t.Errorf("detach: %v", err)
	}
}

func TestAttachRejectsAnUnknownCore(t *testing.T) {
	target := simtarget.New(simtarget.Options{
		Multidrop: true, TargetSel: rp2040.TARGET_CORE_0,
	})
	// A plausible typo: the right chip, an instance that does not exist.
	_, err := rp2040.Attach(context.Background(), bitbang.New(target), 0x21002927)
	if err == nil {
		t.Fatal("attached to a core that is not one of the three")
	}
	if !strings.Contains(err.Error(), "TARGET_CORE_0") {
		t.Errorf("error %q does not name the values it wanted", err)
	}
	// It must say so rather than looking like an empty position, which is what
	// a TARGETSEL nobody answers would otherwise be mistaken for.
	var te *swd.TransferError
	if errors.As(err, &te) {
		t.Errorf("a bad argument came back as a wire failure: %v", err)
	}
}
