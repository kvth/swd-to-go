package rp2040

import (
	"context"
	"fmt"

	"github.com/kvth/swd-to-go"
)

// Attach brings the wire up and selects one core of an RP2040. It is the
// bring-up and nothing else: no clock is set, no debug port is powered, and no
// MEM-AP is initialised. Those are separate steps, taken by whoever needs them
// -- see the package comment.
//
// The order is not negotiable. The port may come up in JTAG, so it goes to
// dormant first, then to SWD, then takes a line reset; only then can TARGETSEL
// pick a core, because the RP2040 puts both of them plus a rescue DP on one
// multidrop bus and none of them answers until it has been addressed. Anything
// else and the target either stays silent or answers for the wrong core.
//
// The DPIDR read at the end is not optional either, and it is what makes this
// a presence check as well as a bring-up. A multidrop target does not consider
// itself selected until its DPIDR has been read, and a DP coming out of a line
// reset ignores everything else until that read has happened -- so the cheapest
// legal thing to do here is also the thing that proves a target is on the wire.
// One transfer. There is nothing faster; see [swd.Probe.Transfer].
//
// A caller polling a bus of muxed targets to see which are populated wants
// exactly this call and nothing after it. An error means nothing answered.
//
// detach is never nil, even when the error is not: a caller can defer it
// straight away. It returns the bus to the dormant state, which is what
// another debugger on this multidrop target needs in order to take it over
// without a power cycle. Leaving it in SWD is not fatal -- the next debugger's
// bring-up starts with a line reset -- but it is untidy and costs that debugger
// a retry.
//
// core must be one of [TARGET_CORE_0], [TARGET_CORE_1] or [TARGET_RESCUE].
// Anything else is rejected before the wire is touched: a TARGETSEL nobody
// answers is indistinguishable from an empty position, so a typo would come
// back as "nothing is plugged in" and be believed.
//
// Set the clock before calling this, with [swd.Probe.SetClock]. Bring-up runs
// at whatever the probe is already set to.
func Attach(ctx context.Context, probe swd.Probe, core uint32) (detach func(context.Context) error, err error) {
	detach = func(ctx context.Context) error { return swd.SWDToDormant(ctx, probe) }

	switch core {
	case TARGET_CORE_0, TARGET_CORE_1, TARGET_RESCUE:
	default:
		return detach, fmt.Errorf("core 0x%08x is not one of TARGET_CORE_0, TARGET_CORE_1 or TARGET_RESCUE", core)
	}

	if err := probe.Connect(ctx); err != nil {
		return detach, fmt.Errorf("connect: %w", err)
	}
	if err := swd.JTAGToDormant(ctx, probe); err != nil {
		return detach, fmt.Errorf("JTAG to dormant: %w", err)
	}
	if err := swd.DormantToSWD(ctx, probe); err != nil {
		return detach, fmt.Errorf("dormant to SWD: %w", err)
	}
	if err := swd.LineReset(ctx, probe); err != nil {
		return detach, fmt.Errorf("line reset: %w", err)
	}
	if err := swd.SelectTarget(ctx, probe, core); err != nil {
		return detach, fmt.Errorf("select target 0x%08x: %w", core, err)
	}
	if err := checkIDR(ctx, probe); err != nil {
		return detach, fmt.Errorf("target 0x%08x: %w", core, err)
	}
	return detach, nil
}

// checkIDR reads DPIDR and requires it be an RP2040's.
//
// The value is compared and not just the error, because an empty position can
// answer without failing at all: an undriven SWD line floats to one rail or the
// other, so the read succeeds and carries 0x00000000 or 0xffffffff. Neither is
// an RP2040's DPIDR.
//
// A failed read is wrapped as it is. A caller polling a bus that wants to tell
// a position with nothing in it from a probe that has died can look for a
// [swd.TransferError] in the chain: that is the wire answering, where a dead
// socket or a cancelled context is not.
func checkIDR(ctx context.Context, probe swd.Probe) error {
	idr, err := dpRead(ctx, probe, dpIDR)
	if err != nil {
		return fmt.Errorf("read DPIDR: %w", err)
	}
	if idr != dpidr {
		return fmt.Errorf("DPIDR is 0x%08x, and an RP2040 reports 0x%08x", idr, dpidr)
	}
	return nil
}
