// Package rp2040 brings up an RP2040's debug port and programs its flash
// through the bootrom.
//
// The chip has no flash controller a debugger could drive. Its bootrom exports
// the routines that talk to the external QSPI part instead, and a host programs
// flash by halting a core, pointing it at one of those routines and letting it
// run. That is what [Flasher] does here.
//
// The pieces, in the order a caller uses them:
//
//   - [Attach] does the bring-up and only the bring-up: dormant, SWD, line
//     reset, the multidrop TARGETSEL that picks which of the two cores (or the
//     rescue port) answers, and the DPIDR read that completes it. It returns a
//     detach func. It sets no clock -- do that yourself first, with
//     [swd.Probe.SetClock] -- and it initialises nothing above the wire.
//   - [Rescue] is the way back in when the code on the target is what is
//     stopping you — it resets the chip and stops it in the bootrom before
//     anything runs out of flash.
//   - [Flasher] halts the core, finds the bootrom's function table and runs
//     the flash routines. [Flasher.FlashImage] is the whole write in one call,
//     with the preverify, verify and reset steps OpenOCD's `program` has.
//
// The debug port and the MEM-AP are the caller's to build, in between:
//
//	probe.SetClock(ctx, 1_500_000)
//	detach, err := rp2040.Attach(ctx, probe, rp2040.TARGET_CORE_0)
//	if err != nil {
//		return err // nothing answered: no target on this wire
//	}
//	defer detach(context.WithoutCancel(ctx))
//
//	dpc := dp.New(probe)          // powers the debug domains up
//	if err := dpc.Init(ctx); err != nil {
//		return err
//	}
//	apc := mem.New(dpc, 0)        // one round trip, plus the byte-access probe
//	if err := apc.Init(ctx); err != nil {
//		return err
//	}
//
// They are separate steps because plenty of work does not need them. Polling a
// bus to see which targets are populated is [Attach] and nothing else — one
// transfer past the switching sequences — and paying for a debug-port power-up
// and a MEM-AP probe on every poll of every position adds up on a board with a
// dozen of them. Bring them up on the runs that have something to say.
package rp2040

// The three debug ports on an RP2040's multidrop SWD wire.
//
// These are TARGETID values, and TARGETID is the register that names the chip
// -- so each of these spells out "Raspberry Pi, part 0x1002, instance N":
//
//	0x01002927
//	  |  |  ||
//	  |  |  |+- bit 0, read as one
//	  |  |  +-- TDESIGNER 0x493, JEP106 continuation 9 identity 0x13:
//	  |  |      Raspberry Pi
//	  |  +----- TPARTNO 0x1002: RP2040
//	  +-------- TINSTANCE: which of the three ports below
//
// That matters for what a bring-up can conclude. TARGETSEL is written with the
// TARGETID that should answer and the rest of the wire stays quiet, so a target
// that answers one of these has already asserted it is an RP2040 -- the
// identity check is the addressing, which is why nothing here reads TARGETID
// back. The DPIDR is checked on top of it.
const (
	// TARGET_CORE_0 is core 0's debug port, and the one to use unless you have
	// a reason not to: both cores see the same bootrom and the same SRAM, so
	// either can flash.
	TARGET_CORE_0 uint32 = 0x01002927
	// TARGET_CORE_1 is core 1's.
	TARGET_CORE_1 uint32 = 0x11002927
	// TARGET_RESCUE is not a core's debug port at all. See [Rescue].
	TARGET_RESCUE uint32 = 0xf1002927
)

// dpidr is what an RP2040's debug port answers, and what [Attach] and [Rescue]
// require before they will believe there is one on the wire.
//
// All three ports above report it. DPIDR describes the debug port, not the
// chip -- designer ARM, ARM's part number 0xbc for the SW-DP design, DPv2,
// minimal -- and the three instances are the same debug port three times over.
// What tells them apart is TARGETID, which is the value TARGETSEL is written
// with, so it has already been asserted by the time DPIDR is read at all.
//
// The low 28 bits are fixed by that design. The top nibble is a revision, and
// a stepping that moved it would be rejected here: the error prints the value
// read, so the fix is to widen this.
const dpidr uint32 = 0x0bc12477

const (
	// bootromMagic is 'M', 'u', then a version byte this ignores, and is what
	// says the thing on the other end is an RP2040 at all.
	bootromMagic uint32 = 0x01754d
	// bootromMagicAddr is where it sits; the function table pointer is the
	// u16 four bytes later.
	bootromMagicAddr uint32 = 0x00000010

	// dhcsrDbgKey has to be in the top half of every DHCSR write or the write
	// is ignored.
	dhcsrDbgKey uint32 = 0xA05F << 16

	// CSW's access-size and auto-increment fields, which a batched transfer
	// list driving DRW itself has to set rather than inherit.
	cswSizeMask    uint32 = 0x7
	cswSizeWord    uint32 = 0x2
	cswAddrIncMask uint32 = 0x30
)
