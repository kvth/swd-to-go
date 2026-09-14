// Writing a whole image to flash, with the steps OpenOCD's `program` command
// has: preverify, verify, and reset when it is done.
//
// The reason those steps are worth having rather than "erase, write, hope" is
// that every one of them is a read the caller would otherwise have to do
// itself, and reads over a debug link are the expensive part. Doing them here
// means they can be shared: the read that decides whether the image is already
// in place is the same read that decides which sectors need erasing, and a run
// that finds the image already there does no writing at all.
package rp2040

import (
	"bytes"
	"context"
	"fmt"
	"time"
)

// EraseMode says which of the sectors an image covers get erased.
type EraseMode int

const (
	// EraseAuto erases only what needs it. A sector already holding the bytes
	// about to be written is left alone entirely; one that is already blank is
	// programmed without being erased first. This costs one read of the range
	// and is almost always the quickest, because a rebuilt image usually
	// differs from the one on the part in a small part of itself.
	EraseAuto EraseMode = iota
	// EraseAll erases every sector the image covers, whatever is in them, and
	// reads nothing first. The right choice when the read-back cannot be
	// trusted -- a part whose XIP window is not currently mapped, say.
	EraseAll
	// EraseNone programs without erasing. Flash programming can only clear
	// bits, so this writes correctly only into sectors that are already blank
	// or already hold a superset of the bits being written; anywhere else it
	// silently produces the AND of the two. It is for callers that have
	// erased the range themselves.
	EraseNone
)

func (m EraseMode) String() string {
	switch m {
	case EraseAuto:
		return "auto"
	case EraseAll:
		return "all"
	case EraseNone:
		return "none"
	}
	return fmt.Sprintf("EraseMode(%d)", int(m))
}

// Options control one [Flasher.FlashImage] run.
//
// The zero value writes the image at offset 0, erasing only the sectors that
// need it, and leaves the core halted afterwards — the same shape as OpenOCD's
// `program <file>`, to which PreVerify, Verify and Reset add its `preverify`,
// `verify` and `reset` keywords.
type Options struct {
	// Offset is where in flash the image goes, as a byte offset and not an
	// address: 0 is the start of flash, which the XIP window maps at
	// [FlashBase]. It must be a multiple of [FlashSector] unless Erase is
	// [EraseNone], because an erase clears whole sectors and a partial one
	// would take bytes the image does not cover with it.
	Offset uint32

	// Erase picks which sectors are cleared. See [EraseMode].
	Erase EraseMode

	// PreVerify reads the range back before writing anything and skips the
	// write entirely if the image is already there. This is OpenOCD's
	// `preverify`: for a target that is already up to date it turns a flash
	// cycle into one read.
	//
	// [EraseAuto] does the same read for its own purposes, so asking for both
	// costs nothing extra.
	PreVerify bool

	// Verify reads the range back afterwards and compares it. What it catches
	// that the write itself does not is a part that acknowledged a program it
	// did not perform -- a failing sector, or a range that was never erased.
	Verify bool

	// Reset resets the target when the write is done. Without Run the core is
	// caught at the reset vector and left halted, which is where a debugger
	// wants it; with Run it is let go, which is what a freshly flashed board
	// wants.
	Reset bool
	Run   bool

	// Pad is what the tail of the last page is filled with, since the ROM
	// programs whole 256-byte pages. Zero means 0xFF, which is what erased
	// flash reads as and so leaves the tail indistinguishable from unwritten.
	Pad byte

	// Timeout bounds each of the core operations -- the halt, and the reset if
	// one is asked for. Zero means two seconds. The ROM calls have their own
	// timeouts, sized to what the flash part needs.
	Timeout time.Duration

	// Progress is called through each phase: "preverify", "erase", "program"
	// and "verify". It may be nil.
	Progress Progress
}

// Result is what a [Flasher.FlashImage] run did. The byte counts are what
// crossed the link or changed on the part, so a run over an already-current
// target reports zeros for everything but Total and Skipped.
type Result struct {
	// Total is the image's length, after padding to a whole page.
	Total int
	// Sectors is how many flash sectors it covers.
	Sectors int
	// Skipped is how many of those already held the right bytes and were left
	// alone. Only [EraseAuto] and PreVerify can skip anything.
	Skipped int
	// Erased and Programmed are how many sectors were cleared and how many
	// were written. A sector that was already blank is programmed but not
	// erased, so these need not be equal.
	Erased     int
	Programmed int

	// UpToDate is set when the whole image was already in place and nothing
	// was written.
	UpToDate bool
	// Verified is set when the image was compared against the part and
	// matched -- by the verify pass, or by a preverify that found it already
	// there.
	Verified bool

	// CRC32 is the IEEE checksum of the padded image, for logging something
	// comparable against another tool's.
	CRC32 uint32

	// Elapsed is the wall-clock time the whole run took.
	Elapsed time.Duration
}

// String is a one-line summary, which is what most callers do with a Result.
func (r Result) String() string {
	if r.UpToDate {
		return fmt.Sprintf("%d bytes already in place (%d sectors, crc32 0x%08x) in %v",
			r.Total, r.Sectors, r.CRC32, r.Elapsed.Round(time.Millisecond))
	}
	s := fmt.Sprintf("%d bytes, %d/%d sectors written (%d erased, %d skipped, crc32 0x%08x) in %v",
		r.Total, r.Programmed, r.Sectors, r.Erased, r.Skipped, r.CRC32,
		r.Elapsed.Round(time.Millisecond))
	if r.Verified {
		s += ", verified"
	}
	return s
}

// sectorPlan is what a run decided to do with one sector.
type sectorPlan struct {
	erase   bool
	program bool
}

// FlashImage writes one image to flash and does everything around it: the
// halt, the bootrom lookup, the preverify, the erase, the write, the verify and
// the reset. It is the call a tool makes; the methods it is built from are
// there for anything that needs the steps apart.
//
// The order matters and is not negotiable. The read-backs happen while flash is
// memory mapped — before [Flasher.Prep] and after [Flasher.Finish] — because in
// between the QSPI interface is in direct command mode and the XIP window
// answers with nothing useful. Between Prep and Finish the target cannot
// execute from flash at all, which is why a run that dies in the middle leaves
// a board that will not boot until something flashes it again.
func (f *Flasher) FlashImage(ctx context.Context, image []byte, opts Options) (Result, error) {
	started := time.Now()

	timeout := opts.Timeout
	if timeout == 0 {
		timeout = 2 * time.Second
	}
	pad := opts.Pad
	if pad == 0 {
		pad = 0xFF
	}

	var res Result
	if len(image) == 0 {
		res.UpToDate = true
		res.Elapsed = time.Since(started)
		return res, nil
	}
	if opts.Erase != EraseNone && opts.Offset%FlashSector != 0 {
		return res, fmt.Errorf("flash offset 0x%x is not a multiple of the %d-byte sector, "+
			"and erasing it would take the rest of the sector with it",
			opts.Offset, FlashSector)
	}
	if opts.Offset%FlashPage != 0 {
		return res, fmt.Errorf("flash offset 0x%x is not a multiple of the %d-byte page",
			opts.Offset, FlashPage)
	}

	// The ROM programs whole pages, so the image is padded up to one. The copy
	// is deliberate: the caller's slice is not ours to extend or alter.
	padded := image
	if n := len(image) % int(FlashPage); n != 0 {
		padded = make([]byte, len(image)+int(FlashPage)-n)
		copy(padded, image)
		for i := len(image); i < len(padded); i++ {
			padded[i] = pad
		}
	}

	res.Total = len(padded)
	res.CRC32 = CRC32(padded)

	// Sectors are counted from the sector the image starts in, which is the
	// image's own start when the offset is aligned and everything but
	// EraseNone requires that it is.
	sectorStart := opts.Offset &^ (FlashSector - 1)
	sectorEnd := (opts.Offset + uint32(len(padded)) + FlashSector - 1) &^ (FlashSector - 1)
	res.Sectors = int((sectorEnd - sectorStart) / FlashSector)

	if err := f.Attach(ctx, timeout); err != nil {
		return res, err
	}

	// --- plan -----------------------------------------------------------
	//
	// Both the preverify answer and the per-sector erase decisions come out of
	// one read of the range, taken while flash is still mapped.

	plan := make([]sectorPlan, res.Sectors)
	needsRead := opts.PreVerify || opts.Erase == EraseAuto

	if needsRead {
		current, err := f.readForPlan(ctx, opts.Offset, len(padded), opts.Progress)
		if err != nil {
			return res, fmt.Errorf("preverify: %w", err)
		}
		if bytes.Equal(current, padded) {
			// The read that decides this is the same read a verify would do,
			// so there is nothing left to check afterwards.
			res.UpToDate, res.Verified = true, true
		}
		f.planSectors(plan, opts, padded, current)
	} else {
		for i := range plan {
			plan[i] = sectorPlan{erase: opts.Erase != EraseNone, program: true}
		}
	}

	for _, p := range plan {
		switch {
		case p.erase:
			res.Erased++
			res.Programmed++
		case p.program:
			res.Programmed++
		default:
			res.Skipped++
		}
	}

	// --- write ----------------------------------------------------------
	//
	// Skipped entirely when the plan came out empty. Prep takes flash out of
	// the memory map and Finish puts it back, so a pair of them around nothing
	// is not free: it is the window in which the target cannot boot.

	if res.Programmed != 0 || res.Erased != 0 {
		if err := f.Prep(ctx); err != nil {
			return res, fmt.Errorf("prepare flash: %w", err)
		}
		// Finish runs even when the erase or the write failed, because a
		// target that boots something stale beats one that boots nothing.
		writeErr := f.runPlan(ctx, plan, opts, padded, sectorStart)
		if err := f.Finish(ctx); err != nil && writeErr == nil {
			writeErr = fmt.Errorf("finish flash: %w", err)
		}
		if writeErr != nil {
			return res, writeErr
		}
	}

	// --- verify ---------------------------------------------------------

	if opts.Verify && !res.Verified {
		if err := f.Verify(ctx, opts.Offset, padded, opts.Progress); err != nil {
			return res, err
		}
		res.Verified = true
	}

	if err := f.finishRun(ctx, opts, timeout); err != nil {
		return res, err
	}
	res.Elapsed = time.Since(started)
	return res, nil
}

// readForPlan reads the range the image covers, reporting as the "preverify"
// phase.
func (f *Flasher) readForPlan(ctx context.Context, offset uint32, length int, report Progress) ([]byte, error) {
	if report == nil {
		return f.ReadFlash(ctx, offset, length)
	}
	out := make([]byte, 0, length)
	for done := 0; done < length; {
		n := readChunk
		if n > length-done {
			n = length - done
		}
		chunk, err := f.ReadFlash(ctx, offset+uint32(done), n)
		if err != nil {
			return nil, err
		}
		out = append(out, chunk...)
		done += n
		report("preverify", done, length)
	}
	return out, nil
}

// planSectors decides, sector by sector, what has to happen: nothing if the
// sector already holds the right bytes, a program on its own if it is blank,
// and an erase first otherwise.
//
// `current` is what the part holds over exactly the range `want` covers, so
// both are indexed from the image's start rather than the sector's. The first
// and last sector may extend past it at one end, which is why the slices are
// clamped.
func (f *Flasher) planSectors(plan []sectorPlan, opts Options, want, current []byte) {
	sectorStart := opts.Offset &^ (FlashSector - 1)

	for i := range plan {
		// Where this sector sits within the image, which for the first sector
		// starts partway in when the offset is not sector-aligned.
		lo := int(sectorStart+uint32(i)*FlashSector) - int(opts.Offset)
		hi := lo + int(FlashSector)
		if lo < 0 {
			lo = 0
		}
		if hi > len(want) {
			hi = len(want)
		}

		switch {
		case bytes.Equal(current[lo:hi], want[lo:hi]):
			// Already right: no erase, no write, nothing over the link.
		case opts.Erase == EraseNone:
			plan[i] = sectorPlan{program: true}
		case opts.Erase == EraseAll || !isBlank(current[lo:hi]):
			plan[i] = sectorPlan{erase: true, program: true}
		default:
			// Blank, so the erase would be a no-op the flash part would still
			// spend milliseconds on.
			plan[i] = sectorPlan{program: true}
		}
	}
}

// runPlan carries the plan out: every contiguous run of sectors that needs
// erasing is erased in one call, and every contiguous run that needs
// programming is written in one pass through the bounce buffer. Grouping is
// what keeps a sparse change from costing a ROM call per sector.
func (f *Flasher) runPlan(ctx context.Context, plan []sectorPlan, opts Options, padded []byte, sectorStart uint32) error {
	erased, toErase := 0, countIf(plan, func(p sectorPlan) bool { return p.erase })
	for _, run := range runsOf(plan, func(p sectorPlan) bool { return p.erase }) {
		start := sectorStart + uint32(run.lo)*FlashSector
		end := sectorStart + uint32(run.hi)*FlashSector
		if err := f.EraseRange(ctx, start, end, nil); err != nil {
			return err
		}
		erased += run.hi - run.lo
		if opts.Progress != nil {
			opts.Progress("erase", erased, toErase)
		}
	}

	programmed, toProgram := 0, countIf(plan, func(p sectorPlan) bool { return p.program })
	for _, run := range runsOf(plan, func(p sectorPlan) bool { return p.program }) {
		// Clamp to the image: the first and last sectors of the range can
		// reach past it when the image does not fill them.
		lo := int(sectorStart+uint32(run.lo)*FlashSector) - int(opts.Offset)
		hi := int(sectorStart+uint32(run.hi)*FlashSector) - int(opts.Offset)
		if lo < 0 {
			lo = 0
		}
		if hi > len(padded) {
			hi = len(padded)
		}
		if lo >= hi {
			continue
		}
		at := opts.Offset + uint32(lo)
		if err := f.Program(ctx, at, padded[lo:hi], nil); err != nil {
			return err
		}
		programmed += run.hi - run.lo
		if opts.Progress != nil {
			opts.Progress("program", programmed, toProgram)
		}
	}
	return nil
}

// finishRun is the reset, if one was asked for. It is its own step because a
// preverify that found nothing to do still has to honour it.
func (f *Flasher) finishRun(ctx context.Context, opts Options, timeout time.Duration) error {
	if !opts.Reset {
		return nil
	}
	if err := f.Reset(ctx, !opts.Run, timeout); err != nil {
		return fmt.Errorf("reset: %w", err)
	}
	return nil
}

// --- small helpers ----------------------------------------------------------

// isBlank reports whether a range reads as erased flash.
func isBlank(b []byte) bool {
	for _, v := range b {
		if v != 0xFF {
			return false
		}
	}
	return true
}

type run struct{ lo, hi int }

// runsOf groups the indices matching pred into maximal contiguous runs.
func runsOf(plan []sectorPlan, pred func(sectorPlan) bool) []run {
	var runs []run
	for i := 0; i < len(plan); i++ {
		if !pred(plan[i]) {
			continue
		}
		j := i
		for j < len(plan) && pred(plan[j]) {
			j++
		}
		runs = append(runs, run{lo: i, hi: j})
		i = j
	}
	return runs
}

func countIf(plan []sectorPlan, pred func(sectorPlan) bool) int {
	n := 0
	for _, p := range plan {
		if pred(p) {
			n++
		}
	}
	return n
}
