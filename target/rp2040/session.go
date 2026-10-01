package rp2040

import (
	"context"
	"fmt"
	"time"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
)

// sessionTimeout bounds halting and resetting the core in a [Session].
const sessionTimeout = 2 * time.Second

// xipWindowEnd is the end of the four XIP aliases of flash, cached and not,
// that start at [FlashBase]. Every read below it goes through the SSI.
const xipWindowEnd uint32 = 0x14000000

// Session is one RP2040 core brought up as far as its MEM-AP -- the state
// OpenOCD is in after `init` -- with the commands an OpenOCD command line runs
// against it:
//
//	program <file> <addr> preverify verify reset    Session.Program
//	dump_image <file> <addr> <len>                  Session.ReadMemory
//	reset / reset run, reset halt                   Session.Reset
//	-c "set RESCUE 1"                               Rescue, before Open
//
// It is for the one-off command, not for polling: a caller asking a muxed bus
// which positions are populated wants [Attach] and nothing after it, and
// paying for a debug-port power-up and a MEM-AP probe per poll is what this
// would add.
//
// A Session holds the debug port and MEM-AP shadows of the one target it was
// opened on, so it is used for that target and then closed.
type Session struct {
	flasher *Flasher
	detach  func(context.Context) error
}

// Open attaches to one core ([Attach]) and brings its debug port and MEM-AP
// up. It sets no clock -- do that on the probe first -- and does not halt the
// core.
//
// To get in to a board whose firmware locks the debug port out, [Rescue] the
// chip first and Open it afterwards, the way OpenOCD's RESCUE run is followed
// by the run that does the work.
func Open(ctx context.Context, probe swd.Probe, core uint32) (*Session, error) {
	detach, err := Attach(ctx, probe, core)
	if err != nil {
		return nil, err
	}

	dpc := dp.New(probe)
	if err := dpc.Init(ctx); err != nil {
		_ = detach(context.WithoutCancel(ctx))
		return nil, fmt.Errorf("init DP: %w", err)
	}
	apc := mem.New(dpc, 0)
	if err := apc.Init(ctx); err != nil {
		_ = detach(context.WithoutCancel(ctx))
		return nil, fmt.Errorf("init MEM-AP: %w", err)
	}

	// AP 0, and the zeros are the default ROM stack and bounce buffer.
	return &Session{
		flasher: NewFlasher(probe, dpc, apc, 0, 0, 0, 0),
		detach:  detach,
	}, nil
}

// Close puts the wire back in the dormant state. See [Attach].
func (s *Session) Close(ctx context.Context) error {
	return s.detach(ctx)
}

// Program writes an image to flash: OpenOCD's `program`, with the address
// given as opts.Offset, an offset from [FlashBase]. It is [Flasher.FlashImage].
//
// `program <file> <addr> preverify verify reset` is
//
//	Options{Offset: addr - FlashBase, PreVerify: true, Verify: true, Reset: true, Run: true}
func (s *Session) Program(ctx context.Context, image []byte, opts Options) (Result, error) {
	return s.flasher.FlashImage(ctx, image, opts)
}

// ReadMemory reads target memory: OpenOCD's `dump_image`. Like it, it does not
// halt the core.
//
// A range reaching into the XIP window has flash mapped into it first if it
// is not already, so that flash reads as flash on a chip that has not booted
// from it -- which halts the core in that case. A range below or above the
// window never touches the SSI, so SRAM a crash left behind reads as it was
// left.
func (s *Session) ReadMemory(ctx context.Context, addr uint32, length int) ([]byte, error) {
	if length < 0 || uint64(addr)+uint64(length) > 1<<32 {
		return nil, fmt.Errorf("read 0x%08x+%d runs past the end of the address space", addr, length)
	}
	if length > 0 && addr < xipWindowEnd && uint64(addr)+uint64(length) > uint64(FlashBase) {
		if err := s.flasher.ensureXIP(ctx, sessionTimeout); err != nil {
			return nil, err
		}
	}
	s.flasher.memMoved()
	return s.flasher.mem.ReadTargetMemBytes(ctx, addr, length)
}

// Reset resets the core through AIRCR.SYSRESETREQ, as OpenOCD's RP2040 target
// does: `reset run` with halt false, `reset halt` with halt true, which catches
// the core at the reset vector. It is [Flasher.Reset].
func (s *Session) Reset(ctx context.Context, halt bool) error {
	return s.flasher.Reset(ctx, halt, sessionTimeout)
}
