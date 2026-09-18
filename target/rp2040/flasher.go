// RP2040 bootrom flashing driven by the host, over ordinary CMSIS-DAP
// transfers, on top of this module's DAP/DP/MEM-AP client.
//
// The sequence is the same one the probe-side vendor commands run (rp2040.h
// in github.com/kvth/cmsis-dap-tcp-gateway-rpi), and the one the bootrom's ABI
// dictates: find the bootrom function table, then for each routine set
// r0..r3 and r7 and run the
// bootrom's debug_trampoline, which is `blx r7` followed by a breakpoint.
// The difference is where it runs, and that is the whole point of having both.
//
// This is meant to be the fast version of doing it from here, not a straw man.
// Every trick that does not change what reaches the target is used:
//
//   - A whole ROM call's register setup goes out as ONE DAP_Transfer packet.
//     Naively each core register is a DCRDR write and a DCRSR write, each of
//     those a TAR write and a DRW write, each of those its own packet: 28
//     packets for the seven registers a call needs. Batched, it is one, with
//     the resume and the first halt poll appended to it.
//   - Bulk memory moves use block transfers, one per 1 kB auto-increment
//     region, which is what the MEM-AP's TAR wrap allows.
//   - Flash is prepared once and finished once for the whole image, not per
//     page, and programmed out of a large bounce buffer.
//
// What is left is inherent to driving it from here: the bounce buffer costs a
// TAR write packet per 1 kB on top of each block write, and verifying means
// reading the image back over the link rather than asking the probe for a CRC.
package rp2040

import (
	"context"
	"encoding/binary"
	"fmt"
	"hash/crc32"
	"time"

	"github.com/kvth/swd-to-go"
)

// Progress is called after each step of a phase with how far through it is.
// It may be nil.
type Progress func(phase string, done, total int)

const (
	// Flash geometry. FlashBase is where the XIP window puts flash in the
	// target's address map; every offset below is relative to the start of
	// flash, not to it.
	FlashBase   uint32 = 0x10000000
	FlashPage   uint32 = 256
	FlashSector uint32 = 4096

	eraseBlockSize uint32 = 64 * 1024
	eraseBlockCmd  uint32 = 0xD8

	// The bootrom magic and its function table pointer are bootromMagic and
	// bootromMagicAddr, shared with rp2040.go.

	// MEM-AP registers, as the DP client numbers them.
	regCSW uint8 = 0x00
	regTAR uint8 = 0x04
	regDRW uint8 = 0x0C

	// ARMv6-M debug registers.
	addrAIRCR uint32 = 0xE000ED0C
	addrDHCSR uint32 = 0xE000EDF0
	addrDCRSR uint32 = 0xE000EDF4
	addrDCRDR uint32 = 0xE000EDF8
	addrDEMCR uint32 = 0xE000EDFC

	dhcsrCDebugEn  uint32 = 1 << 0
	dhcsrCHalt     uint32 = 1 << 1
	dhcsrCMaskInts uint32 = 1 << 3
	dhcsrSHalt     uint32 = 1 << 17
	dhcsrSLockup   uint32 = 1 << 19
	dhcsrSResetSt  uint32 = 1 << 25

	dcrsrWrite uint32 = 1 << 16

	demcrVCCoreReset uint32 = 1 << 0
	aircrVectKey     uint32 = 0x05FA << 16
	aircrSysResetReq uint32 = 1 << 2

	xpsrThumb uint32 = 1 << 24

	coreR0   uint32 = 0
	coreR7   uint32 = 7
	coreSP   uint32 = 13
	coreLR   uint32 = 14
	corePC   uint32 = 15
	coreXPSR uint32 = 16
	coreMSP  uint32 = 17

	// Defaults matching the probe-side commands, so both tools put the ROM
	// stack and the bounce buffer in the same place.
	defaultStackTop   uint32 = 0x20042000
	defaultStagingAdr uint32 = 0x20020000
	defaultStagingLen uint32 = 64 * 1024
)

// jumpIndex names the bootrom entry points, in the order they are looked up.
type jumpIndex int

const (
	jTrampoline jumpIndex = iota
	jTrampolineEnd
	jConnectInternalFlash
	jFlashExitXIP
	jFlashRangeErase
	jFlashRangeProgram
	jFlashFlushCache
	jFlashEnterCmdXIP
	jumpCount
)

var jumpTags = [jumpCount]uint16{
	tag('D', 'T'), tag('D', 'E'), tag('I', 'F'), tag('E', 'X'),
	tag('R', 'E'), tag('R', 'P'), tag('F', 'C'), tag('C', 'X'),
}

var jumpNames = [jumpCount]string{
	"debug_trampoline", "debug_trampoline_end", "connect_internal_flash",
	"flash_exit_xip", "flash_range_erase", "flash_range_program",
	"flash_flush_cache", "flash_enter_cmd_xip",
}

func tag(a, b byte) uint16 { return uint16(b)<<8 | uint16(a) }

// DebugPort is the part of a debug port a [Flasher] needs. target/dp's Client
// is one.
//
// Only the sticky-error clear: everything else a flashing run does to the DP
// it does through [Memory], or straight through the [swd.Probe] as a batched
// transfer list.
type DebugPort interface {
	// ClearErrors clears the DP's sticky error bits, which a reset leaves set
	// because transfers fault while the target is held in it.
	ClearErrors(ctx context.Context) error
}

// Memory is the part of a MEM-AP client a [Flasher] needs: target memory at
// word and byte granularity, single registers for the core's debug registers,
// and the invalidate that tells it the batched transfers below have moved CSW
// and TAR behind its back. target/mem's Client is one.
//
// It is declared here so that flashing depends on a way of reaching target
// memory rather than on target/mem's implementation of one.
type Memory interface {
	ReadTargetReg(ctx context.Context, addr uint32) (uint32, error)
	WriteTargetReg(ctx context.Context, addr uint32, value uint32) error
	ReadTargetMem(ctx context.Context, addr uint32, length int) ([]uint32, error)
	ReadTargetMemBytes(ctx context.Context, addr uint32, length int) ([]byte, error)
	WriteTargetMemBytes(ctx context.Context, addr uint32, data []byte) error

	// Invalidate forgets the client's shadow of CSW and TAR. A [Flasher]
	// batches its own AP transfers straight to the probe, so it has to say so.
	Invalidate()
}

// Flasher drives one RP2040's bootrom flash routines through one MEM-AP.
//
// It is the fast way to do this from the host: a whole ROM call's register
// setup goes out as one transfer list, bulk moves use block transfers,
// and flash is prepared once and finished once around a large bounce buffer.
type Flasher struct {
	probe swd.Probe
	dpc   DebugPort
	mem   Memory
	ap    uint8

	jump       [jumpCount]uint16
	stackTop   uint32
	staging    uint32
	stagingLen uint32

	maxBatch int // transfers that fit in one list

	// csw is the AP's CSW as a batch below needs it -- word-sized, no
	// auto-increment -- and cswSet is whether it is known to still be there.
	// The debug registers are on the PPB, which is word-only, so a batch that
	// inherited a byte-sized CSW from the MEM-AP client would read zeros and
	// write nothing at all.
	csw    uint32
	cswSet bool
}

func NewFlasher(probe swd.Probe, dpc DebugPort, m Memory, ap uint8,
	stackTop, staging, stagingLen uint32) *Flasher {
	if stackTop == 0 {
		stackTop = defaultStackTop
	}
	if staging == 0 {
		staging = defaultStagingAdr
	}
	if stagingLen == 0 {
		stagingLen = defaultStagingLen
	}
	// A write transfer is a request byte and four data bytes; the header is a
	// few more. Leave generous room rather than tracking the exact framing.
	maxBatch := (probe.MaxBlockSize()*4 - 16) / 5
	if maxBatch < 8 {
		maxBatch = 8
	}
	return &Flasher{
		probe: probe, dpc: dpc, mem: m, ap: ap,
		stackTop: stackTop, staging: staging, stagingLen: stagingLen,
		maxBatch: maxBatch,
	}
}

// --- batched AP access ------------------------------------------------------

// apBatch runs a prepared list of AP transfers as one packet and returns the
// values of the reads in it.
//
// These go straight to the probe rather than through the MEM-AP client, which
// is the whole point -- a client call per transfer is a packet per transfer.
// The cost is that the client's idea of where TAR points is now wrong, so it
// is told to forget it; that is bookkeeping, not traffic.
func (f *Flasher) apBatch(ctx context.Context, reqs []swd.Request) ([]uint32, error) {
	if len(reqs) == 0 {
		return nil, nil
	}
	// These transfers drive DRW directly, so the access size in CSW is theirs
	// to establish: the MEM-AP client moves it as it likes -- a byte-granular
	// read leaves it byte-sized -- and the debug registers this batch reaches
	// are on the PPB, where anything narrower than a word reads as zero and is
	// written nowhere. One transfer, and only when something may have moved it.
	if !f.cswSet {
		csw, err := f.wordCSW(ctx)
		if err != nil {
			return nil, err
		}
		reqs = append([]swd.Request{{Op: swd.OpWrite, AP: true, Reg: regCSW, Data: csw}}, reqs...)
	}
	status, data, err := f.probe.Transfer(ctx, reqs)
	f.mem.Invalidate()
	if err != nil {
		f.cswSet = false
		return data, err
	}
	if !status.Ok() {
		f.cswSet = false
		return data, fmt.Errorf("batch of %d transfers stopped: %v", len(reqs), status)
	}
	f.cswSet = true
	return data, nil
}

// wordCSW is the CSW this Flasher needs: whatever the AP came up with, with
// the size forced to a word and auto-increment off. It is read from the AP
// once rather than assumed, so the AP's own protection bits are kept.
func (f *Flasher) wordCSW(ctx context.Context) (uint32, error) {
	if f.csw != 0 {
		return f.csw, nil
	}
	_, v, err := f.probe.Transfer(ctx, []swd.Request{{Op: swd.OpRead, AP: true, Reg: regCSW}})
	f.mem.Invalidate()
	if err != nil {
		return 0, fmt.Errorf("read CSW: %w", err)
	}
	if len(v) == 0 {
		return 0, fmt.Errorf("read CSW: no value")
	}
	f.csw = (v[0] &^ (cswSizeMask | cswAddrIncMask)) | cswSizeWord
	return f.csw, nil
}

// memMoved records that the MEM-AP client has driven CSW itself, so the next
// batch has to put back the size it needs.
func (f *Flasher) memMoved() { f.cswSet = false }

// wr appends "write `value` to target address `addr`" to a batch.
func wr(reqs []swd.Request, addr, value uint32) []swd.Request {
	return append(reqs,
		swd.Request{Op: swd.OpWrite, AP: true, Reg: regTAR, Data: addr},
		swd.Request{Op: swd.OpWrite, AP: true, Reg: regDRW, Data: value})
}

// rd appends "read target address `addr`" to a batch. Its value comes back in
// the batch's result slice, in order.
func rd(reqs []swd.Request, addr uint32) []swd.Request {
	return append(reqs,
		swd.Request{Op: swd.OpWrite, AP: true, Reg: regTAR, Data: addr},
		swd.Request{Op: swd.OpRead, AP: true, Reg: regDRW})
}

// regWrite appends a core register write: the value into DCRDR, then the
// register number into DCRSR, which is what starts the transfer.
func regWrite(reqs []swd.Request, reg, value uint32) []swd.Request {
	reqs = wr(reqs, addrDCRDR, value)
	return wr(reqs, addrDCRSR, dcrsrWrite|(reg&0x1F))
}

func (f *Flasher) readDHCSR(ctx context.Context) (uint32, error) {
	v, err := f.apBatch(ctx, rd(nil, addrDHCSR))
	if err != nil {
		return 0, err
	}
	return v[0], nil
}

func (f *Flasher) writeDHCSR(ctx context.Context, bits uint32) error {
	_, err := f.apBatch(ctx, wr(nil, addrDHCSR, dhcsrDbgKey|bits))
	return err
}

func (f *Flasher) readCoreReg(ctx context.Context, reg uint32) (uint32, error) {
	reqs := wr(nil, addrDCRSR, reg&0x1F)
	reqs = rd(reqs, addrDCRDR)
	v, err := f.apBatch(ctx, reqs)
	if err != nil {
		return 0, err
	}
	return v[0], nil
}

// --- core control -----------------------------------------------------------

func (f *Flasher) waitHalted(ctx context.Context, timeout time.Duration) (uint32, error) {
	deadline := time.Now().Add(timeout)
	for {
		v, err := f.readDHCSR(ctx)
		if err != nil {
			return 0, err
		}
		if v&dhcsrSHalt != 0 {
			return v, nil
		}
		if time.Now().After(deadline) {
			return v, fmt.Errorf("the core did not halt (dhcsr 0x%08x)", v)
		}
	}
}

func (f *Flasher) Halt(ctx context.Context, timeout time.Duration) error {
	if err := f.writeDHCSR(ctx, dhcsrCDebugEn|dhcsrCHalt); err != nil {
		return err
	}
	if _, err := f.waitHalted(ctx, timeout); err != nil {
		return err
	}
	// C_MASKINTS may only be changed while halted, so it goes in separately.
	return f.writeDHCSR(ctx, dhcsrCDebugEn|dhcsrCHalt|dhcsrCMaskInts)
}

func (f *Flasher) Resume(ctx context.Context) error {
	return f.writeDHCSR(ctx, dhcsrCDebugEn)
}

// Reset drives AIRCR.SYSRESETREQ, optionally catching the core at the reset
// vector with DEMCR.VC_CORERESET.
//
// Transfers fail while the target is in reset, so faults are tolerated for as
// long as the budget lasts. S_RESET_ST -- sticky, and cleared by reading it --
// has to be seen before this calls it done, or "the core is halted" would be
// satisfied by the core we halted ourselves on the way in.
func (f *Flasher) Reset(ctx context.Context, haltAfter bool, timeout time.Duration) error {
	bits := dhcsrCDebugEn
	if haltAfter {
		bits |= dhcsrCHalt
	}
	_ = f.writeDHCSR(ctx, bits)

	f.memMoved()
	demcr, err := f.mem.ReadTargetReg(ctx, addrDEMCR)
	if err != nil {
		return fmt.Errorf("read DEMCR: %w", err)
	}
	armed := demcr &^ demcrVCCoreReset
	if haltAfter {
		armed = demcr | demcrVCCoreReset
	}
	if err := f.mem.WriteTargetReg(ctx, addrDEMCR, armed); err != nil {
		return fmt.Errorf("write DEMCR: %w", err)
	}

	// The reset can land before this write is acknowledged, so a fault here
	// says nothing about whether it worked.
	_ = f.mem.WriteTargetReg(ctx, addrAIRCR, aircrVectKey|aircrSysResetReq)

	deadline := time.Now().Add(timeout)
	sawReset, done := false, false
	for !done {
		v, err := f.readDHCSR(ctx)
		switch {
		case err != nil:
			_ = f.dpc.ClearErrors(ctx) // a fault in reset is expected; clear it
		case v&dhcsrSResetSt != 0:
			sawReset = true
		case sawReset:
			done = !haltAfter || v&dhcsrSHalt != 0
		}
		if !done && time.Now().After(deadline) {
			if !sawReset {
				return fmt.Errorf("reset was not observed")
			}
			return fmt.Errorf("the target stayed in reset")
		}
	}

	// Disarm the vector catch, or every later reset would stop here too.
	_ = f.mem.WriteTargetReg(ctx, addrDEMCR, demcr&^demcrVCCoreReset)
	if haltAfter {
		return f.writeDHCSR(ctx, dhcsrCDebugEn|dhcsrCHalt|dhcsrCMaskInts)
	}
	return f.writeDHCSR(ctx, dhcsrCDebugEn)
}

// --- bootrom ----------------------------------------------------------------

// LoadJumpTable checks the bootrom magic and reads every entry point. The
// table is pulled over in one block read and walked here rather than chased
// two bytes at a time, which would be two packets per entry visited.
func (f *Flasher) LoadJumpTable(ctx context.Context) error {
	f.memMoved()
	head, err := f.mem.ReadTargetMem(ctx, bootromMagicAddr, 2)
	if err != nil {
		return fmt.Errorf("read bootrom magic: %w", err)
	}
	if head[0]&0x00FFFFFF != bootromMagic {
		return fmt.Errorf("no RP2040 bootrom at 0x%08x (read 0x%08x)", bootromMagicAddr, head[0])
	}
	table := uint32(uint16(head[1]))

	// The whole table comfortably fits; 512 bytes is 128 entries.
	f.memMoved()
	raw, err := f.mem.ReadTargetMemBytes(ctx, table, 512)
	if err != nil {
		return fmt.Errorf("read bootrom function table: %w", err)
	}

	found := 0
	for off := 0; off+4 <= len(raw); off += 4 {
		entryTag := binary.LittleEndian.Uint16(raw[off:])
		if entryTag == 0 {
			break
		}
		for i := jumpIndex(0); i < jumpCount; i++ {
			if entryTag == jumpTags[i] {
				f.jump[i] = binary.LittleEndian.Uint16(raw[off+2:])
				found++
			}
		}
	}
	for i := jumpIndex(0); i < jumpCount; i++ {
		if f.jump[i] == 0 {
			return fmt.Errorf("bootrom has no %s", jumpNames[i])
		}
	}
	// Branch targets for us, not for a `bx`, so the Thumb bit comes off. The
	// routine addresses keep theirs.
	f.jump[jTrampoline] &^= 1
	f.jump[jTrampolineEnd] &^= 1
	return nil
}

func (f *Flasher) Trampoline() uint16 { return f.jump[jTrampoline] }

// --- calling a bootrom routine ----------------------------------------------

// romCall runs one bootrom entry point through the debug trampoline.
//
// The register setup, the resume and the first halt poll all go out in one
// packet: everything up to the resume is a write with no result to wait for,
// and a call that finishes inside one round trip then needs no second one.
func (f *Flasher) romCall(ctx context.Context, which jumpIndex, args []uint32, timeout time.Duration) error {
	fn := uint32(f.jump[which])
	tramp := uint32(f.jump[jTrampoline])
	trampEnd := uint32(f.jump[jTrampolineEnd])

	var reqs []swd.Request
	for i, a := range args {
		reqs = regWrite(reqs, coreR0+uint32(i), a)
	}
	reqs = regWrite(reqs, coreR7, fn)
	// Both stack views get the same value, so it does not matter which one the
	// core is currently using.
	reqs = regWrite(reqs, coreSP, f.stackTop)
	reqs = regWrite(reqs, coreMSP, f.stackTop)
	// Only the Thumb bit matters here: anything left over from the application
	// would make the return from the routine mean something else.
	reqs = regWrite(reqs, coreXPSR, xpsrThumb)
	// `blx` overwrites LR before the routine sees it; a backstop only.
	reqs = regWrite(reqs, coreLR, trampEnd|1)
	reqs = regWrite(reqs, corePC, tramp)
	reqs = wr(reqs, addrDHCSR, dhcsrDbgKey|dhcsrCDebugEn|dhcsrCMaskInts) // run
	reqs = rd(reqs, addrDHCSR)

	if len(reqs) > f.maxBatch {
		return fmt.Errorf("rom call setup needs %d transfers, packet holds %d", len(reqs), f.maxBatch)
	}
	res, err := f.apBatch(ctx, reqs)
	if err != nil {
		return fmt.Errorf("call %s: %w", jumpNames[which], err)
	}

	dhcsr := res[0]
	if dhcsr&dhcsrSHalt == 0 {
		dhcsr, err = f.waitHalted(ctx, timeout)
		if err != nil {
			_ = f.writeDHCSR(ctx, dhcsrCDebugEn|dhcsrCHalt|dhcsrCMaskInts)
			return fmt.Errorf("call %s: %w", jumpNames[which], err)
		}
	}
	// Re-assert C_HALT: the breakpoint stopped the core, but DHCSR still says
	// "running" until we say otherwise, and a later resume would not take.
	if err := f.writeDHCSR(ctx, dhcsrCDebugEn|dhcsrCHalt|dhcsrCMaskInts); err != nil {
		return err
	}

	// It has to have stopped on the trampoline's own breakpoint. Anywhere else
	// means the routine did not run to completion.
	pc, err := f.readCoreReg(ctx, corePC)
	if err != nil {
		return err
	}
	if pc&^1 != trampEnd || dhcsr&dhcsrSLockup != 0 {
		return fmt.Errorf("call %s went astray: pc 0x%08x, expected 0x%08x (dhcsr 0x%08x)",
			jumpNames[which], pc, trampEnd, dhcsr)
	}
	return nil
}

// Attach gets a flashing session to the point where the ROM routines can be
// called: the core stopped, and the bootrom's function table read.
func (f *Flasher) Attach(ctx context.Context, timeout time.Duration) error {
	if err := f.Halt(ctx, timeout); err != nil {
		return fmt.Errorf("halt the core: %w", err)
	}
	if err := f.LoadJumpTable(ctx); err != nil {
		return fmt.Errorf("read the bootrom function table: %w", err)
	}
	return nil
}

// --- the flash sequence -----------------------------------------------------

// Prep takes the QSPI interface out of memory-mapped mode. Between this and
// Finish the target cannot execute from flash.
func (f *Flasher) Prep(ctx context.Context) error {
	if err := f.romCall(ctx, jConnectInternalFlash, nil, time.Second); err != nil {
		return err
	}
	return f.romCall(ctx, jFlashExitXIP, nil, time.Second)
}

// Finish flushes the cache and puts flash back in the memory map, which is
// what makes the target bootable again. Both calls are attempted even if the
// first fails: a target that boots from a stale cache beats one that does not
// boot at all.
func (f *Flasher) Finish(ctx context.Context) error {
	flushErr := f.romCall(ctx, jFlashFlushCache, nil, time.Second)
	xipErr := f.romCall(ctx, jFlashEnterCmdXIP, nil, time.Second)
	if flushErr != nil {
		return flushErr
	}
	return xipErr
}

// Erase clears the whole sectors an image at `offset` will occupy. Note the
// rounding: a partial sector at either end is erased in full, taking whatever
// else was in it.
func (f *Flasher) Erase(ctx context.Context, offset uint32, length int, report Progress) error {
	if length == 0 {
		return nil
	}
	start := offset &^ (FlashSector - 1)
	end := (offset + uint32(length) + FlashSector - 1) &^ (FlashSector - 1)
	return f.EraseRange(ctx, start, end, report)
}

// EraseRange clears [start, end), both of which must be sector-aligned.
//
// The range goes out in erase-block-sized calls so the ROM can use the part's
// block erase where a whole block is covered, which is an order of magnitude
// quicker than clearing it a sector at a time.
func (f *Flasher) EraseRange(ctx context.Context, start, end uint32, report Progress) error {
	if start%FlashSector != 0 || end%FlashSector != 0 {
		return fmt.Errorf("erase range 0x%x..0x%x is not sector-aligned", start, end)
	}
	if end <= start {
		return nil
	}
	for a := start; a < end; {
		n := end - a
		if n > eraseBlockSize {
			n = eraseBlockSize
		}
		// Keep the calls on erase-block boundaries, or a range that straddles
		// one denies the ROM the block erase for both halves.
		if next := (a + eraseBlockSize) &^ (eraseBlockSize - 1); a+n > next {
			n = next - a
		}
		args := []uint32{a, n, eraseBlockSize, eraseBlockCmd}
		if err := f.romCall(ctx, jFlashRangeErase, args, 8*time.Second); err != nil {
			return fmt.Errorf("erase 0x%x+%d: %w", a, n, err)
		}
		a += n
		if report != nil {
			report("erase", int(a-start), int(end-start))
		}
	}
	return nil
}

// Program writes one contiguous image at a flash offset, over sectors Erase
// has already cleared. `data` is already padded to a page.
//
// Same shape as the probe-side version: fill the bounce buffer, then one ROM
// call for the whole of it, rather than a call per page.
func (f *Flasher) Program(ctx context.Context, offset uint32, data []byte, report Progress) error {
	if offset%FlashPage != 0 {
		return fmt.Errorf("flash offset 0x%x is not a multiple of %d", offset, FlashPage)
	}
	if uint32(len(data))%FlashPage != 0 {
		return fmt.Errorf("image length %d is not a multiple of %d", len(data), FlashPage)
	}
	chunk := int(f.stagingLen)
	for done := 0; done < len(data); {
		n := chunk
		if n > len(data)-done {
			n = len(data) - done
		}
		f.memMoved()
		if err := f.mem.WriteTargetMemBytes(ctx, f.staging, data[done:done+n]); err != nil {
			return fmt.Errorf("fill bounce buffer for 0x%x: %w", offset+uint32(done), err)
		}
		args := []uint32{offset + uint32(done), f.staging, uint32(n)}
		if err := f.romCall(ctx, jFlashRangeProgram, args, 10*time.Second); err != nil {
			return fmt.Errorf("program 0x%x+%d: %w", offset+uint32(done), n, err)
		}
		done += n
		if report != nil {
			report("program", done, len(data))
		}
	}
	return nil
}

// readChunk is how much flash one read-back asks for at a time. Bigger than
// the MEM-AP's 1 kB auto-increment window by a good margin, so the cost is
// block transfers rather than TAR reloads, and small enough that progress
// moves.
const readChunk = 16 * 1024

// ReadFlash reads back through the XIP window, which only answers once flash
// is memory mapped -- so before [Flasher.Prep] or after [Flasher.Finish], not
// between them.
func (f *Flasher) ReadFlash(ctx context.Context, offset uint32, length int) ([]byte, error) {
	out := make([]byte, length)
	for done := 0; done < length; {
		n := readChunk
		if n > length-done {
			n = length - done
		}
		f.memMoved()
		got, err := f.mem.ReadTargetMemBytes(ctx, FlashBase+offset+uint32(done), n)
		if err != nil {
			return nil, fmt.Errorf("read back 0x%x+%d: %w", offset+uint32(done), n, err)
		}
		copy(out[done:], got)
		done += n
	}
	return out, nil
}

// Verify reads the image back through the XIP window and compares it. There is
// no probe-side checksum to ask for here, so the whole image crosses the link.
func (f *Flasher) Verify(ctx context.Context, offset uint32, data []byte, report Progress) error {
	for done := 0; done < len(data); {
		n := readChunk
		if n > len(data)-done {
			n = len(data) - done
		}
		f.memMoved()
		got, err := f.mem.ReadTargetMemBytes(ctx, FlashBase+offset+uint32(done), n)
		if err != nil {
			return fmt.Errorf("read back 0x%x+%d: %w", offset+uint32(done), n, err)
		}
		want := data[done : done+n]
		if i := mismatch(got, want); i >= 0 {
			return fmt.Errorf("verify failed at 0x%08x: flash 0x%02x, image 0x%02x",
				FlashBase+offset+uint32(done+i), got[i], want[i])
		}
		done += n
		if report != nil {
			report("verify", done, len(data))
		}
	}
	return nil
}

// mismatch is the index of the first byte that differs, or -1.
func mismatch(got, want []byte) int {
	for i := range want {
		if got[i] != want[i] {
			return i
		}
	}
	return -1
}

// CRC32 is only used to print something comparable to the probe-side tool's
// checksum; the comparison itself is done byte for byte above.
func CRC32(data []byte) uint32 { return crc32.ChecksumIEEE(data) }
