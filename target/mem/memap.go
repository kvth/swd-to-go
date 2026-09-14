// Package mem is target memory through a MEM-AP.
//
// A MEM-AP access is three registers: CSW says how wide the access is and
// whether the address advances, TAR is the address, and every read or write of
// DRW performs one access at it. The expensive part is that each of those is a
// transfer on the wire, and over a packet transport a transfer is a round trip.
//
// So the client here remembers what it last put in CSW and TAR and skips any
// write that would put the same value there again. That is worth more than it
// looks: polling one register — waiting for a core to halt, say — costs one
// transfer per poll instead of two, and a run of block transfers reloads TAR
// only at the 1 kB boundaries where the AP's auto-increment wraps.
//
// The shadow assumes this client is the only thing driving its AP. Anything
// else that writes CSW or TAR behind its back — a caller batching raw
// transfers of its own, as target/rp2040's Flasher does — has to say so with
// [Client.Invalidate], or the next access here will skip a TAR write it
// needed and land at the wrong address.
//
// # One client per target
//
// The shadow is one target's state, so a wire multiplexed between several
// targets wants one [Client] per target. Three things make sharing one worse
// here than it is a layer down:
//
//   - CSW and TAR describe the AP of whichever target was last on the wire.
//     After a switch they describe a chip that is not listening, and the next
//     access skips a TAR write it needed and lands at the wrong address --
//     quietly, with a plausible value.
//   - Whether the AP does byte-sized accesses is a property of that AP, found
//     out by writing CSW and reading it back. A shared client re-probes on
//     every switch; a per-target one finds out once, ever.
//   - A shared client has to be re-initialised on every switch, which throws
//     away the saved writes the shadow exists for. Per-target clients keep
//     theirs, and a target revisited a moment later is still described by the
//     shadow it left behind.
//
// A [Client] is an allocation and no I/O, so a dozen of them cost nothing.
// [Client.Invalidate] is there for a caller who shares one anyway, and for the
// legitimate case it was written for: something else writing CSW or TAR behind
// this client's back on the same target.
package mem

//
// This file is derived from the Mongoose OS mos tool
// (github.com/mongoose-os/mos,
// cli/flash/common/cmsis-dap/memap/cmsis_dap_memap.go) and has been modified:
// the CSW/TAR shadow, byte-sized access and its word-only fallback, and the
// log/slog tracing are new; the register definitions, the initialisation
// check, the auto-increment handling and the exported API shape come from the
// original. See the NOTICE file.
//
// Copyright (c) 2014-2019 Cesanta Software Limited
// All rights reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

import (
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"log/slog"

	"github.com/kvth/swd-to-go"
)

type MemAPReg uint8

const (
	CSW  MemAPReg = 0x00
	TAR           = 0x04
	DRW           = 0x0c
	BD0           = 0x10
	BD1           = 0x14
	BD2           = 0x18
	BD3           = 0x1c
	BASE          = 0xf8
)

// CSW fields this client drives. Everything else in the register is left as
// [cswBase] has it.
const (
	cswSizeMask = 0x7
	cswSizeByte = 0x0
	cswSizeWord = 0x2

	cswAddrIncMask   = 0x3 << 4
	cswAddrIncOff    = 0x0 << 4
	cswAddrIncSingle = 0x1 << 4

	CSW_DeviceEn = 0x40

	// cswBase is the rest of CSW: device enabled, and the bus attributes
	// (HPROT, MasterType) this code has always asked for. Size and
	// auto-increment are OR'd in per access.
	cswBase uint32 = 0x23000040

	// tarAutoIncRegion is how far TAR advances on its own before wrapping
	// rather than carrying. Every run of DRW accesses stops at one of these.
	tarAutoIncRegion uint32 = 1024
)

// errNoByteAccess is internal: it says the AP cannot do byte-sized accesses, so
// a partial word has to be read-modify-written instead.
var errNoByteAccess = errors.New("mem: the MEM-AP does not support byte accesses")

// DebugPort is the part of a debug port a MEM-AP client needs: access-port
// registers, and the sticky-error clear that has to happen before any of them
// will answer again after a fault. target/dp's Client is one.
//
// It is declared here rather than in target/dp so that this package depends on
// the five calls it makes and not on the type that implements them -- which is
// why this package does not import target/dp at all.
type DebugPort interface {
	ReadAPReg(ctx context.Context, apSel, apReg uint8) (uint32, error)
	ReadAPRegMulti(ctx context.Context, apSel, apReg uint8, length int) ([]uint32, error)
	WriteAPReg(ctx context.Context, apSel, apReg uint8, value uint32) error
	WriteAPRegMulti(ctx context.Context, apSel, apReg uint8, values []uint32) error

	// ClearErrors clears the DP's sticky error bits.
	ClearErrors(ctx context.Context) error
}

// Client is target memory through one MEM-AP. See the package comment for the
// CSW/TAR shadow it keeps and what invalidates it.
type Client struct {
	dpc   DebugPort
	apSel uint8
	log   *slog.Logger

	// The AP's registers as this client last left them. See the package
	// comment for what the shadow is worth and what invalidates it.
	csw      uint32
	cswKnown bool
	tar      uint32
	tarKnown bool

	// Whether the AP can do byte-sized accesses. The size field is read-only
	// for widths an AP does not implement, so the only way to find out is to
	// write it and read it back.
	byteProbed bool
	byteOK     bool
}

// New builds a MEM-AP client on a debug port. apSel picks the access port.
func New(dpc DebugPort, apSel uint8) *Client {
	return &Client{dpc: dpc, apSel: apSel}
}

// Invalidate forgets what this client remembers about the AP's CSW and TAR.
// Call it whenever something else has written either of them -- see the
// package comment. It touches no wire.
// SetLogger points this client's tracing at a logger of the caller's choosing.
// Unset, it uses [slog.Default]. Every target access is one record at
// [swd.LevelTrace]; the one thing logged above that is an AP that turns out
// not to do byte accesses, which is a capability discovered once.
func (mapc *Client) SetLogger(l *slog.Logger) { mapc.log = l }

func (mapc *Client) logger() *slog.Logger {
	if mapc.log != nil {
		return mapc.log
	}
	return slog.Default()
}

// hex32 keeps addresses and register values readable in a log line. It is only
// ever reached inside an Enabled guard, so the allocation is not on a hot path.
func hex32(v uint32) string { return fmt.Sprintf("0x%08x", v) }

// Invalidate forgets what this client remembers about the AP's CSW and TAR, so
// that the next access writes both rather than assuming. It touches no wire.
//
// The shadow describes one AP on one target, and it is wrong whenever
// something else has written those registers or the AP it describes is no
// longer the one on the other end of the probe:
//
//   - The target reset or rebooted underneath this client: a power cycle or
//     brownout, nRESET, [swd.Probe.ResetTarget], an RP2040 rescue, or an
//     external OpenOCD `reset` on the same wire. Whether a given reset reaches
//     the AP depends on which power domain it lands in -- a core reset through
//     AIRCR.SYSRESETREQ does not, a chip reset does. Where that is not obvious,
//     invalidate: the cost is one TAR write, and the cost of being wrong is a
//     read of the wrong address.
//   - Something else drove the chip: an OpenOCD run that flashed or dumped it,
//     a second debugger on a multidrop target, or a caller batching raw
//     transfers of its own, as target/rp2040's Flasher does.
//   - A wire multiplexed to a different target, or the device at this position
//     hot-plugged and replaced between visits. The shadow now describes a chip
//     that is not listening, and the DPIDR read at bring-up will not notice,
//     because the replacement answers it just as well -- so a position that has
//     been away, however briefly, invalidates when it comes back. Prefer one
//     client per target; see the package comment.
//
// Getting this wrong is quiet rather than loud: the access lands at whatever
// address TAR actually holds and comes back with a plausible value.
//
// This drops the register shadow and nothing else. Whether the AP does
// byte-sized accesses is a property of that AP, found out once by probing, and
// survives any reset of the target -- but not a swap for a different device.
// [Client.Init] is the one that forgets that too, and re-reads CSW to check the
// AP is enabled, which is why a bring-up calls Init rather than this.
func (mapc *Client) Invalidate() {
	mapc.cswKnown = false
	mapc.tarKnown = false
}

// ClearErrors also drops the shadow: a transfer that faulted may have left TAR
// somewhere this client cannot work out, and guessing would put the next
// access at the wrong address.
func (mapc *Client) ClearErrors(ctx context.Context) error {
	mapc.Invalidate()
	return mapc.dpc.ClearErrors(ctx)
}

// ReadReg and WriteReg are the unmediated register access, for a caller that
// wants a register this client does not drive itself. Touching CSW or TAR
// through them drops the shadow, since what they leave behind is the caller's
// business and not tracked here.
func (mapc *Client) ReadReg(ctx context.Context, reg MemAPReg) (uint32, error) {
	value, err := mapc.dpc.ReadAPReg(ctx, mapc.apSel, uint8(reg))
	if l := mapc.logger(); l.Enabled(ctx, swd.LevelTrace) {
		l.Log(ctx, swd.LevelTrace, "ap read", "reg", reg, "value", hex32(value))
	}
	return value, err
}

func (mapc *Client) WriteReg(ctx context.Context, reg MemAPReg, value uint32) error {
	if l := mapc.logger(); l.Enabled(ctx, swd.LevelTrace) {
		l.Log(ctx, swd.LevelTrace, "ap write", "reg", reg, "value", hex32(value))
	}
	if reg == CSW || reg == TAR {
		mapc.Invalidate()
	}
	return mapc.dpc.WriteAPReg(ctx, mapc.apSel, uint8(reg), value)
}

// --- register staging -------------------------------------------------------

// setCSW puts the AP in the given access width and auto-increment mode, and
// costs nothing when it is already in it.
func (mapc *Client) setCSW(ctx context.Context, size, addrInc uint32) error {
	want := cswBase | (size & cswSizeMask) | (addrInc & cswAddrIncMask)
	if mapc.cswKnown && mapc.csw == want {
		return nil
	}
	if err := mapc.dpc.WriteAPReg(ctx, mapc.apSel, uint8(CSW), want); err != nil {
		mapc.cswKnown = false
		return err
	}
	mapc.csw, mapc.cswKnown = want, true
	return nil
}

// setTAR points the AP at an address, and costs nothing when it is already
// there — which is what makes polling one register a single transfer.
func (mapc *Client) setTAR(ctx context.Context, addr uint32) error {
	if mapc.tarKnown && mapc.tar == addr {
		return nil
	}
	if err := mapc.dpc.WriteAPReg(ctx, mapc.apSel, uint8(TAR), addr); err != nil {
		mapc.tarKnown = false
		return err
	}
	mapc.tar, mapc.tarKnown = addr, true
	return nil
}

// tarAdvanced records where n bytes of auto-incrementing DRW traffic starting
// at addr left TAR. The AP wraps inside the auto-increment region rather than
// carrying out of it, which is exactly what a run ending on the boundary hits.
func (mapc *Client) tarAdvanced(addr, n uint32) {
	end := addr + n
	mapc.tar = (addr &^ (tarAutoIncRegion - 1)) | (end & (tarAutoIncRegion - 1))
	mapc.tarKnown = true
}

// ensureByteAccess reports whether the AP implements byte-sized accesses,
// probing once and remembering the answer.
func (mapc *Client) ensureByteAccess(ctx context.Context) error {
	if mapc.byteProbed {
		if mapc.byteOK {
			return nil
		}
		return errNoByteAccess
	}
	mapc.byteProbed = true

	if err := mapc.setCSW(ctx, cswSizeByte, cswAddrIncOff); err != nil {
		return err
	}
	readBack, err := mapc.dpc.ReadAPReg(ctx, mapc.apSel, uint8(CSW))
	if err != nil {
		mapc.byteProbed = false
		mapc.cswKnown = false
		return err
	}
	// Whatever came back is what the AP actually holds, size field included.
	mapc.csw, mapc.cswKnown = readBack, true

	mapc.byteOK = readBack&cswSizeMask == cswSizeByte
	if !mapc.byteOK {
		mapc.logger().DebugContext(ctx, "MEM-AP does not support byte accesses",
			"ap", mapc.apSel, "csw", hex32(readBack))
		return errNoByteAccess
	}
	return nil
}

func (mapc *Client) Init(ctx context.Context) error {
	mapc.Invalidate()
	mapc.byteProbed, mapc.byteOK = false, false

	csw, err := mapc.ReadReg(ctx, CSW)
	if err != nil {
		return err
	}
	if csw&CSW_DeviceEn == 0 {
		return fmt.Errorf("MEM-AP is disabled")
	}
	// Basic mode, word access, increment by one: the state every block
	// transfer wants, and the one the shadow starts from.
	return mapc.setCSW(ctx, cswSizeWord, cswAddrIncSingle)
}

// --- word access ------------------------------------------------------------

// ReadTargetReg reads one word. Auto-increment goes off for it, so that a
// caller polling the same address pays one transfer per poll rather than a TAR
// write as well.
func (mapc *Client) ReadTargetReg(ctx context.Context, addr uint32) (uint32, error) {
	if err := mapc.setCSW(ctx, cswSizeWord, cswAddrIncOff); err != nil {
		return 0, err
	}
	if err := mapc.setTAR(ctx, addr); err != nil {
		return 0, err
	}
	value, err := mapc.dpc.ReadAPReg(ctx, mapc.apSel, uint8(DRW))
	if err != nil {
		mapc.tarKnown = false
		return 0, err
	}
	if l := mapc.logger(); l.Enabled(ctx, swd.LevelTrace) {
		l.Log(ctx, swd.LevelTrace, "read reg", "addr", hex32(addr), "value", hex32(value))
	}
	return value, nil
}

// WriteTargetReg writes one word, the same way.
func (mapc *Client) WriteTargetReg(ctx context.Context, addr uint32, value uint32) error {
	if err := mapc.setCSW(ctx, cswSizeWord, cswAddrIncOff); err != nil {
		return err
	}
	if err := mapc.setTAR(ctx, addr); err != nil {
		return err
	}
	if l := mapc.logger(); l.Enabled(ctx, swd.LevelTrace) {
		l.Log(ctx, swd.LevelTrace, "write reg", "addr", hex32(addr), "value", hex32(value))
	}
	if err := mapc.dpc.WriteAPReg(ctx, mapc.apSel, uint8(DRW), value); err != nil {
		mapc.tarKnown = false
		return err
	}
	return nil
}

// ReadTargetMem reads length WORDS at addr, which must be word-aligned. Runs
// longer than the MEM-AP's auto-increment window are split automatically.
func (mapc *Client) ReadTargetMem(ctx context.Context, addr uint32, length int) ([]uint32, error) {
	if l := mapc.logger(); l.Enabled(ctx, swd.LevelTrace) {
		l.Log(ctx, swd.LevelTrace, "read mem", "addr", hex32(addr), "words", length)
	}
	if addr%4 != 0 {
		return nil, fmt.Errorf("addr must be word-aligned, got 0x%x", addr)
	}
	if length == 0 {
		return nil, nil
	}
	if err := mapc.setCSW(ctx, cswSizeWord, cswAddrIncSingle); err != nil {
		return nil, err
	}

	res := make([]uint32, 0, length)
	for i := 0; i < length; {
		if err := mapc.setTAR(ctx, addr); err != nil {
			return nil, err
		}
		cl := mapc.runToBoundary(addr, length-i)
		values, err := mapc.dpc.ReadAPRegMulti(ctx, mapc.apSel, uint8(DRW), cl)
		if err != nil {
			mapc.tarKnown = false
			return nil, err
		}
		mapc.tarAdvanced(addr, uint32(cl*4))
		res = append(res, values...)
		addr += uint32(cl * 4)
		i += cl
	}
	return res, nil
}

// WriteTargetMem writes data at addr, which must be word-aligned, the same way.
func (mapc *Client) WriteTargetMem(ctx context.Context, addr uint32, data []uint32) error {
	if l := mapc.logger(); l.Enabled(ctx, swd.LevelTrace) {
		l.Log(ctx, swd.LevelTrace, "write mem", "addr", hex32(addr), "words", len(data))
	}
	if addr%4 != 0 {
		return fmt.Errorf("addr must be word-aligned, got 0x%x", addr)
	}
	if len(data) == 0 {
		return nil
	}
	if err := mapc.setCSW(ctx, cswSizeWord, cswAddrIncSingle); err != nil {
		return err
	}

	for i := 0; i < len(data); {
		if err := mapc.setTAR(ctx, addr); err != nil {
			return err
		}
		cl := mapc.runToBoundary(addr, len(data)-i)
		if err := mapc.dpc.WriteAPRegMulti(ctx, mapc.apSel, uint8(DRW), data[i:i+cl]); err != nil {
			mapc.tarKnown = false
			return err
		}
		mapc.tarAdvanced(addr, uint32(cl*4))
		addr += uint32(cl * 4)
		i += cl
	}
	return nil
}

// runToBoundary is how many of the remaining words can go out before TAR
// wraps and has to be reloaded.
func (mapc *Client) runToBoundary(addr uint32, remaining int) int {
	n := int((tarAutoIncRegion - addr&(tarAutoIncRegion-1)) / 4)
	if n > remaining {
		n = remaining
	}
	return n
}

// --- byte access ------------------------------------------------------------

// rwBytes moves a handful of bytes one byte-sized access at a time. It is for
// the unaligned ends of a range, where a word access would touch memory the
// caller did not name — which for an MMIO register or a location the target is
// itself writing is not the same thing at all.
func (mapc *Client) rwBytes(ctx context.Context, addr uint32, buf []byte, isWrite bool) error {
	if err := mapc.ensureByteAccess(ctx); err != nil {
		return err
	}
	if err := mapc.setCSW(ctx, cswSizeByte, cswAddrIncOff); err != nil {
		return err
	}

	for i := range buf {
		at := addr + uint32(i)
		lane := 8 * (at & 3) // which byte lane of DRW it sits in

		if err := mapc.setTAR(ctx, at); err != nil {
			return err
		}
		if isWrite {
			if err := mapc.dpc.WriteAPReg(ctx, mapc.apSel, uint8(DRW), uint32(buf[i])<<lane); err != nil {
				mapc.tarKnown = false
				return err
			}
			continue
		}
		v, err := mapc.dpc.ReadAPReg(ctx, mapc.apSel, uint8(DRW))
		if err != nil {
			mapc.tarKnown = false
			return err
		}
		buf[i] = byte(v >> lane)
	}
	return nil
}

// readWordsInto reads whole words straight into buf, which must be a multiple
// of four bytes long. It saves the caller's slice being built twice.
func (mapc *Client) readWordsInto(ctx context.Context, addr uint32, buf []byte) error {
	words, err := mapc.ReadTargetMem(ctx, addr, len(buf)/4)
	if err != nil {
		return err
	}
	for i, w := range words {
		binary.LittleEndian.PutUint32(buf[i*4:], w)
	}
	return nil
}

func (mapc *Client) writeWordsFrom(ctx context.Context, addr uint32, buf []byte) error {
	words := make([]uint32, len(buf)/4)
	for i := range words {
		words[i] = binary.LittleEndian.Uint32(buf[i*4:])
	}
	return mapc.WriteTargetMem(ctx, addr, words)
}

// ReadTargetMemBytes reads any address and length: byte accesses at the
// unaligned ends, block word transfers through the middle.
func (mapc *Client) ReadTargetMemBytes(ctx context.Context, addr uint32, length int) ([]byte, error) {
	if length < 0 {
		return nil, fmt.Errorf("negative length %d", length)
	}
	if length == 0 {
		return nil, nil
	}
	out := make([]byte, length)

	at, buf := addr, out
	head := int((4 - addr%4) % 4)
	if head > len(buf) {
		head = len(buf)
	}
	if head != 0 {
		err := mapc.rwBytes(ctx, at, buf[:head], false)
		if errors.Is(err, errNoByteAccess) {
			return mapc.readBytesViaWords(ctx, addr, length)
		}
		if err != nil {
			return nil, err
		}
		at, buf = at+uint32(head), buf[head:]
	}

	if words := len(buf) &^ 3; words != 0 {
		if err := mapc.readWordsInto(ctx, at, buf[:words]); err != nil {
			return nil, err
		}
		at, buf = at+uint32(words), buf[words:]
	}

	if len(buf) != 0 {
		err := mapc.rwBytes(ctx, at, buf, false)
		if errors.Is(err, errNoByteAccess) {
			return mapc.readBytesViaWords(ctx, addr, length)
		}
		if err != nil {
			return nil, err
		}
	}
	return out, nil
}

// WriteTargetMemBytes writes any address and length, the same way.
func (mapc *Client) WriteTargetMemBytes(ctx context.Context, addr uint32, data []byte) error {
	if len(data) == 0 {
		return nil
	}

	at, buf := addr, data
	head := int((4 - addr%4) % 4)
	if head > len(buf) {
		head = len(buf)
	}
	if head != 0 {
		err := mapc.rwBytes(ctx, at, buf[:head], true)
		if errors.Is(err, errNoByteAccess) {
			return mapc.writeBytesViaWords(ctx, addr, data)
		}
		if err != nil {
			return err
		}
		at, buf = at+uint32(head), buf[head:]
	}

	if words := len(buf) &^ 3; words != 0 {
		if err := mapc.writeWordsFrom(ctx, at, buf[:words]); err != nil {
			return err
		}
		at, buf = at+uint32(words), buf[words:]
	}

	if len(buf) != 0 {
		err := mapc.rwBytes(ctx, at, buf, true)
		if errors.Is(err, errNoByteAccess) {
			return mapc.writeBytesViaWords(ctx, addr, data)
		}
		if err != nil {
			return err
		}
	}
	return nil
}

// --- the word-only fallback -------------------------------------------------
//
// An AP that implements only word accesses cannot be asked for an unaligned
// range at all, so the range is widened to whole words and the ends are
// read-modify-written. That is a real difference in behaviour, not just in
// speed: the bytes either side of the range are read and written back, and
// anything the target changed in between is lost. It is the best such an AP
// can do, and the reason the byte path above is preferred.

func (mapc *Client) readBytesViaWords(ctx context.Context, addr uint32, length int) ([]byte, error) {
	start := addr &^ 3
	end := (addr + uint32(length) + 3) &^ 3

	buf := make([]byte, end-start)
	if err := mapc.readWordsInto(ctx, start, buf); err != nil {
		return nil, err
	}
	offset := addr - start
	return buf[offset : offset+uint32(length)], nil
}

func (mapc *Client) writeBytesViaWords(ctx context.Context, addr uint32, data []byte) error {
	start := addr &^ 3
	end := (addr + uint32(len(data)) + 3) &^ 3

	buf := make([]byte, end-start)
	if start != addr || end != addr+uint32(len(data)) {
		if err := mapc.readWordsInto(ctx, start, buf); err != nil {
			return err
		}
	}
	copy(buf[addr-start:], data)
	return mapc.writeWordsFrom(ctx, start, buf)
}

func (r MemAPReg) String() string {
	switch r {
	case CSW:
		return "CSW"
	case TAR:
		return "TAR"
	case DRW:
		return "DRW"
	case BD0:
		return "BD0"
	case BD1:
		return "BD1"
	case BD2:
		return "BD2"
	case BD3:
		return "BD3"
	case BASE:
		return "BASE"
	}
	return fmt.Sprintf("0x%x", uint8(r))
}
