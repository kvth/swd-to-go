// Package dp is the ARM debug port: the first layer above a [swd.Probe].
//
// A transfer on the wire can only reach four registers at a time. Which four
// depends on the DP's SELECT register, so every access to an access port is
// really two things -- point SELECT at the right AP and bank, then read or
// write one of the four registers now visible. This package is what hides that
// split: [Client.ReadAPReg] and friends take an AP number and a register
// offset and do whatever SELECT work that implies.
//
// It remembers what it last wrote to SELECT and skips a write that would
// change nothing, which matters because a run of accesses to one AP -- which
// is what target/mem does for every word of target memory -- would otherwise
// pay a SELECT write per access. The shadow assumes this client is the only
// thing writing SELECT on this DP.
//
// Above the raw registers sit the two things a caller actually wants at
// bring-up: [Client.Init] powers the debug and system domains up, and
// [Client.ClearErrors] clears the sticky error bits that make every later
// transfer fail.
//
// # One client per target
//
// A [Client]'s shadow is one target's state, so a wire multiplexed between
// several targets wants one [Client] per target rather than one shared between
// them. They cost an allocation and no I/O, so a board with a dozen positions
// holds a dozen of these and nothing else.
//
// Sharing one across a mux is not wrong, but it has to be told: call
// [Client.Invalidate] or [Client.Init] after every switch, which throws away
// exactly the saved writes the shadow exists for. Per-target clients keep
// theirs. What sharing actually costs is in target/mem, whose shadow is bigger
// and includes a capability it has to discover by probing.
package dp

//
// This file is derived from the Mongoose OS mos tool
// (github.com/mongoose-os/mos, cli/flash/common/cmsis-dap/dp/cmsis_dap_dp.go)
// and has been modified: the DAP client became swd.Probe, errors are wrapped
// rather than annotated, logging moved to log/slog, and the SELECT shadow and
// its Invalidate are new. See the NOTICE file.
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
	"fmt"
	"log/slog"

	"github.com/kvth/swd-to-go"
)

type DPReg uint8

const (
	DPIDR      DPReg = 0x00
	DPCTRLSTAT       = 0x04
	DPSELECT         = 0x08
)

// Client is a debug-port client on a probe. It is the first layer above
// [swd.Probe]: everything higher up reaches the wire through this.
//
// It is a concrete type rather than an interface because there is one debug
// port and one way to drive it. A caller that wants to substitute something
// else substitutes the [swd.Probe] underneath, which is where the seam is.
type Client struct {
	probe swd.Probe
	log   *slog.Logger

	// The DP's SELECT register as this client last left it. See [Client.Invalidate]
	// for what makes it wrong.
	selectValue uint32
	selectKnown bool
}

// Invalidate forgets what this client remembers about the DP's SELECT
// register, so that the next AP access writes it rather than assuming. It
// touches no wire.
//
// The shadow is one target's state, and it is wrong whenever something else
// has moved SELECT or the target it describes is no longer the one on the
// other end of the probe:
//
//   - A line reset, which is what every bring-up starts with -- so a client
//     kept across a re-attach needs this, or [Client.Init], which writes
//     SELECT itself and is what a bring-up normally calls anyway.
//   - A wire multiplexed to a different target, or the device at this position
//     hot-plugged and replaced between visits. The shadow now describes a chip
//     that is not listening, and the DPIDR read at bring-up will not notice,
//     because the replacement answers it just as well. Prefer one client per
//     target there; see the package comment.
//   - The target reset or rebooted underneath this client: a power cycle or
//     brownout, nRESET, [swd.Probe.ResetTarget], or an RP2040 rescue, which
//     holds the whole chip in reset. A core reset through AIRCR.SYSRESETREQ
//     lands in a different power domain and leaves the debug port alone, so it
//     does not need this. Where it is not obvious which domain a reset reached,
//     invalidate: the cost is one SELECT write on the next AP access.
//   - Something else driving the same wire: an external OpenOCD run to flash
//     or reset the chip, a second debugger on a multidrop target, or a caller
//     batching raw transfers of its own. They write SELECT and nothing here is
//     told.
//
// Getting this wrong is quiet rather than loud: the access goes to whatever AP
// and bank the target actually has selected, and comes back with a value.
//
// [Client.Init] covers all of these on the path that matters, since it writes
// SELECT and sets the shadow from what it wrote -- a bring-up wants one or the
// other, not both.
func (dpc *Client) Invalidate() {
	dpc.selectKnown = false
}

// SetLogger points this client's tracing at a logger of the caller's choosing.
// Unset, it uses [slog.Default]. Every register access is one record at
// [swd.LevelTrace]; nothing else is logged.
func (dpc *Client) SetLogger(l *slog.Logger) { dpc.log = l }

func (dpc *Client) logger() *slog.Logger {
	if dpc.log != nil {
		return dpc.log
	}
	return slog.Default()
}

// hex32 keeps register values readable in a log line. It is only ever reached
// inside an Enabled guard, so the allocation is not on any hot path.
func hex32(v uint32) string { return fmt.Sprintf("0x%08x", v) }

// New builds a debug-port client on a probe.
func New(probe swd.Probe) *Client {
	return &Client{probe: probe}
}

func (dpc *Client) readReg(ctx context.Context, reg uint8, ap bool) (uint32, error) {
	_, data, err := dpc.probe.Transfer(ctx, []swd.Request{
		{Op: swd.OpRead, AP: ap, Reg: reg},
	})
	if err != nil {
		return 0, fmt.Errorf("failed to read DP reg %d: %w", reg, err)
	}
	return data[0], nil
}

func (dpc *Client) readRegMulti(ctx context.Context, reg uint8, ap bool, length int) ([]uint32, error) {
	maxChunkSize := dpc.probe.MaxBlockSize()
	var res []uint32
	for length > 0 {
		chunkSize := length
		if chunkSize > maxChunkSize {
			chunkSize = maxChunkSize
		}
		chunk, err := dpc.probe.BlockRead(ctx, ap, reg, chunkSize)
		if err != nil {
			return nil, err
		}
		res = append(res, chunk...)
		length -= chunkSize
	}
	return res, nil
}

func (dpc *Client) ReadDPReg(ctx context.Context, reg DPReg) (uint32, error) {
	value, err := dpc.readReg(ctx, uint8(reg), false /* ap */)
	if l := dpc.logger(); l.Enabled(ctx, swd.LevelTrace) {
		l.Log(ctx, swd.LevelTrace, "dp read", "reg", reg, "value", hex32(value))
	}
	return value, err
}

func (dpc *Client) writeReg(ctx context.Context, reg uint8, ap bool, value uint32) error {
	_, _, err := dpc.probe.Transfer(ctx, []swd.Request{
		{Op: swd.OpWrite, AP: ap, Reg: reg, Data: value},
	})
	return err
}

func (dpc *Client) writeRegMulti(ctx context.Context, reg uint8, ap bool, values []uint32) error {
	offset := 0
	maxChunkSize := dpc.probe.MaxBlockSize()
	for offset < len(values) {
		chunk := values[offset:]
		if len(chunk) > maxChunkSize {
			chunk = chunk[:maxChunkSize]
		}
		if err := dpc.probe.BlockWrite(ctx, ap, reg, chunk); err != nil {
			return err
		}
		offset += len(chunk)
	}
	return nil
}

func (dpc *Client) WriteDPReg(ctx context.Context, reg DPReg, value uint32) error {
	if l := dpc.logger(); l.Enabled(ctx, swd.LevelTrace) {
		l.Log(ctx, swd.LevelTrace, "dp write", "reg", reg, "value", hex32(value))
	}
	return dpc.writeReg(ctx, uint8(reg), false /* ap */, value)
}

// ABORT bits, each clearing one sticky error flag in CTRL/STAT.
const (
	STKCMPCLR  uint32 = 1 << 1
	STKERRCLR  uint32 = 1 << 2
	WDERRCLR   uint32 = 1 << 3
	ORUNERRCLR uint32 = 1 << 4

	ALLERRCLR = STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR

	// dpABORT is register 0x00 written rather than read.
	dpABORT DPReg = 0x00
)

// ClearErrors clears the DP's sticky error bits. A transfer that faulted
// leaves them set and every later transfer fails until they are gone, so
// anything that expects faults -- resetting a target, walking over unmapped
// memory -- has to call this before carrying on.
func (dpc *Client) ClearErrors(ctx context.Context) error {
	return dpc.WriteDPReg(ctx, dpABORT, ALLERRCLR)
}

func (dpc *Client) Init(ctx context.Context) error {
	if err := dpc.ClearErrors(ctx); err != nil {
		return err
	}

	if err := dpc.WriteDPReg(ctx, DPSELECT, 0); err != nil {
		return err
	}

	dpc.selectValue, dpc.selectKnown = 0, true
	//if err := dpc.SetDbgPower(ctx, true, true); err != nil {
	//	return err
	//}

	// Clear all the errors (if any).
	//if err := dpc.WriteDPReg(ctx, DPCTRLSTAT, 0x50000f00); err != nil {
	//	return err
	//}

	var CDBGPWRUPREQ uint32 = (1 << 28)
	var CSYSPWRUPREQ uint32 = (1 << 30)
	if err := dpc.WriteDPReg(ctx, DPCTRLSTAT, CDBGPWRUPREQ|CSYSPWRUPREQ); err != nil {
		return err
	}
	return nil
}

// GetIDR reads DPIDR, the register that identifies the debug port.
//
// The read is not optional. A DP coming out of a line reset answers nothing
// else until DPIDR has been read, and a target on a multidrop wire does not
// consider itself selected until the same read happens.
//
// What comes back is the raw word. Whether it is the right one is the caller's
// question, because only the caller knows what it is talking to: see
// [github.com/kvth/swd-to-go/target/rp2040.DPIDR] for one that does.
func (dpc *Client) GetIDR(ctx context.Context) (uint32, error) {
	v, err := dpc.ReadDPReg(ctx, DPIDR)
	if err != nil {
		return 0, fmt.Errorf("failed to read DPIDR: %w", err)
	}
	return v, nil
}

func (dpc *Client) SetDbgPower(ctx context.Context, dbg, sys bool) error {
	var reqMask, ackMask uint32
	if dbg {
		reqMask |= 0x10000000
		ackMask |= 0x20000000
	}
	if sys {
		reqMask |= 0x40000000
		ackMask |= 0x80000000
	}
	for {
		statValue, err := dpc.ReadDPReg(ctx, DPCTRLSTAT)
		if err != nil {
			return fmt.Errorf("failed to read DPCTRLSTAT: %w", err)
		}
		if statValue&0xf0000000 == (reqMask | ackMask) {
			break
		}
		ctrlValue := (statValue & 0x07ffffff) | reqMask
		if err := dpc.WriteDPReg(ctx, DPCTRLSTAT, ctrlValue); err != nil {
			return fmt.Errorf("failed to write DPCTRLSTAT: %w", err)
		}
	}
	return nil
}

func (dpc *Client) DbgReset(ctx context.Context) error {
	statValue, err := dpc.ReadDPReg(ctx, DPCTRLSTAT)
	if err != nil {
		return fmt.Errorf("failed to read DPCTRLSTAT: %w", err)
	}
	// Set reset request
	ctrlValue := (statValue & 0xf3ffffff) | 0x04000000
	if err := dpc.WriteDPReg(ctx, DPCTRLSTAT, ctrlValue); err != nil {
		return fmt.Errorf("failed to write DPCTRLSTAT: %w", err)
	}
	// Wait for ack
	for {
		statValue, err = dpc.ReadDPReg(ctx, DPCTRLSTAT)
		if err != nil {
			return fmt.Errorf("failed to read DPCTRLSTAT: %w", err)
		}
		if statValue&0x08000000 != 0 {
			break
		}
	}
	// Remove request
	ctrlValue = (statValue & 0xf3ffffff)
	if err := dpc.WriteDPReg(ctx, DPCTRLSTAT, ctrlValue); err != nil {
		return fmt.Errorf("failed to write DPCTRLSTAT: %w", err)
	}
	// Wait for ack to clear
	for {
		statValue, err = dpc.ReadDPReg(ctx, DPCTRLSTAT)
		if err != nil {
			return fmt.Errorf("failed to read DPCTRLSTAT: %w", err)
		}
		if statValue&0x08000000 == 0 {
			break
		}
	}
	return nil
}

func (dpc *Client) selectAP(ctx context.Context, apSel, apBank uint8) error {
	// The bits this does not drive -- DPBANKSEL and the reserved field -- are
	// carried over from whatever is already there. With no shadow to carry
	// them over from they start at zero, which is a value rather than a guess:
	// the write below then puts the register in a state this client knows.
	var base uint32
	if dpc.selectKnown {
		base = dpc.selectValue
	}
	sv := (base & 0x00ffff0f) | (uint32(apSel) << 24) | ((uint32(apBank) & 0xf) << 4)
	if dpc.selectKnown && sv == dpc.selectValue {
		return nil
	}
	if err := dpc.WriteDPReg(ctx, DPSELECT, sv); err != nil {
		return fmt.Errorf("failed to select AP %d bank %d: %w", apSel, apBank, err)
	}
	dpc.selectValue, dpc.selectKnown = sv, true
	return nil
}

func (dpc *Client) ReadAPReg(ctx context.Context, apSel, apReg uint8) (uint32, error) {
	apBank := apReg / 16
	if err := dpc.selectAP(ctx, apSel, apBank); err != nil {
		return 0, err
	}
	apReg = apReg % 16
	return dpc.readReg(ctx, apReg, true /* ap */)
}

func (dpc *Client) ReadAPRegMulti(ctx context.Context, apSel, apReg uint8, length int) ([]uint32, error) {
	apBank := apReg / 16
	if err := dpc.selectAP(ctx, apSel, apBank); err != nil {
		return nil, err
	}
	apReg = apReg % 16
	return dpc.readRegMulti(ctx, apReg, true /* ap */, length)
}

func (dpc *Client) WriteAPReg(ctx context.Context, apSel, apReg uint8, value uint32) error {
	apBank := apReg / 16
	if err := dpc.selectAP(ctx, apSel, apBank); err != nil {
		return err
	}
	apReg = apReg % 16
	return dpc.writeReg(ctx, apReg, true /* ap */, value)
}

func (dpc *Client) WriteAPRegMulti(ctx context.Context, apSel, apReg uint8, values []uint32) error {
	apBank := apReg / 16
	if err := dpc.selectAP(ctx, apSel, apBank); err != nil {
		return err
	}
	apReg = apReg % 16
	return dpc.writeRegMulti(ctx, apReg, true /* ap */, values)
}

func (r DPReg) String() string {
	switch r {
	case DPIDR:
		return "DPIDR"
	case DPCTRLSTAT:
		return "DPCTRLSTAT"
	case DPSELECT:
		return "DPSELECT"
	}
	return fmt.Sprintf("0x%x", uint8(r))
}
