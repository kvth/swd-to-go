// Package swd is the interface layer every other package in this module is
// written against.
//
// There are three altitudes here, and keeping them apart is the whole point:
//
//   - [BitBanger] is a pin-level SWD driver. It knows about bits, turnarounds
//     and a single ACK. The packages under drivers/ implement it.
//
//   - [Probe] is a transaction-level SWD probe: reads and writes of DP and AP
//     registers, with WAIT retries, posted AP reads and RDBUFF fixups already
//     handled. probe/bitbang turns any BitBanger into one; cmsis/client is one
//     that speaks CMSIS-DAP to a probe somewhere else. They are
//     interchangeable.
//
//   - Everything above — target/dp, target/mem, target/rp2040, target/rtt —
//     is written against Probe and nothing else.
//
// cmsis/server sits off to the side: it consumes a Probe and serves CMSIS-DAP
// packets for it, which is the mirror image of cmsis/client.
package swd

//
// Parts of this file are derived from the Mongoose OS mos tool
// (github.com/mongoose-os/mos,
// cli/flash/common/cmsis-dap/dap/cmsis_dap_client_interface.go) and have been
// modified: [Status]'s predicate methods and the [Request]/[Op] shape come
// from the original's TransferStatus and TransferRequest/TransferOp, and
// [Probe] follows its DAPClient method set. The Pin, Sequence and Config
// types, the block and sequence calls and all documentation are new.
// See the NOTICE file.
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
	"time"
)

// Probe is one SWD debug probe.
//
// Implementations are not required to be safe for concurrent use, and none of
// the ones in this module are: a debug port is a single piece of wire and the
// posted-read state machine running over it has no room for a second caller.
// Serialise access yourself if you need it.
type Probe interface {
	// Connect powers the SWD port up and drives the lines. It does not touch
	// the wire protocol: the line reset and the dormant/SWD switching dance are
	// the caller's business, through [Probe.Sequence] and the helpers in this
	// package.
	Connect(ctx context.Context) error

	// Disconnect releases the port and floats the lines.
	Disconnect(ctx context.Context) error

	// SetClock sets the SWCLK frequency in Hz. Probes round to whatever they
	// can actually generate, which for a bitbang driver is "not very close".
	SetClock(ctx context.Context, hz uint32) error

	// Configure sets the transfer parameters described by [Config].
	Configure(ctx context.Context, cfg Config) error

	// Sequence clocks numBits raw bits out of SWDIO with no protocol framing at
	// all: line reset, JTAG-to-dormant, dormant-to-SWD. data is packed
	// LSB-first, first bit in bit 0 of data[0]. numBits is 1..256.
	Sequence(ctx context.Context, numBits int, data []byte) error

	// SWDSequence clocks a list of individually directed sequences and returns
	// the bits the input ones read back, concatenated and packed LSB-first.
	//
	// This is the only way to write TARGETSEL on a multidrop bus, where the
	// target deliberately does not drive an ACK and an ordinary transfer would
	// call that a protocol error. See [SelectTarget].
	SWDSequence(ctx context.Context, seqs []Sequence) ([]byte, error)

	// Transfer runs a list of DP and AP reads and writes in order.
	//
	// values holds one word per read, in request order; writes and
	// match-value reads contribute none. status describes the transfer the
	// list stopped on, so a short values slice says how far it got. A
	// request with Timestamp set contributes an extra word immediately
	// before its own (see [Request.Timestamp]).
	//
	// The pipelining an AP read needs — issue now, collect the result from the
	// next transfer or from RDBUFF — is the probe's job, not the caller's.
	Transfer(ctx context.Context, reqs []Request) (status Status, values []uint32, err error)

	// BlockRead reads n words through one register with the address
	// auto-incrementing in the probe. n must not exceed [Probe.MaxBlockSize].
	BlockRead(ctx context.Context, ap bool, reg uint8, n int) ([]uint32, error)

	// BlockWrite writes data through one register the same way.
	BlockWrite(ctx context.Context, ap bool, reg uint8, data []uint32) error

	// MaxBlockSize is the most words one block transfer can carry. For a probe
	// on the far end of a packet transport this is what the packet size allows;
	// for a bitbang probe it is a self-imposed chunk size.
	MaxBlockSize() int

	// ResetTarget runs the probe's device-specific reset sequence, which is not
	// the same thing as pulsing nRESET through [Probe.WritePins]. A probe with
	// no such sequence of its own returns [ErrNotImplemented].
	ResetTarget(ctx context.Context) error

	// ReadPins samples all the SWJ pins at once.
	ReadPins(ctx context.Context) (Pin, error)

	// WritePins drives the pins selected by mask to the levels in value, then
	// waits up to timeout for them to actually read back that way. It returns
	// the levels it ended up seeing. A zero timeout does not wait.
	//
	// A probe whose wiring simply does not have a pin reports it low and lets a
	// wait on it time out, which is the truthful answer; a probe that cannot
	// reach the pins at all returns [ErrNotImplemented].
	WritePins(ctx context.Context, value, mask Pin, timeout time.Duration) (Pin, error)

	// Close releases the probe. It does not disconnect the port first.
	Close(ctx context.Context) error
}

// Pin is a bit mask over the SWJ pins, using the CMSIS-DAP bit positions so it
// can be passed through DAP_SWJ_Pins unchanged.
type Pin uint8

const (
	PinSWCLK  Pin = 1 << 0
	PinSWDIO  Pin = 1 << 1
	PinTDI    Pin = 1 << 2
	PinTDO    Pin = 1 << 3
	PinNTRST  Pin = 1 << 5
	PinNRESET Pin = 1 << 7
)

// Config holds the transfer parameters a probe applies on the caller's behalf.
// It is one struct rather than two calls because callers set it once at
// startup; a probe that needs two commands on the wire to apply it sends two.
type Config struct {
	// IdleCycles is the number of clocks appended after each transfer.
	IdleCycles uint8

	// WaitRetry is how many times to repeat a transfer that answered WAIT
	// before giving up.
	WaitRetry uint16

	// MatchRetry is how many times to repeat a match-value read whose value did
	// not match.
	MatchRetry uint16

	// Turnaround is the number of SWCLK cycles the bus takes to change
	// direction, 1 to 4. Zero means leave it alone, which is 1.
	Turnaround uint8

	// DataPhase requests the data phase be clocked out even on a WAIT or FAULT
	// response. Targets differ on whether they want this.
	DataPhase bool
}

// Op is what one [Request] does.
type Op uint8

const (
	// OpRead reads the register.
	OpRead Op = iota
	// OpReadMatch reads the register repeatedly until the value, masked with
	// the mask set by a preceding OpWriteMatch, equals Data. It produces no
	// value; failure shows up as [Status.ValueMismatch].
	OpReadMatch
	// OpWrite writes Data to the register.
	OpWrite
	// OpWriteMatch sets the mask used by subsequent OpReadMatch requests. It
	// touches no wire.
	OpWriteMatch
)

// Request is one DP or AP register access.
type Request struct {
	Op Op
	// AP selects the access port bank; false means the debug port.
	AP bool
	// Reg is the register offset within the bank, a multiple of 4 in 0x0..0xc.
	// Which bank those four registers belong to is whatever DP SELECT was last
	// set to.
	Reg uint8
	// Data is the value for OpWrite, the match value for OpReadMatch and the
	// match mask for OpWriteMatch. It is ignored for OpRead.
	Data uint32

	// Timestamp asks the probe to sample its own clock when this transfer
	// completes and return it as an extra word in values, immediately before
	// this request's data word (or in place of it, for a write, which has
	// none). It exists so cmsis/server can pass the CMSIS-DAP TIMESTAMP bit
	// through to a probe that supports it; leave it false and every read
	// contributes exactly one word.
	Timestamp bool
}

// Sequence is one directed run of clocks for [Probe.SWDSequence].
type Sequence struct {
	// NumCycles is 1 to 64.
	NumCycles uint8
	// Input releases SWDIO and samples it; otherwise SWDIO is driven from Data.
	Input bool
	// Data is required for output sequences and ignored for input ones. It is
	// packed LSB-first and must hold exactly (NumCycles+7)/8 bytes.
	Data []byte
}

// Status is the result of the transfer a [Probe.Transfer] list stopped on. The
// low three bits are the raw SWD ACK; the rest flag failures the ACK has no
// encoding for.
type Status uint8

const (
	// StatusOK is ACK=OK with no error flags.
	StatusOK Status = 1
	// StatusWait is ACK=WAIT: the target was busy and the retries ran out.
	StatusWait Status = 2
	// StatusFault is ACK=FAULT: the target refused the transfer.
	StatusFault Status = 4
	// StatusError means no valid ACK came back, or a parity check failed.
	StatusError Status = 8
	// StatusMismatch means an OpReadMatch never saw its value.
	StatusMismatch Status = 0x10
)

// Ok reports whether the transfer succeeded outright.
func (s Status) Ok() bool {
	return s.AckValue() == 1 && !s.SWDError() && !s.ValueMismatch()
}

// AckValue is the raw three-bit SWD ACK: 1 OK, 2 WAIT, 4 FAULT.
func (s Status) AckValue() uint8 { return uint8(s & 7) }

// SWDError reports a protocol or parity error, which the ACK cannot express.
func (s Status) SWDError() bool { return s&StatusError != 0 }

// ValueMismatch reports that an OpReadMatch gave up.
func (s Status) ValueMismatch() bool { return s&StatusMismatch != 0 }

func (s Status) String() string {
	switch {
	case s.Ok():
		return "OK"
	case s.SWDError():
		return "ERROR"
	case s.ValueMismatch():
		return "MISMATCH"
	case s.AckValue() == uint8(StatusWait):
		return "WAIT"
	case s.AckValue() == uint8(StatusFault):
		return "FAULT"
	default:
		return "NO-ACK"
	}
}
