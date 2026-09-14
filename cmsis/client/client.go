// Package client is a [swd.Probe] that speaks CMSIS-DAP to a probe somewhere
// else.
//
// The packet encoding lives here once. How the packets get to the probe is a
// [Transport], and the packages under cmsis/transport supply those: USB HID, a
// TCP or Unix socket, or a cmsis/server in the same process. Swapping one for
// another changes nothing above this line.
package client

//
// The command helpers here (newCmd/exec/execStatus, info/infoString) and the
// DAP_Info accessor set follow the structure of the Mongoose OS mos tool's
// CMSIS-DAP client (github.com/mongoose-os/mos,
// cli/flash/common/cmsis-dap/dap/cmsis_dap_client.go), Copyright (c) 2014-2019
// Cesanta Software Limited, licensed under the Apache License, Version 2.0.
// The packet encoding itself is the CMSIS-DAP specification's, and the bodies,
// the Transport seam and everything from SWDSequence down are new.
// See the NOTICE file.
//

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"fmt"
	"time"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/cmsis/wire"
)

// Transport carries CMSIS-DAP packets to a probe and brings the replies back.
//
// Implementations may assume one Exchange at a time: the protocol has no
// request identifiers, so a reply belongs to whoever asked last.
type Transport interface {
	// Exchange sends one request packet, waits for the reply, writes it into
	// response and returns how many bytes of it are valid.
	Exchange(ctx context.Context, request, response []byte) (int, error)

	// MaxPacketSize is the largest packet this transport can carry. Zero means
	// it has no opinion and whatever the probe reports through
	// DAP_ID_PACKET_SIZE wins.
	MaxPacketSize() int

	// Close releases the transport.
	Close() error
}

// Probe is a CMSIS-DAP probe reached through a [Transport].
type Probe struct {
	t             Transport
	maxPacketSize int
}

var (
	_ swd.Probe = (*Probe)(nil)
	_ swd.Cmsis = (*Probe)(nil)
)

// New builds a probe on a transport and asks it how big its packets are, which
// doubles as a check that the thing on the far end is answering at all.
func New(ctx context.Context, t Transport) (*Probe, error) {
	p := &Probe{
		t:             t,
		maxPacketSize: 512, // enough to ask the question below
	}

	resp, err := p.info(ctx, wire.InfoPacketSize)
	if err != nil {
		p.t.Close()
		return nil, fmt.Errorf("cmsis: failed to get packet size: %w", err)
	}
	if resp.Len() >= 3 {
		var length uint8
		var size uint16
		binary.Read(resp, binary.LittleEndian, &length)
		binary.Read(resp, binary.LittleEndian, &size)
		if size > 0 {
			p.maxPacketSize = int(size)
		}
	}
	if tm := t.MaxPacketSize(); tm > 0 && tm < p.maxPacketSize {
		p.maxPacketSize = tm
	}

	return p, nil
}

// MaxPacketSize is the largest CMSIS-DAP packet this probe accepts.
func (p *Probe) MaxPacketSize() int { return p.maxPacketSize }

func (p *Probe) Close(ctx context.Context) error {
	_ = ctx
	return p.t.Close()
}

// ---------------- Packet plumbing ----------------

func newCmd(id uint8) *bytes.Buffer {
	return bytes.NewBuffer([]byte{id})
}

// exec sends one packet and returns the reply with the echoed command ID
// stripped.
func (p *Probe) exec(ctx context.Context, args *bytes.Buffer) (*bytes.Buffer, error) {
	packet := args.Bytes()
	if len(packet) > p.maxPacketSize {
		return nil, fmt.Errorf("cmsis: packet too long (max %d, got %d)", p.maxPacketSize, len(packet))
	}

	resp := make([]byte, p.maxPacketSize)
	n, err := p.t.Exchange(ctx, packet, resp)
	if err != nil {
		return nil, err
	}
	if n <= 0 {
		return nil, fmt.Errorf("cmsis: empty response")
	}
	if n > len(resp) {
		return nil, fmt.Errorf("cmsis: transport reported %d bytes into a %d byte buffer", n, len(resp))
	}
	if resp[0] != packet[0] {
		return nil, fmt.Errorf("cmsis: command echo mismatch, want 0x%02x got 0x%02x", packet[0], resp[0])
	}
	return bytes.NewBuffer(resp[1:n]), nil
}

// execStatus sends a packet whose whole reply is a status byte.
func (p *Probe) execStatus(ctx context.Context, args *bytes.Buffer) error {
	id := args.Bytes()[0]
	resp, err := p.exec(ctx, args)
	if err != nil {
		return err
	}
	if resp.Len() == 0 {
		return fmt.Errorf("cmsis: command 0x%02x returned no status", id)
	}
	if st := resp.Bytes()[0]; st != wire.StatusOK {
		return fmt.Errorf("cmsis: command 0x%02x failed (0x%02x)", id, st)
	}
	return nil
}

// ---------------- DAP_Info ----------------

// Info is the raw DAP_Info command, for the IDs [swd.Cmsis] does not cover.
func (p *Probe) info(ctx context.Context, id uint8) (*bytes.Buffer, error) {
	args := newCmd(wire.CmdInfo)
	args.WriteByte(id)
	return p.exec(ctx, args)
}

func (p *Probe) infoString(ctx context.Context, id uint8) (string, error) {
	resp, err := p.info(ctx, id)
	if err != nil {
		return "", err
	}
	length, err := resp.ReadByte()
	if err != nil {
		return "", fmt.Errorf("cmsis: short DAP_Info(0x%02x) response", id)
	}
	s := make([]byte, length)
	if _, err := resp.Read(s); err != nil && length > 0 {
		return "", fmt.Errorf("cmsis: short DAP_Info(0x%02x) response", id)
	}
	// Probes null-terminate these; Go strings do not want the terminator.
	return string(bytes.TrimRight(s, "\x00")), nil
}

func (p *Probe) VendorID(ctx context.Context) (string, error) {
	return p.infoString(ctx, wire.InfoVendor)
}
func (p *Probe) ProductID(ctx context.Context) (string, error) {
	return p.infoString(ctx, wire.InfoProduct)
}
func (p *Probe) SerialNumber(ctx context.Context) (string, error) {
	return p.infoString(ctx, wire.InfoSerialNumber)
}
func (p *Probe) FirmwareVersion(ctx context.Context) (string, error) {
	return p.infoString(ctx, wire.InfoFirmwareVer)
}
func (p *Probe) TargetVendor(ctx context.Context) (string, error) {
	return p.infoString(ctx, wire.InfoDeviceVendor)
}
func (p *Probe) TargetName(ctx context.Context) (string, error) {
	return p.infoString(ctx, wire.InfoDeviceName)
}

// ---------------- Connection ----------------

func (p *Probe) Connect(ctx context.Context) error {
	args := newCmd(wire.CmdConnect)
	args.WriteByte(wire.PortSWD)
	resp, err := p.exec(ctx, args)
	if err != nil {
		return err
	}
	if resp.Len() == 0 || resp.Bytes()[0] != wire.PortSWD {
		return fmt.Errorf("cmsis: probe would not connect in SWD mode")
	}
	return nil
}

func (p *Probe) Disconnect(ctx context.Context) error {
	return p.execStatus(ctx, newCmd(wire.CmdDisconnect))
}

func (p *Probe) SetConnected(ctx context.Context, on bool) error {
	return p.hostStatus(ctx, wire.HostStatusConnected, on)
}

func (p *Probe) SetRunning(ctx context.Context, on bool) error {
	return p.hostStatus(ctx, wire.HostStatusRunning, on)
}

func (p *Probe) hostStatus(ctx context.Context, kind uint8, on bool) error {
	args := newCmd(wire.CmdHostStatus)
	args.WriteByte(kind)
	if on {
		args.WriteByte(1)
	} else {
		args.WriteByte(0)
	}
	return p.execStatus(ctx, args)
}

func (p *Probe) ResetTarget(ctx context.Context) error {
	return p.execStatus(ctx, newCmd(wire.CmdResetTarget))
}

// ---------------- Pins ----------------

// ReadPins samples the SWJ pins through DAP_SWJ_Pins, which is a write of an
// empty mask: the command always reports the pin state back.
func (p *Probe) ReadPins(ctx context.Context) (swd.Pin, error) {
	return p.WritePins(ctx, 0, 0, 0)
}

// WritePins drives the selected pins and waits for them to read back as asked,
// which is how DAP_SWJ_Pins lets a caller confirm a reset actually took.
//
// Which pins are wired is the far end's business: a probe reports an unwired
// pin low and a wait on it times out. CMSIS-DAP caps the wait at 3 seconds.
func (p *Probe) WritePins(ctx context.Context, value, mask swd.Pin, timeout time.Duration) (swd.Pin, error) {
	const maxWait = 3 * time.Second
	if timeout < 0 {
		timeout = 0
	}
	if timeout > maxWait {
		timeout = maxWait
	}

	args := newCmd(wire.CmdSWJPins)
	args.WriteByte(byte(value))
	args.WriteByte(byte(mask))
	binary.Write(args, binary.LittleEndian, uint32(timeout/time.Microsecond))

	resp, err := p.exec(ctx, args)
	if err != nil {
		return 0, err
	}
	state, err := resp.ReadByte()
	if err != nil {
		return 0, errors.New("cmsis: short DAP_SWJ_Pins response")
	}
	return swd.Pin(state), nil
}

func (p *Probe) SetClock(ctx context.Context, hz uint32) error {
	args := newCmd(wire.CmdSWJClock)
	binary.Write(args, binary.LittleEndian, hz)
	return p.execStatus(ctx, args)
}

// Configure needs two commands on the wire: CMSIS-DAP splits the retry counts
// and the wire timing across DAP_TransferConfigure and DAP_SWD_Configure.
func (p *Probe) Configure(ctx context.Context, cfg swd.Config) error {
	args := newCmd(wire.CmdTransferConfigure)
	binary.Write(args, binary.LittleEndian, cfg.IdleCycles)
	binary.Write(args, binary.LittleEndian, cfg.WaitRetry)
	binary.Write(args, binary.LittleEndian, cfg.MatchRetry)
	if err := p.execStatus(ctx, args); err != nil {
		return err
	}

	turnaround := cfg.Turnaround
	if turnaround == 0 {
		turnaround = 1
	}
	if turnaround > 4 {
		return fmt.Errorf("cmsis: turnaround must be 1..4, got %d", turnaround)
	}
	value := turnaround - 1
	if cfg.DataPhase {
		value |= 0x04
	}
	args = newCmd(wire.CmdSWDConfigure)
	args.WriteByte(value)
	return p.execStatus(ctx, args)
}

// ---------------- Sequences ----------------

func (p *Probe) Sequence(ctx context.Context, numBits int, data []byte) error {
	if numBits < 1 || numBits > 256 {
		return fmt.Errorf("cmsis: sequence length must be 1..256, got %d", numBits)
	}
	need := (numBits + 7) / 8
	if len(data) < need {
		return fmt.Errorf("cmsis: sequence needs %d data bytes for %d bits, got %d", need, numBits, len(data))
	}

	args := newCmd(wire.CmdSWJSequence)
	// 256 bits is encoded as zero, which is the only reason this is not a plain
	// cast.
	args.WriteByte(uint8(numBits))
	args.Write(data[:need])
	return p.execStatus(ctx, args)
}

func (p *Probe) SWDSequence(ctx context.Context, seqs []swd.Sequence) ([]byte, error) {
	if len(seqs) == 0 {
		return nil, fmt.Errorf("cmsis: at least one sequence is required")
	}
	if len(seqs) > 255 {
		return nil, fmt.Errorf("cmsis: too many sequences: %d", len(seqs))
	}

	args := newCmd(wire.CmdSWDSequence)
	args.WriteByte(uint8(len(seqs)))

	expect := 0
	for i, s := range seqs {
		if s.NumCycles < 1 || s.NumCycles > 64 {
			return nil, fmt.Errorf("cmsis: sequence %d: NumCycles must be 1..64, got %d", i, s.NumCycles)
		}
		// 64 cycles is encoded as zero.
		info := s.NumCycles & wire.SeqClockMask
		byteCount := int(s.NumCycles+7) / 8

		if s.Input {
			args.WriteByte(info | wire.SeqDataIn)
			expect += byteCount
			continue
		}
		if len(s.Data) < byteCount {
			return nil, fmt.Errorf("cmsis: sequence %d: need %d data bytes for %d cycles, got %d",
				i, byteCount, s.NumCycles, len(s.Data))
		}
		args.WriteByte(info)
		args.Write(s.Data[:byteCount])
	}

	resp, err := p.exec(ctx, args)
	if err != nil {
		return nil, err
	}
	st, err := resp.ReadByte()
	if err != nil {
		return nil, fmt.Errorf("cmsis: short DAP_SWD_Sequence response")
	}
	if st != wire.StatusOK {
		return nil, fmt.Errorf("cmsis: DAP_SWD_Sequence failed (0x%02x)", st)
	}
	if resp.Len() != expect {
		return nil, fmt.Errorf("cmsis: DAP_SWD_Sequence returned %d bytes, want %d", resp.Len(), expect)
	}
	return bytes.Clone(resp.Bytes()), nil
}

// ---------------- Transfers ----------------

// transferRetries is how many times a whole transfer list is resent when the
// probe reports it ran out of its own WAIT retries. The target is busy, not
// broken, and a transfer that answered WAIT did not happen — so resending the
// list is safe even when it contains writes.
const transferRetries = 5

func (p *Probe) Transfer(ctx context.Context, reqs []swd.Request) (swd.Status, []uint32, error) {
	var (
		st     swd.Status
		values []uint32
		err    error
	)
	for i := 0; i < transferRetries; i++ {
		st, values, err = p.doTransfer(ctx, reqs)
		if err != nil && st == swd.StatusWait {
			continue
		}
		return st, values, err
	}
	return swd.StatusWait, nil, &swd.TransferError{Status: swd.StatusWait, Total: len(reqs)}
}

func (p *Probe) doTransfer(ctx context.Context, reqs []swd.Request) (swd.Status, []uint32, error) {
	if len(reqs) > 255 {
		return 0, nil, fmt.Errorf("cmsis: too many transfers in one list: %d", len(reqs))
	}

	args := newCmd(wire.CmdTransfer)
	args.WriteByte(0) // DAP index: a JTAG chain position, always 0 in SWD
	args.WriteByte(uint8(len(reqs)))

	expect := 0
	for i, r := range reqs {
		raw, wantsData, produces, err := encodeRequest(r)
		if err != nil {
			return 0, nil, fmt.Errorf("cmsis: transfer %d: %w", i, err)
		}
		args.WriteByte(raw)
		if wantsData {
			binary.Write(args, binary.LittleEndian, r.Data)
		}
		expect += produces
	}

	resp, err := p.exec(ctx, args)
	if err != nil {
		return 0, nil, err
	}

	var done uint8
	var st swd.Status
	if binary.Read(resp, binary.LittleEndian, &done) != nil ||
		binary.Read(resp, binary.LittleEndian, &st) != nil {
		return st, nil, fmt.Errorf("cmsis: short DAP_Transfer response")
	}
	if !st.Ok() || int(done) != len(reqs) {
		return st, nil, &swd.TransferError{Status: st, Completed: int(done), Total: len(reqs)}
	}

	values := make([]uint32, 0, expect)
	for i := 0; i < expect; i++ {
		var v uint32
		if binary.Read(resp, binary.LittleEndian, &v) != nil {
			return st, values, fmt.Errorf("cmsis: DAP_Transfer returned %d of %d words", i, expect)
		}
		values = append(values, v)
	}
	return st, values, nil
}

// encodeRequest packs one request and reports how many bytes of data it carries
// and how many response words it produces.
func encodeRequest(r swd.Request) (raw uint8, wantsData bool, produces int, err error) {
	if r.Reg&^uint8(wire.TransferAddr) != 0 {
		return 0, false, 0, fmt.Errorf("register 0x%02x is not one of 0x0, 0x4, 0x8, 0xc", r.Reg)
	}
	raw = r.Reg & wire.TransferAddr
	if r.AP {
		raw |= wire.TransferAPnDP
	}
	if r.Timestamp {
		raw |= wire.TransferTimestamp
	}

	switch r.Op {
	case swd.OpRead:
		raw |= wire.TransferRnW
		produces = 1
	case swd.OpReadMatch:
		// A match read polls until it matches and reports the outcome in the
		// status, not as a value.
		raw |= wire.TransferRnW | wire.TransferMatchValue
		wantsData = true
	case swd.OpWrite:
		wantsData = true
	case swd.OpWriteMatch:
		// This only sets the mask for later match reads; nothing goes on the
		// wire and nothing comes back.
		raw |= wire.TransferMatchMask
		wantsData = true
		return raw, wantsData, 0, nil
	default:
		return 0, false, 0, fmt.Errorf("unknown op %d", r.Op)
	}

	if r.Timestamp {
		produces++
	}
	return raw, wantsData, produces, nil
}

func (p *Probe) MaxBlockSize() int {
	// One byte of command, one of DAP index, two of count and one of request,
	// and the rest is words.
	const header = 5
	return (p.maxPacketSize - header) / 4
}

func (p *Probe) BlockRead(ctx context.Context, ap bool, reg uint8, n int) ([]uint32, error) {
	if n == 0 {
		return nil, nil
	}
	if n < 0 || n > p.MaxBlockSize() {
		return nil, fmt.Errorf("cmsis: block read of %d words, max %d", n, p.MaxBlockSize())
	}
	raw, err := blockRequest(ap, reg, true)
	if err != nil {
		return nil, err
	}

	args := newCmd(wire.CmdTransferBlock)
	args.WriteByte(0)
	binary.Write(args, binary.LittleEndian, uint16(n))
	args.WriteByte(raw)

	resp, err := p.exec(ctx, args)
	if err != nil {
		return nil, err
	}

	done, st, err := readBlockHeader(resp)
	if err != nil {
		return nil, err
	}
	if !st.Ok() || done != n {
		return nil, &swd.TransferError{Status: st, Completed: done, Total: n}
	}

	out := make([]uint32, 0, n)
	for i := 0; i < n; i++ {
		var v uint32
		if binary.Read(resp, binary.LittleEndian, &v) != nil {
			return out, fmt.Errorf("cmsis: block read returned %d of %d words", i, n)
		}
		out = append(out, v)
	}
	return out, nil
}

func (p *Probe) BlockWrite(ctx context.Context, ap bool, reg uint8, data []uint32) error {
	if len(data) == 0 {
		return nil
	}
	if len(data) > p.MaxBlockSize() {
		return fmt.Errorf("cmsis: block write of %d words, max %d", len(data), p.MaxBlockSize())
	}
	raw, err := blockRequest(ap, reg, false)
	if err != nil {
		return err
	}

	args := newCmd(wire.CmdTransferBlock)
	args.WriteByte(0)
	binary.Write(args, binary.LittleEndian, uint16(len(data)))
	args.WriteByte(raw)
	for _, v := range data {
		binary.Write(args, binary.LittleEndian, v)
	}

	resp, err := p.exec(ctx, args)
	if err != nil {
		return err
	}

	done, st, err := readBlockHeader(resp)
	if err != nil {
		return err
	}
	if !st.Ok() || done != len(data) {
		return &swd.TransferError{Status: st, Completed: done, Total: len(data)}
	}
	return nil
}

func readBlockHeader(resp *bytes.Buffer) (done int, st swd.Status, err error) {
	var count uint16
	if binary.Read(resp, binary.LittleEndian, &count) != nil ||
		binary.Read(resp, binary.LittleEndian, &st) != nil {
		return 0, 0, errors.New("cmsis: short DAP_TransferBlock response")
	}
	return int(count), st, nil
}

func blockRequest(ap bool, reg uint8, read bool) (uint8, error) {
	if reg&^uint8(wire.TransferAddr) != 0 {
		return 0, fmt.Errorf("cmsis: register 0x%02x is not one of 0x0, 0x4, 0x8, 0xc", reg)
	}
	raw := reg & wire.TransferAddr
	if ap {
		raw |= wire.TransferAPnDP
	}
	if read {
		raw |= wire.TransferRnW
	}
	return raw, nil
}

// ---------------- Vendor escape hatch ----------------

// VendorCommand sends one command in the ID_DAP_Vendor0..31 range and returns
// the reply with the echoed command ID stripped. request is what follows the
// ID; the ID itself is cmd, and this is the only way to reach that range --
// what a vendor command means is a particular probe's business, and nothing
// here interprets one.
//
// The range is checked rather than trusted: this is the vendor escape hatch,
// not a way to put an arbitrary packet on the wire behind the rest of this
// probe's back.
func (p *Probe) VendorCommand(ctx context.Context, cmd uint8, request []byte) ([]byte, error) {
	if cmd < wire.CmdVendor0 || cmd > wire.CmdVendor31 {
		return nil, fmt.Errorf("cmsis: command 0x%02x is not in the ID_DAP_Vendor0..31 range", cmd)
	}
	args := newCmd(cmd)
	args.Write(request)
	resp, err := p.exec(ctx, args)
	if err != nil {
		return nil, err
	}
	return bytes.Clone(resp.Bytes()), nil
}
