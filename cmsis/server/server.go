// Package server serves CMSIS-DAP packets for a [swd.Probe].
//
// It is the mirror image of cmsis/client: the client turns Probe calls into
// packets, this turns packets back into Probe calls. Neither contains any SWD
// logic of its own — the posted-read pipeline, the WAIT retries and the block
// transfers all live in the probe behind it — which is what lets a server in
// front of a bitbang probe re-export a locally wired target, and a server in
// front of a client re-export a probe it reached over a socket.
//
// Where the probe cannot do what a command asks, the reply is whatever the
// protocol already has for that case rather than a stub that lies.
// DAP_SWJ_Pins and DAP_ResetTarget are whatever the probe's own [swd.Probe]
// methods answer: pins that report [swd.ErrNotImplemented] become DAP_ERROR,
// since every line low would be a guess, while a reset that reports it becomes
// an OK with no device-specific reset, which is what a probe without one says.
// DAP_HostStatus needs a probe that is itself a [swd.Cmsis] and answers OK
// without one, there being nothing to light up. JTAG is not implemented at all.
package server

import (
	"context"
	"encoding/binary"
	"errors"
	"time"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/cmsis/wire"
)

// Options describes the probe to a connecting debugger. The zero value gives
// the defaults below, which is what you want unless you are impersonating a
// specific probe.
type Options struct {
	Vendor          string
	Product         string
	SerialNumber    string
	FirmwareVersion string
	TargetVendor    string
	TargetName      string
	BoardVendor     string
	BoardName       string

	// PacketSize is the largest request this server accepts, and what it
	// reports through DAP_ID_PACKET_SIZE. PacketCount is how many packets a
	// debugger may have in flight.
	PacketSize  int
	PacketCount int

	// HandleVendor, when set, is offered every command in the
	// ID_DAP_Vendor0..31 range before the server refuses it.
	//
	// Nothing in this module implements one. What a vendor command means is a
	// particular probe's business, and a CMSIS-DAP implementation that made up
	// its own meanings would be lying to any other probe's client; a program
	// that owns both ends supplies a handler here instead.
	//
	// cmd is the command ID, request is what followed it, and response is
	// where to write the reply -- also after the ID, which the server echoes
	// itself.
	//
	// It returns how many request bytes it consumed and how many response
	// bytes it produced, both excluding the command ID, and false if it does
	// not claim this command, in which case the server answers
	// ID_DAP_Invalid. The counts matter: a batched command after this one is
	// read from where this one stopped.
	//
	// Neither slice is guaranteed to be any particular length -- a command
	// late in a batch gets whatever the ones before it left -- so a handler
	// must check both before indexing and decline when there is not enough
	// room. The server contains a handler that gets this wrong, but an
	// ID_DAP_Invalid is all the client will see.
	HandleVendor func(ctx context.Context, cmd uint8, request, response []byte) (consumed, produced int, ok bool)
}

func (o Options) withDefaults() Options {
	if o.Vendor == "" {
		o.Vendor = "swd-to-go"
	}
	if o.Product == "" {
		o.Product = "CMSIS-DAP probe"
	}
	if o.SerialNumber == "" {
		o.SerialNumber = "000000000000"
	}
	if o.FirmwareVersion == "" {
		o.FirmwareVersion = "2.1.2"
	}
	if o.TargetVendor == "" {
		o.TargetVendor = "Arm"
	}
	if o.TargetName == "" {
		o.TargetName = "Cortex-M"
	}
	if o.BoardVendor == "" {
		o.BoardVendor = "Arm"
	}
	if o.BoardName == "" {
		o.BoardName = "Arm board"
	}
	if o.PacketSize <= 0 {
		o.PacketSize = 1024
	}
	if o.PacketCount <= 0 {
		o.PacketCount = 8
	}
	return o
}

// Server turns CMSIS-DAP packets into calls on a probe. It is not safe for
// concurrent use, for the same reason the probe behind it is not; the
// transports in cmsis/server/tcp serve one client at a time.
type Server struct {
	probe swd.Probe
	opts  Options

	// ctx is what the probe calls get. Packet handlers have nowhere to put a
	// context of their own — the wire protocol has no notion of one — so the
	// server carries the one it was built with.
	ctx context.Context

	// cfg accumulates what DAP_TransferConfigure and DAP_SWD_Configure each set
	// half of, since [swd.Config] is applied whole.
	cfg swd.Config

	port uint8
}

// New builds a server in front of a probe. ctx bounds every probe call the
// server makes; use [context.Background] if you do not want that.
func New(ctx context.Context, probe swd.Probe, opts Options) *Server {
	return &Server{
		probe: probe,
		opts:  opts.withDefaults(),
		ctx:   ctx,
		cfg:   swd.Config{WaitRetry: 100, Turnaround: 1},
		port:  wire.PortDisabled,
	}
}

// MaxPacketSize is the largest request packet this server accepts.
func (s *Server) MaxPacketSize() int { return s.opts.PacketSize }

// ---------------- DAP_Info ----------------

func putString(dst []byte, str string) uint8 {
	n := copy(dst, str)
	if n < len(dst) {
		dst[n] = 0
		n++
	}
	return uint8(n)
}

func (s *Server) info(id uint8, out []byte) uint8 {
	switch id {
	case wire.InfoVendor:
		return putString(out, s.opts.Vendor)
	case wire.InfoProduct:
		return putString(out, s.opts.Product)
	case wire.InfoSerialNumber:
		return putString(out, s.opts.SerialNumber)
	case wire.InfoFirmwareVer:
		return putString(out, s.opts.FirmwareVersion)
	case wire.InfoDeviceVendor:
		return putString(out, s.opts.TargetVendor)
	case wire.InfoDeviceName:
		return putString(out, s.opts.TargetName)
	case wire.InfoBoardVendor:
		return putString(out, s.opts.BoardVendor)
	case wire.InfoBoardName:
		return putString(out, s.opts.BoardName)
	case wire.InfoProductFWVer:
		return putString(out, "")
	case wire.InfoCapabilities:
		out[0] = wire.CapSWD | wire.CapAtomicCommands | wire.CapTimestampClock
		out[1] = 0
		return 2
	case wire.InfoTimestampClock:
		binary.LittleEndian.PutUint32(out, wire.TimestampHz)
		return 4
	case wire.InfoPacketSize:
		binary.LittleEndian.PutUint16(out, uint16(s.opts.PacketSize))
		return 2
	case wire.InfoPacketCount:
		out[0] = byte(s.opts.PacketCount)
		return 1
	default:
		return 0
	}
}

// ---------------- Connection ----------------

func (s *Server) connect(req, resp []byte) uint32 {
	port := req[0]
	if port == wire.PortAutodetect {
		port = wire.PortSWD
	}
	if port != wire.PortSWD || s.probe.Connect(s.ctx) != nil {
		s.port = wire.PortDisabled
		resp[0] = wire.PortDisabled
		return (1 << 16) | 1
	}
	s.port = wire.PortSWD
	resp[0] = wire.PortSWD
	return (1 << 16) | 1
}

func (s *Server) disconnect(resp []byte) uint32 {
	s.port = wire.PortDisabled
	resp[0] = status(s.probe.Disconnect(s.ctx))
	return 1
}

func (s *Server) hostStatus(req, resp []byte) uint32 {
	hs, ok := s.probe.(swd.Cmsis)
	if !ok {
		// Nothing to light up. Saying OK is the polite answer and what real
		// probes without LEDs do.
		resp[0] = wire.StatusOK
		return (2 << 16) | 1
	}

	on := req[1] != 0
	var err error
	switch req[0] {
	case wire.HostStatusConnected:
		err = hs.SetConnected(s.ctx, on)
	case wire.HostStatusRunning:
		err = hs.SetRunning(s.ctx, on)
	default:
		resp[0] = wire.StatusError
		return (2 << 16) | 1
	}
	resp[0] = status(err)
	return (2 << 16) | 1
}

func (s *Server) resetTarget(resp []byte) uint32 {
	// A probe with no device-specific sequence says so with
	// swd.ErrNotImplemented; DAP_ResetTarget reports that as "no
	// device-specific reset" rather than an error.
	err := s.probe.ResetTarget(s.ctx)
	if errors.Is(err, swd.ErrNotImplemented) {
		resp[0] = wire.StatusOK
		resp[1] = 0 // no device-specific reset
		return 2
	}
	resp[0] = status(err)
	resp[1] = 1
	return 2
}

func (s *Server) delay(req, resp []byte) uint32 {
	micros := binary.LittleEndian.Uint16(req[:2])
	// A busy wait, because the whole point of this command is a delay shorter
	// and tighter than the scheduler can give.
	end := time.Now().Add(time.Duration(micros) * time.Microsecond)
	for time.Now().Before(end) {
	}
	resp[0] = wire.StatusOK
	return (2 << 16) | 1
}

// ---------------- Clock, pins and configuration ----------------

func (s *Server) swjClock(req, resp []byte) uint32 {
	hz := binary.LittleEndian.Uint32(req[:4])
	if hz == 0 {
		resp[0] = wire.StatusError
		return (4 << 16) | 1
	}
	resp[0] = status(s.probe.SetClock(s.ctx, hz))
	return (4 << 16) | 1
}

func (s *Server) swjPins(req, resp []byte) uint32 {
	value := swd.Pin(req[0])
	mask := swd.Pin(req[1])
	waitMicros := binary.LittleEndian.Uint32(req[2:6])

	// A probe that cannot reach the pins says so with swd.ErrNotImplemented.
	// Reporting every line low would be a guess; DAP_ERROR is the truth.
	pins, err := s.probe.WritePins(s.ctx, value, mask, time.Duration(waitMicros)*time.Microsecond)
	if err != nil {
		resp[0] = wire.StatusError
		return (6 << 16) | 1
	}
	resp[0] = byte(pins)
	return (6 << 16) | 1
}

func (s *Server) transferConfigure(req, resp []byte) uint32 {
	s.cfg.IdleCycles = req[0]
	s.cfg.WaitRetry = binary.LittleEndian.Uint16(req[1:3])
	s.cfg.MatchRetry = binary.LittleEndian.Uint16(req[3:5])
	resp[0] = status(s.probe.Configure(s.ctx, s.cfg))
	return (5 << 16) | 1
}

func (s *Server) swdConfigure(req, resp []byte) uint32 {
	s.cfg.Turnaround = (req[0] & 0x03) + 1
	s.cfg.DataPhase = req[0]&0x04 != 0
	resp[0] = status(s.probe.Configure(s.ctx, s.cfg))
	return (1 << 16) | 1
}

// ---------------- Sequences ----------------

func (s *Server) swjSequence(req, resp []byte) uint32 {
	count := int(req[0])
	if count == 0 {
		count = 256
	}
	byteCount := uint32(count+7) >> 3
	if int(byteCount)+1 > len(req) {
		resp[0] = wire.StatusError
		return ((byteCount + 1) << 16) | 1
	}
	resp[0] = status(s.probe.Sequence(s.ctx, count, req[1:1+byteCount]))
	return ((byteCount + 1) << 16) | 1
}

func (s *Server) swdSequence(req, resp []byte) uint32 {
	count := int(req[0])
	pos := 1

	seqs := make([]swd.Sequence, 0, count)
	for i := 0; i < count; i++ {
		if pos >= len(req) {
			resp[0] = wire.StatusError
			return (uint32(pos) << 16) | 1
		}
		info := req[pos]
		pos++

		cycles := info & wire.SeqClockMask
		if cycles == 0 {
			cycles = 64
		}
		byteCount := int(cycles+7) / 8

		if info&wire.SeqDataIn != 0 {
			seqs = append(seqs, swd.Sequence{NumCycles: cycles, Input: true})
			continue
		}
		if pos+byteCount > len(req) {
			resp[0] = wire.StatusError
			return (uint32(pos) << 16) | 1
		}
		seqs = append(seqs, swd.Sequence{NumCycles: cycles, Data: req[pos : pos+byteCount]})
		pos += byteCount
	}

	out, err := s.probe.SWDSequence(s.ctx, seqs)
	if err != nil {
		resp[0] = wire.StatusError
		return (uint32(pos) << 16) | 1
	}

	resp[0] = wire.StatusOK
	n := copy(resp[1:], out)
	return (uint32(pos) << 16) | uint32(1+n)
}

// ---------------- Transfers ----------------

// transfer decodes DAP_Transfer, hands the whole list to the probe and writes
// back [count, status, words...].
//
// The request has to be walked to its end even when the probe stops early,
// because the caller needs the byte count to find the next command in a batch.
func (s *Server) transfer(req, resp []byte) uint32 {
	pos := 1 // the DAP index, which is a JTAG chain position and always 0 here
	count := int(req[pos])
	pos++

	reqs := make([]swd.Request, 0, count)
	for i := 0; i < count; i++ {
		if pos >= len(req) {
			break
		}
		raw := req[pos]
		pos++

		r := swd.Request{
			AP:        raw&wire.TransferAPnDP != 0,
			Reg:       raw & wire.TransferAddr,
			Timestamp: raw&wire.TransferTimestamp != 0,
		}

		var wantsData bool
		switch {
		case raw&wire.TransferRnW != 0 && raw&wire.TransferMatchValue != 0:
			r.Op, wantsData = swd.OpReadMatch, true
		case raw&wire.TransferRnW != 0:
			r.Op = swd.OpRead
		case raw&wire.TransferMatchMask != 0:
			r.Op, wantsData = swd.OpWriteMatch, true
		default:
			r.Op, wantsData = swd.OpWrite, true
		}
		if wantsData {
			if pos+4 > len(req) {
				break
			}
			r.Data = binary.LittleEndian.Uint32(req[pos : pos+4])
			pos += 4
		}
		reqs = append(reqs, r)
	}

	st, values, err := s.probe.Transfer(s.ctx, reqs)
	done := len(reqs)
	var te *swd.TransferError
	if errors.As(err, &te) {
		done = te.Completed
	} else if err != nil {
		// Not a transfer failure — a context cancellation or a malformed
		// request. Report it as a protocol error, which is the only thing the
		// wire can say.
		st, done = swd.StatusError, 0
	}

	resp[0] = byte(done)
	resp[1] = byte(st)
	n := 2
	for _, v := range values {
		if n+4 > len(resp) {
			break
		}
		binary.LittleEndian.PutUint32(resp[n:n+4], v)
		n += 4
	}
	return (uint32(pos) << 16) | uint32(n)
}

// transferBlock decodes DAP_TransferBlock and writes back
// [count(2), status, words...].
func (s *Server) transferBlock(req, resp []byte) uint32 {
	count := int(binary.LittleEndian.Uint16(req[1:3]))
	raw := req[3]
	ap := raw&wire.TransferAPnDP != 0
	reg := raw & wire.TransferAddr
	read := raw&wire.TransferRnW != 0

	// The request length is fixed by the header, not by how far the probe got.
	consumed := uint32(4)
	if !read {
		consumed += uint32(count) * 4
	}

	done, st, values := 0, swd.StatusOK, []uint32(nil)

	switch {
	case count == 0:
		// Nothing to do, and nothing went wrong.
	case read:
		var err error
		values, err = s.probe.BlockRead(s.ctx, ap, reg, count)
		done, st = len(values), blockStatus(err)
	default:
		data := make([]uint32, 0, count)
		for i := 0; i < count && 4+i*4+4 <= len(req); i++ {
			data = append(data, binary.LittleEndian.Uint32(req[4+i*4:8+i*4]))
		}
		err := s.probe.BlockWrite(s.ctx, ap, reg, data)
		done, st = len(data), blockStatus(err)
		if err != nil {
			done = completedFrom(err)
		}
	}

	binary.LittleEndian.PutUint16(resp[0:2], uint16(done))
	resp[2] = byte(st)
	n := 3
	for _, v := range values {
		if n+4 > len(resp) {
			break
		}
		binary.LittleEndian.PutUint32(resp[n:n+4], v)
		n += 4
	}
	return (consumed << 16) | uint32(n)
}

// writeAbort writes the DP ABORT register, which is how a debugger clears a
// sticky error. The result is deliberately not reported: ABORT is the command
// you send when the port is already unhappy.
func (s *Server) writeAbort(req, resp []byte) uint32 {
	value := binary.LittleEndian.Uint32(req[1:5])
	_, _, _ = s.probe.Transfer(s.ctx, []swd.Request{{Op: swd.OpWrite, Reg: 0x00, Data: value}})
	resp[0] = wire.StatusOK
	return (5 << 16) | 1
}

func blockStatus(err error) swd.Status {
	if err == nil {
		return swd.StatusOK
	}
	var te *swd.TransferError
	if errors.As(err, &te) {
		return te.Status
	}
	return swd.StatusError
}

func completedFrom(err error) int {
	var te *swd.TransferError
	if errors.As(err, &te) {
		return te.Completed
	}
	return 0
}

func status(err error) byte {
	if err != nil {
		return wire.StatusError
	}
	return wire.StatusOK
}

// ---------------- Dispatch ----------------

// processCmd handles one command and returns a packed value: the request bytes
// consumed in the upper 16 bits, the response bytes produced in the lower 16.
func (s *Server) processCmd(request []byte, response []byte) uint32 {
	if len(request) == 0 || len(response) == 0 {
		return 0
	}

	cmd := request[0]
	if cmd >= wire.CmdVendor0 && cmd <= wire.CmdVendor31 {
		// This is a CMSIS-DAP implementation, not a particular probe: what a
		// vendor command means is that probe's business, so nothing here
		// claims any of the range unless the program that built this server
		// said what its probe means by it.
		if consumed, produced, ok := s.vendor(cmd, request[1:], response[1:]); ok {
			response[0] = cmd
			return ((uint32(consumed) + 1) << 16) | (uint32(produced) + 1)
		}
		response[0] = wire.CmdInvalid
		return (1 << 16) | 1
	}

	// Every handler below indexes its request and response directly, so the
	// room they need is checked once, here. Without this a truncated packet --
	// or a batch that ran its response buffer down to the last byte -- indexes
	// past the end, and this server faces a network.
	minReq, minResp := commandMinimums(cmd)
	if len(request) < 1+minReq || len(response) < 1+minResp {
		response[0] = wire.CmdInvalid
		return (1 << 16) | 1
	}

	response[0] = cmd
	req := request[1:]
	resp := response[1:]

	var num uint32

	switch cmd {
	case wire.CmdInfo:
		n := s.info(req[0], resp[1:])
		resp[0] = n
		return (2 << 16) | (2 + uint32(n))

	case wire.CmdHostStatus:
		num = s.hostStatus(req, resp)
	case wire.CmdConnect:
		num = s.connect(req, resp)
	case wire.CmdDisconnect:
		num = s.disconnect(resp)
	case wire.CmdDelay:
		num = s.delay(req, resp)
	case wire.CmdResetTarget:
		num = s.resetTarget(resp)
	case wire.CmdSWJPins:
		num = s.swjPins(req, resp)
	case wire.CmdSWJClock:
		num = s.swjClock(req, resp)
	case wire.CmdSWJSequence:
		num = s.swjSequence(req, resp)
	case wire.CmdSWDConfigure:
		num = s.swdConfigure(req, resp)
	case wire.CmdSWDSequence:
		num = s.swdSequence(req, resp)
	case wire.CmdTransferConfigure:
		num = s.transferConfigure(req, resp)
	case wire.CmdTransfer:
		num = s.transfer(req, resp)
	case wire.CmdTransferBlock:
		num = s.transferBlock(req, resp)
	case wire.CmdWriteABORT:
		num = s.writeAbort(req, resp)

	case wire.CmdJTAGSequence:
		num = s.jtagSequence(req, resp)
	case wire.CmdJTAGConfigure:
		num = ((uint32(req[0]) + 1) << 16) | 1
		resp[0] = wire.StatusError
	case wire.CmdJTAGIDCode:
		num = (1 << 16) | 1
		resp[0] = wire.StatusError

	default:
		response[0] = wire.CmdInvalid
		return (1 << 16) | 1
	}

	// The command ID is one request byte and one response byte on top of
	// whatever the handler reported, and both counts have to be *added* rather
	// than or-ed in: a handler that consumed request bytes of its own already
	// has a non-zero upper half, and or-ing 1<<16 into it would leave the
	// command ID uncounted. That only shows up once commands are packed
	// back-to-back, where the next one is then read from the wrong offset.
	return num + (1 << 16) + 1
}

// vendor offers one command to the configured handler, and contains it.
//
// The handler is code this package did not write, reached from a socket, so a
// bug in it must not take the gateway down with it: a panic — indexing past
// the response buffer is the obvious one — is turned into a refusal, and so is
// any count that does not fit the buffers it was given.
func (s *Server) vendor(cmd uint8, request, response []byte) (consumed, produced int, ok bool) {
	if s.opts.HandleVendor == nil {
		return 0, 0, false
	}

	defer func() {
		if r := recover(); r != nil {
			consumed, produced, ok = 0, 0, false
		}
	}()

	consumed, produced, ok = s.opts.HandleVendor(s.ctx, cmd, request, response)
	if !ok || consumed < 0 || produced < 0 || consumed > len(request) || produced > len(response) {
		return 0, 0, false
	}
	return consumed, produced, true
}

// commandMinimums is how many request and response bytes each command needs
// past the command ID before its handler can safely look at either. Handlers
// that produce a variable amount bound the rest themselves.
func commandMinimums(cmd uint8) (req, resp int) {
	switch cmd {
	case wire.CmdInfo:
		// One ID in; a length byte and up to four bytes of value out.
		return 1, 5
	case wire.CmdHostStatus:
		return 2, 1
	case wire.CmdConnect:
		return 1, 1
	case wire.CmdDisconnect:
		return 0, 1
	case wire.CmdDelay:
		return 2, 1
	case wire.CmdResetTarget:
		return 0, 2
	case wire.CmdSWJPins:
		return 6, 1
	case wire.CmdSWJClock:
		return 4, 1
	case wire.CmdSWJSequence:
		return 1, 1
	case wire.CmdSWDConfigure:
		return 1, 1
	case wire.CmdSWDSequence:
		return 1, 1
	case wire.CmdTransferConfigure:
		return 5, 1
	case wire.CmdTransfer:
		// A DAP index and a count in; a count and a status out.
		return 2, 2
	case wire.CmdTransferBlock:
		return 4, 3
	case wire.CmdWriteABORT:
		return 5, 1
	case wire.CmdJTAGSequence, wire.CmdJTAGConfigure:
		return 1, 1
	case wire.CmdJTAGIDCode:
		return 0, 1
	default:
		return 0, 1
	}
}

// jtagSequence refuses the command but still walks the request, so a batch
// containing one can find the command after it.
func (s *Server) jtagSequence(req, resp []byte) uint32 {
	resp[0] = wire.StatusError
	consumed := uint32(1)

	count := int(req[0])
	pos := 1
	for i := 0; i < count && pos < len(req); i++ {
		info := uint32(req[pos])
		pos++
		cycles := info & wire.SeqClockMask
		if cycles == 0 {
			cycles = 64
		}
		byteCount := (cycles + 7) / 8
		pos += int(byteCount)
		consumed += byteCount + 1
	}
	return (consumed << 16) | 1
}
