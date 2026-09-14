// Package bitbang turns a pin-level [swd.BitBanger] into a transaction-level
// [swd.Probe].
//
// This is the layer that makes a locally wired target and a CMSIS-DAP probe on
// the far end of a cable interchangeable. Everything a bitbang driver does not
// do — retrying a transfer that answered WAIT, pipelining AP reads and
// collecting them from RDBUFF, match-value polling, block transfers — happens
// here, once, and cmsis/server reaches the same code by holding one of these
// rather than by reimplementing it over byte buffers.
package bitbang

import (
	"context"
	"fmt"
	"time"

	"github.com/kvth/swd-to-go"
)

// SWD request bits, as they go on the wire. Bits 2 and 3 carry the register
// address, which is why a register offset masks straight in.
const (
	reqAPnDP = 1 << 0
	reqRnW   = 1 << 1
	reqAddr  = 0xc

	// regRDBUFF is the DP register that hands back the result of a posted AP
	// read. Reading it is how a pipelined read is drained.
	regRDBUFF  = 0x0c
	rdbuffRead = regRDBUFF | reqRnW
)

// timestampHz is the resolution of the clock [swd.Request.Timestamp] samples,
// matching what CMSIS-DAP reports through DAP_ID_TIMESTAMP_CLOCK.
const timestampHz = 1_000_000

// DefaultMaxBlockSize is how many words one block transfer carries by default.
// Nothing on this side of the wire imposes a limit, so the only thing it trades
// off is how long a single call occupies the bus before the caller gets a look
// in.
const DefaultMaxBlockSize = 1024

// Probe is a [swd.Probe] driven by bit banging.
//
// It is not safe for concurrent use. The posted-read pipeline is a state
// machine spanning several transfers, and a second caller landing in the middle
// of one collects the first caller's data.
type Probe struct {
	d swd.BitBanger

	cfg       swd.Config
	matchMask uint32

	start     time.Time
	timestamp uint32

	maxBlock int
}

// Compile-time proof that a bitbang probe is a probe. It is not a [swd.Cmsis]:
// there is no CMSIS-DAP on the other end of a GPIO header to ask.
var _ swd.Probe = (*Probe)(nil)

// New wraps a pin-level driver. The driver's defaults stand until
// [Probe.Configure] is called; the retry count starts at 100, which is what
// CMSIS-DAP firmware uses.
func New(d swd.BitBanger) *Probe {
	return &Probe{
		d: d,
		cfg: swd.Config{
			WaitRetry:  100,
			Turnaround: 1,
		},
		start:    time.Now(),
		maxBlock: DefaultMaxBlockSize,
	}
}

// SetMaxBlockSize overrides [DefaultMaxBlockSize].
func (p *Probe) SetMaxBlockSize(n int) {
	if n > 0 {
		p.maxBlock = n
	}
}

func (p *Probe) Connect(ctx context.Context) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	p.d.PortOn()
	return nil
}

func (p *Probe) Disconnect(ctx context.Context) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	p.d.PortOff()
	return nil
}

func (p *Probe) SetClock(ctx context.Context, hz uint32) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if hz == 0 {
		return fmt.Errorf("bitbang: clock frequency must not be zero")
	}
	p.d.SetFrequencyHz(hz)
	return nil
}

func (p *Probe) Configure(ctx context.Context, cfg swd.Config) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	turnaround := cfg.Turnaround
	if turnaround == 0 {
		turnaround = 1
	}
	if turnaround > 4 {
		return fmt.Errorf("bitbang: turnaround must be 1..4, got %d", turnaround)
	}
	p.cfg = cfg
	p.cfg.Turnaround = turnaround
	p.d.SwdConfigure(turnaround, cfg.DataPhase)
	return nil
}

func (p *Probe) MaxBlockSize() int { return p.maxBlock }

func (p *Probe) Close(ctx context.Context) error {
	_ = ctx
	return p.d.Close()
}

// ---------------- Sequences ----------------

func (p *Probe) Sequence(ctx context.Context, numBits int, data []byte) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if numBits < 1 || numBits > 256 {
		return fmt.Errorf("bitbang: sequence length must be 1..256, got %d", numBits)
	}
	if need := (numBits + 7) / 8; len(data) < need {
		return fmt.Errorf("bitbang: sequence needs %d data bytes for %d bits, got %d", need, numBits, len(data))
	}
	p.d.SwjSequence(uint32(numBits), data)
	return nil
}

func (p *Probe) SWDSequence(ctx context.Context, seqs []swd.Sequence) ([]byte, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(seqs) == 0 {
		return nil, fmt.Errorf("bitbang: at least one sequence is required")
	}

	var out []byte
	for i, s := range seqs {
		if s.NumCycles < 1 || s.NumCycles > 64 {
			return nil, fmt.Errorf("bitbang: sequence %d: NumCycles must be 1..64, got %d", i, s.NumCycles)
		}
		n := uint32(s.NumCycles)
		byteCount := int((n + 7) / 8)

		if s.Input {
			p.d.SwdioOutDisable()
			buf := make([]byte, byteCount)
			p.d.SwdReadSequence(n, buf)
			out = append(out, buf...)
			continue
		}

		if len(s.Data) < byteCount {
			return nil, fmt.Errorf("bitbang: sequence %d: need %d data bytes for %d cycles, got %d",
				i, byteCount, s.NumCycles, len(s.Data))
		}
		p.d.SwdioOutEnable()
		p.d.SwdWriteSequence(n, s.Data)
	}

	// Leave the host driving the line, which is where a transfer expects to
	// find it.
	p.d.SwdioOutEnable()
	return out, nil
}

// ---------------- Transfers ----------------

// Transfer is the posted-read state machine.
//
// An AP read does not answer with its own data: the value arrives on the next
// transfer, or from RDBUFF if there is no next transfer to carry it. So a read
// request may issue no wire transfer at all (it just posts), and the request
// after it may yield two words (the posted one and its own). Callers see none
// of that — they get one word per read, in order.
func (p *Probe) Transfer(ctx context.Context, reqs []swd.Request) (swd.Status, []uint32, error) {
	if err := ctx.Err(); err != nil {
		return 0, nil, err
	}

	var (
		status     swd.Status
		values     []uint32
		postRead   bool
		checkWrite bool
		data       uint32
		completed  int
	)

loop:
	for _, r := range reqs {
		raw, err := rawRequest(r)
		if err != nil {
			return 0, nil, err
		}

		if r.Op == swd.OpRead || r.Op == swd.OpReadMatch {
			if postRead {
				// An AP read that is not a match read can collect the pending
				// value and post itself in the same transfer. Anything else has
				// to drain RDBUFF first.
				if r.AP && r.Op == swd.OpRead {
					status = p.transfer(raw, &data)
				} else {
					status = p.transfer(rdbuffRead, &data)
					postRead = false
				}
				if !status.Ok() {
					break loop
				}
				values = append(values, data)
				if postRead && r.Timestamp {
					values = append(values, p.timestamp)
				}
			}

			switch {
			case r.Op == swd.OpReadMatch:
				if status = p.matchRead(raw, r.AP, r.Data, &data); !status.Ok() {
					break loop
				}

			case r.AP:
				// Post the read. Its value is collected by whatever comes next,
				// or by the RDBUFF drain at the end.
				if !postRead {
					if status = p.transfer(raw, nil); !status.Ok() {
						break loop
					}
					if r.Timestamp {
						values = append(values, p.timestamp)
					}
					postRead = true
				}

			default:
				// A DP read answers immediately.
				if status = p.transfer(raw, &data); !status.Ok() {
					break loop
				}
				if r.Timestamp {
					values = append(values, p.timestamp)
				}
				values = append(values, data)
			}
			checkWrite = false
		} else {
			if postRead {
				if status = p.transfer(rdbuffRead, &data); !status.Ok() {
					break loop
				}
				values = append(values, data)
				postRead = false
			}

			if r.Op == swd.OpWriteMatch {
				// This sets the mask for later match reads and touches no wire.
				p.matchMask = r.Data
				status = swd.StatusOK
			} else {
				data = r.Data
				if status = p.transfer(raw, &data); !status.Ok() {
					break loop
				}
				if r.Timestamp {
					values = append(values, p.timestamp)
				}
				// A write is only known to have landed once a following read
				// comes back OK, so remember to check at the end.
				checkWrite = true
			}
		}

		completed++
	}

	if status.Ok() {
		switch {
		case postRead:
			if status = p.transfer(rdbuffRead, &data); status.Ok() {
				values = append(values, data)
			}
		case checkWrite:
			status = p.transfer(rdbuffRead, nil)
		}
	}

	if !status.Ok() || completed != len(reqs) {
		return status, values, &swd.TransferError{Status: status, Completed: completed, Total: len(reqs)}
	}
	return status, values, nil
}

// matchRead polls one register until the masked value matches, or the retries
// run out. It produces no value of its own: the caller learns what happened
// from [swd.Status.ValueMismatch].
func (p *Probe) matchRead(raw uint8, ap bool, want uint32, data *uint32) swd.Status {
	if ap {
		// Post it; the value shows up on the next transfer.
		if status := p.transfer(raw, nil); !status.Ok() {
			return status
		}
	}

	var status swd.Status
	for retry := uint32(p.cfg.MatchRetry); ; retry-- {
		if status = p.transfer(raw, data); !status.Ok() {
			return status
		}
		if (*data&p.matchMask) == want || retry == 0 {
			break
		}
	}

	if (*data & p.matchMask) != want {
		status |= swd.StatusMismatch
	}
	return status
}

// transfer runs one wire transfer, repeating it while the target answers WAIT.
func (p *Probe) transfer(raw uint8, data *uint32) swd.Status {
	for retry := uint32(p.cfg.WaitRetry); ; retry-- {
		status := swd.Status(p.d.SwdTransfer(raw, data, p.markTimestamp))
		if status != swd.StatusWait || retry == 0 {
			return status
		}
	}
}

func (p *Probe) markTimestamp() {
	p.timestamp = uint32(time.Since(p.start).Nanoseconds() / int64(time.Second/timestampHz))
}

// ---------------- Block transfers ----------------

func (p *Probe) BlockRead(ctx context.Context, ap bool, reg uint8, n int) ([]uint32, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if n == 0 {
		return nil, nil
	}
	if n < 0 || n > p.maxBlock {
		return nil, fmt.Errorf("bitbang: block read of %d words, max %d", n, p.maxBlock)
	}
	raw, err := rawAccess(ap, reg, true)
	if err != nil {
		return nil, err
	}

	if ap {
		// Post the first read so the pipeline is primed.
		if status := p.transfer(raw, nil); !status.Ok() {
			return nil, &swd.TransferError{Status: status, Total: n}
		}
	}

	out := make([]uint32, 0, n)
	for i := 0; i < n; i++ {
		req := raw
		if ap && i == n-1 {
			// The pipeline runs one behind, so the last word comes from RDBUFF
			// rather than from one more AP read that nobody would collect.
			req = rdbuffRead
		}
		var data uint32
		if status := p.transfer(req, &data); !status.Ok() {
			return out, &swd.TransferError{Status: status, Completed: i, Total: n}
		}
		out = append(out, data)
	}
	return out, nil
}

func (p *Probe) BlockWrite(ctx context.Context, ap bool, reg uint8, data []uint32) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if len(data) == 0 {
		return nil
	}
	if len(data) > p.maxBlock {
		return fmt.Errorf("bitbang: block write of %d words, max %d", len(data), p.maxBlock)
	}
	raw, err := rawAccess(ap, reg, false)
	if err != nil {
		return err
	}

	for i, w := range data {
		v := w
		if status := p.transfer(raw, &v); !status.Ok() {
			return &swd.TransferError{Status: status, Completed: i, Total: len(data)}
		}
	}

	// One read to confirm the last write actually landed.
	if status := p.transfer(rdbuffRead, nil); !status.Ok() {
		return &swd.TransferError{Status: status, Completed: len(data), Total: len(data)}
	}
	return nil
}

// ---------------- Pins ----------------

func (p *Probe) ReadPins(ctx context.Context) (swd.Pin, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return p.readPins(), nil
}

// WritePins drives the selected pins and then waits for them to read back as
// asked, which is how DAP_SWJ_Pins lets a caller confirm a reset actually took.
//
// Only SWCLK and SWDIO are wired: this hardware has no TDI, TDO, nTRST or
// nRESET, so those read as zero and writes to them go nowhere. A caller that
// waits on one of them gets its timeout, which is the truthful answer.
func (p *Probe) WritePins(ctx context.Context, value, mask swd.Pin, timeout time.Duration) (swd.Pin, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}

	if mask&swd.PinSWCLK != 0 {
		if value&swd.PinSWCLK != 0 {
			p.d.SwclkSet()
		} else {
			p.d.SwclkClr()
		}
	}
	if mask&swd.PinSWDIO != 0 {
		if value&swd.PinSWDIO != 0 {
			p.d.SwdioSet()
		} else {
			p.d.SwdioClr()
		}
	}

	if timeout > 0 {
		const maxWait = 3 * time.Second
		if timeout > maxWait {
			timeout = maxWait
		}
		deadline := time.Now().Add(timeout)
		for time.Now().Before(deadline) {
			if p.readPins()&mask == value&mask {
				break
			}
		}
	}

	return p.readPins(), nil
}

// ResetTarget has no device-specific sequence to run: a bitbang probe only
// has the wire, not a DAP on the other end of it. Callers get a plain nRESET
// pulse through [Probe.WritePins] instead.
func (p *Probe) ResetTarget(ctx context.Context) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	return swd.ErrNotImplemented
}

func (p *Probe) readPins() swd.Pin {
	var pins swd.Pin
	if p.d.SwclkIn() != 0 {
		pins |= swd.PinSWCLK
	}
	if p.d.SwdioIn() != 0 {
		pins |= swd.PinSWDIO
	}
	return pins
}

// ---------------- Request encoding ----------------

func rawRequest(r swd.Request) (uint8, error) {
	raw, err := rawAccess(r.AP, r.Reg, r.Op == swd.OpRead || r.Op == swd.OpReadMatch)
	if err != nil {
		return 0, err
	}
	return raw, nil
}

func rawAccess(ap bool, reg uint8, read bool) (uint8, error) {
	if reg&^reqAddr != 0 {
		return 0, fmt.Errorf("bitbang: register 0x%02x is not one of 0x0, 0x4, 0x8, 0xc", reg)
	}
	raw := reg & reqAddr
	if ap {
		raw |= reqAPnDP
	}
	if read {
		raw |= reqRnW
	}
	return raw, nil
}
