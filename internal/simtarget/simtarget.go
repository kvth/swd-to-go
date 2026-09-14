// Package simtarget is a simulated SWD debug port and MEM-AP over ordinary
// memory, standing in for a bitbanged wire and a board.
//
// It implements [swd.BitBanger], so a real bitbang.Probe sits on top of it and
// everything above the bitbang is the production code: the DP/AP shadow, the
// probe-side memory bursts, and the whole vendor command block. What is
// simulated is only the target's side of the wire -- the register file a
// transfer lands in, and the memory behind the MEM-AP.
//
// The posted-read behaviour is the part worth being careful about, because the
// burst code depends on it: an AP read starts an access and returns the
// *previous* one's result, and DP RDBUFF returns the last. That is what lets a
// run of words cost one transfer each instead of two.
package simtarget

import (
	"encoding/binary"
	"sync"

	"github.com/kvth/swd-to-go"
)

// Transfer request bits and acks, as CMSIS-DAP encodes them.
const (
	reqAPnDP = 1 << 0
	reqRnW   = 1 << 1
	reqAddr  = 0x0C

	ackOK    = 1 << 0
	ackWait  = 1 << 1
	ackFault = 1 << 2
)

// DP registers, by A[3:2].
const (
	dpIDR      = 0x00 // read
	dpAbort    = 0x00 // write
	dpCtrlStat = 0x04
	dpSelect   = 0x08 // write
	dpRDBUFF   = 0x0C // read
)

// MEM-AP registers within bank 0.
const (
	apCSW  = 0x00
	apTAR  = 0x04
	apDRW  = 0x0C
	apIDR  = 0x0C // bank 0xF
	apBASE = 0x08 // bank 0xF
)

// CTRL/STAT bits.
const (
	ctrlStickyOrun = 1 << 1
	ctrlStickyCmp  = 1 << 4
	ctrlStickyErr  = 1 << 5
	ctrlWDataErr   = 1 << 7
	ctrlStickyAny  = ctrlStickyOrun | ctrlStickyCmp | ctrlStickyErr | ctrlWDataErr

	ctrlCDBGPWRUPREQ = 1 << 28
	ctrlCDBGPWRUPACK = 1 << 29
	ctrlCSYSPWRUPREQ = 1 << 30
	ctrlCSYSPWRUPACK = 1 << 31
)

// ABORT bits.
const (
	abortStkCmpClr  = 1 << 1
	abortStkErrClr  = 1 << 2
	abortWDErrClr   = 1 << 3
	abortOrunErrClr = 1 << 4
)

// CSW fields.
const (
	cswSizeMask      = 0x7
	cswSizeByte      = 0x0
	cswSizeHalf      = 0x1
	cswSizeWord      = 0x2
	cswAddrIncMask   = 0x3 << 4
	cswAddrIncSingle = 0x1 << 4
	cswAddrIncPacked = 0x2 << 4
	cswDeviceEn      = 1 << 6

	tarAutoIncRegion = 1024
)

// Region is a span of target memory the MEM-AP can reach. Anything outside
// every region faults, which is what a control block search walking a whole RAM
// window runs into.
type Region struct {
	Base uint32
	Data []byte
	// ReadOnly makes writes fault, for a bootrom or an XIP window that is not
	// mapped for writing.
	ReadOnly bool
	// Unmapped takes the region out of the map without forgetting its contents,
	// for an XIP window a flash session has switched off.
	Unmapped bool
}

// MMIO is a simulated peripheral behind the MEM-AP, for the memory-mapped
// registers a command drives rather than a block of RAM: the ARMv6-M debug
// registers an RP2040 flashing session halts the core through, say. Both calls
// report false for an address the device does not answer for, which faults the
// transfer the same way unmapped memory does.
type MMIO interface {
	ReadWord(addr uint32) (uint32, bool)
	WriteWord(addr uint32, value uint32) bool
}

type mmioRange struct {
	base, size uint32
	dev        MMIO
}

// Options configure what the wire looks like.
type Options struct {
	// DPIDR is what a DPIDR read answers.
	DPIDR uint32

	// Multidrop models an SWD multi-drop wire, as an RP2040 has: after a line
	// reset no DP answers until a TARGETSEL write picks one, and the DP that
	// write picks answers nothing until the host reads DPIDR, as ADIv5.2
	// requires. That second rule is an easy step for a client to leave out and
	// an obscure failure to debug without it.
	Multidrop bool
	// TargetSel is which DP the multi-drop wire answers for.
	TargetSel uint32

	// NoPower never acknowledges the power-up request, to exercise a bring-up
	// that gives up.
	NoPower bool

	// PowerLatched keeps the power-up acknowledges set once they have been
	// granted, even after the request is taken away again. That is what a
	// debug port still holding its chip in reset looks like, and so what an
	// RP2040 rescue that did not take looks like from the host.
	PowerLatched bool

	// NoByteAccess makes the MEM-AP refuse byte-sized accesses, as a
	// word-only AP does: the size field reads back as it was.
	NoByteAccess bool
}

// Target is the simulated debug port, its MEM-AP and the memory behind it.
type Target struct {
	mu sync.Mutex

	opts    Options
	regions []*Region
	mmio    []mmioRange

	// Debug port state.
	selectValue uint32
	ctrlStat    uint32
	posted      uint32 // the value a posted read will hand back
	connected   bool   // PortOn has run
	selected    bool   // a DP is listening (always true off a multi-drop wire)
	sawDPIDR    bool   // ADIv5.2: the selected DP answers nothing until DPIDR is read

	// MEM-AP state, one AP.
	csw uint32
	tar uint32

	// Transfers counts every transfer that reached the target, for a test that
	// wants to assert a command is not paying per-word round trips.
	Transfers int
}

// New builds a target with the given memory regions.
func New(opts Options, regions ...*Region) *Target {
	if opts.DPIDR == 0 {
		opts.DPIDR = 0x0BC12477 // an RP2040's, as good a default as any
	}
	if opts.TargetSel == 0 {
		opts.TargetSel = 0x01002927
	}
	t := &Target{
		opts:     opts,
		regions:  regions,
		csw:      cswDeviceEn | cswSizeWord | cswAddrIncSingle,
		selected: !opts.Multidrop,
		sawDPIDR: !opts.Multidrop,
	}
	return t
}

// Region returns the region with the given base, or nil.
func (t *Target) Region(base uint32) *Region {
	for _, r := range t.regions {
		if r.Base == base {
			return r
		}
	}
	return nil
}

// AddRegion adds a memory region after construction.
func (t *Target) AddRegion(r *Region) {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.regions = append(t.regions, r)
}

// AddMMIO maps a simulated peripheral over [base, base+size). It is consulted
// before the memory regions, so it can shadow one.
func (t *Target) AddMMIO(base, size uint32, dev MMIO) {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.mmio = append(t.mmio, mmioRange{base: base, size: size, dev: dev})
}

// readWord and writeWord are every MEM-AP access, once it has been reduced to
// the word the address is in. MMIO first, then the plain memory regions.
func (t *Target) readWord(addr uint32) (uint32, bool) {
	for _, m := range t.mmio {
		if addr >= m.base && addr < m.base+m.size {
			return m.dev.ReadWord(addr)
		}
	}
	if b := t.find(addr, 4, false); b != nil {
		return binary.LittleEndian.Uint32(b), true
	}
	return 0, false
}

func (t *Target) writeWord(addr uint32, value uint32) bool {
	for _, m := range t.mmio {
		if addr >= m.base && addr < m.base+m.size {
			return m.dev.WriteWord(addr, value)
		}
	}
	if b := t.find(addr, 4, true); b != nil {
		binary.LittleEndian.PutUint32(b, value)
		return true
	}
	return false
}

// --- memory ----------------------------------------------------------------

func (t *Target) find(addr uint32, length uint32, forWrite bool) []byte {
	for _, r := range t.regions {
		if r.Unmapped {
			continue
		}
		if addr < r.Base || uint64(addr)+uint64(length) > uint64(r.Base)+uint64(len(r.Data)) {
			continue
		}
		if forWrite && r.ReadOnly {
			return nil
		}
		off := addr - r.Base
		return r.Data[off : off+length]
	}
	return nil
}

// ReadMem reads target memory directly, bypassing the wire, for a test to check
// what a command left behind.
func (t *Target) ReadMem(addr uint32, length int) []byte {
	t.mu.Lock()
	defer t.mu.Unlock()
	b := t.find(addr, uint32(length), false)
	if b == nil {
		return nil
	}
	out := make([]byte, length)
	copy(out, b)
	return out
}

// WriteMem writes target memory directly, bypassing the wire.
func (t *Target) WriteMem(addr uint32, data []byte) bool {
	t.mu.Lock()
	defer t.mu.Unlock()
	b := t.find(addr, uint32(len(data)), true)
	if b == nil {
		return false
	}
	copy(b, data)
	return true
}

// --- the transfer model ----------------------------------------------------

func (t *Target) fault() uint8 {
	t.ctrlStat |= ctrlStickyErr
	return ackFault
}

// SwdTransfer is one SWD transfer against the simulated DP and AP.
func (t *Target) SwdTransfer(req uint8, data *uint32, requestTimestamp func()) uint8 {
	t.mu.Lock()
	defer t.mu.Unlock()

	t.Transfers++
	if requestTimestamp != nil {
		requestTimestamp()
	}

	if !t.connected || !t.selected {
		return 0 // no acknowledge at all: nothing is driving the line
	}

	isAP := req&reqAPnDP != 0
	isRead := req&reqRnW != 0
	addr := uint32(req) & reqAddr

	// ADIv5.2: a multi-drop DP that has just been selected answers nothing
	// until DPIDR is read.
	if !t.sawDPIDR {
		if isAP || !isRead || addr != dpIDR {
			return 0
		}
	}

	if isAP {
		return t.apTransfer(addr, isRead, data)
	}
	return t.dpTransfer(addr, isRead, data)
}

func (t *Target) dpTransfer(addr uint32, isRead bool, data *uint32) uint8 {
	switch {
	case isRead && addr == dpIDR:
		t.sawDPIDR = true
		if data != nil {
			*data = t.opts.DPIDR
		}
		return ackOK

	case !isRead && addr == dpAbort:
		if data != nil {
			v := *data
			var clear uint32
			if v&abortStkCmpClr != 0 {
				clear |= ctrlStickyCmp
			}
			if v&abortStkErrClr != 0 {
				clear |= ctrlStickyErr
			}
			if v&abortWDErrClr != 0 {
				clear |= ctrlWDataErr
			}
			if v&abortOrunErrClr != 0 {
				clear |= ctrlStickyOrun
			}
			t.ctrlStat &^= clear
		}
		return ackOK

	case addr == dpCtrlStat:
		if isRead {
			if data != nil {
				*data = t.ctrlStat
			}
			return ackOK
		}
		if data != nil {
			// The power-up acknowledges follow their requests, unless this
			// target is set up never to grant them or never to let them go.
			keep := t.ctrlStat & ctrlStickyAny
			if t.opts.PowerLatched {
				keep |= t.ctrlStat & (ctrlCDBGPWRUPACK | ctrlCSYSPWRUPACK)
			}
			t.ctrlStat = keep | *data
			if !t.opts.NoPower {
				if *data&ctrlCDBGPWRUPREQ != 0 {
					t.ctrlStat |= ctrlCDBGPWRUPACK
				}
				if *data&ctrlCSYSPWRUPREQ != 0 {
					t.ctrlStat |= ctrlCSYSPWRUPACK
				}
			}
		}
		return ackOK

	case !isRead && addr == dpSelect:
		if data != nil {
			t.selectValue = *data
		}
		return ackOK

	case isRead && addr == dpRDBUFF:
		if data != nil {
			*data = t.posted
		}
		return ackOK
	}

	return t.fault()
}

func (t *Target) apTransfer(addr uint32, isRead bool, data *uint32) uint8 {
	// A sticky error blocks every AP access until it is cleared. This is what
	// makes a burst that walked off the end of memory have to call
	// ClearSticky before it can carry on.
	if t.ctrlStat&ctrlStickyAny != 0 {
		return ackFault
	}

	bank := (t.selectValue >> 4) & 0xF
	if bank != 0 {
		// Only bank 0 (CSW/TAR/DRW) is modelled; anything else reads zero.
		if isRead {
			t.posted = 0
			return ackOK
		}
		return ackOK
	}

	switch addr {
	case apCSW:
		if isRead {
			t.post(t.csw, data)
			return ackOK
		}
		if data != nil {
			size := *data & cswSizeMask
			if size == cswSizeByte && t.opts.NoByteAccess {
				// The size field is read-only for sizes an AP does not
				// implement, so the write leaves it where it was -- which is
				// exactly how a client discovers the AP cannot do them.
				size = t.csw & cswSizeMask
			}
			t.csw = (*data &^ cswSizeMask) | size | cswDeviceEn
		}
		return ackOK

	case apTAR:
		if isRead {
			t.post(t.tar, data)
			return ackOK
		}
		if data != nil {
			t.tar = *data
		}
		return ackOK

	case apDRW:
		return t.drw(isRead, data)
	}

	return t.fault()
}

// post hands back the previously posted value and queues this one, which is how
// a real AP read behaves.
func (t *Target) post(value uint32, data *uint32) {
	if data != nil {
		*data = t.posted
	}
	t.posted = value
}

func (t *Target) drw(isRead bool, data *uint32) uint8 {
	size := t.csw & cswSizeMask
	var width uint32
	switch size {
	case cswSizeByte:
		width = 1
	case cswSizeHalf:
		width = 2
	default:
		width = 4
	}

	// A word access reads the whole word the address is in; a narrower one sits
	// in its own lane within that word.
	wordAddr := t.tar &^ 3
	lane := 8 * (t.tar & 3)

	if isRead {
		word, ok := t.readWord(wordAddr)
		if !ok {
			return t.fault()
		}
		value := word
		if width != 4 {
			mask := uint32(1)<<(8*width) - 1
			value = ((word >> lane) & mask) << lane
		}
		t.post(value, data)
	} else if data != nil {
		word := *data
		if width != 4 {
			// A narrow write only replaces its own lane, so the rest of the
			// word has to be read back first.
			old, ok := t.readWord(wordAddr)
			if !ok {
				return t.fault()
			}
			mask := (uint32(1)<<(8*width) - 1) << lane
			word = (old &^ mask) | (*data & mask)
		}
		if !t.writeWord(wordAddr, word) {
			return t.fault()
		}
	}

	// Auto-increment wraps inside the 1kB region rather than carrying out of it.
	var inc uint32
	switch t.csw & cswAddrIncMask {
	case cswAddrIncSingle:
		inc = width
	case cswAddrIncPacked:
		inc = 4
	}
	if inc != 0 {
		t.tar = (t.tar &^ (tarAutoIncRegion - 1)) |
			((t.tar + inc) & (tarAutoIncRegion - 1))
	}

	return ackOK
}

// --- the rest of swd.BitBanger ---------------------------------------------

// PortOn connects the wire. On a multi-drop wire nothing answers until a switch
// sequence and a TARGETSEL write pick a DP.
func (t *Target) PortOn() {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.connected = true
}

// PortOff disconnects it.
func (t *Target) PortOff() {
	t.mu.Lock()
	defer t.mu.Unlock()
	t.connected = false
}

func (t *Target) SwdioOutEnable()          {}
func (t *Target) SwdioOutDisable()         {}
func (t *Target) SetFrequencyHz(hz uint32) {}
func (t *Target) SwdConfigure(uint8, bool) {}
func (t *Target) SwclkSet()                {}
func (t *Target) SwclkClr()                {}
func (t *Target) SwdioSet()                {}
func (t *Target) SwdioClr()                {}
func (t *Target) SwclkIn() uint8           { return 0 }
func (t *Target) SwdioIn() uint8           { return 1 }
func (t *Target) SwdWriteSequence(bitCount uint32, data []byte) {
	t.sequence(bitCount, data)
}

// SwdReadSequence is the capture phase of a raw sequence. Nothing here drives
// the line during one -- a multi-drop TARGETSEL write is the only place a client
// uses it, and that is precisely the transfer nobody acknowledges.
func (t *Target) SwdReadSequence(bitCount uint32, data []byte) {
	for i := range data {
		data[i] = 0
	}
}

// SwjSequence is a switch sequence: the only part of one this models is that a
// long run of ones is a line reset, which deselects every DP on a multi-drop
// wire.
func (t *Target) SwjSequence(bitCount uint32, data []byte) {
	t.sequence(bitCount, data)
}

func (t *Target) sequence(bitCount uint32, data []byte) {
	t.mu.Lock()
	defer t.mu.Unlock()

	if !t.opts.Multidrop {
		return
	}

	// A line reset is at least 50 clocks with the line high. Anything carrying
	// one deselects whatever DP was listening.
	if runOfOnes(bitCount, data) >= 50 {
		t.selected = false
		t.sawDPIDR = false
	}

	// A TARGETSEL write is the only 33-bit sequence a client sends: 32 bits of
	// target and a parity bit. Whether this DP answers afterwards is whether it
	// names us.
	if bitCount == 33 && len(data) >= 4 {
		want := binary.LittleEndian.Uint32(data[:4])
		t.selected = want == t.opts.TargetSel
		t.sawDPIDR = false
	}
}

// runOfOnes is the longest run of set bits in the sequence, low bit of each byte
// first, which is the order a sequence goes out in.
func runOfOnes(bitCount uint32, data []byte) uint32 {
	var best, run uint32
	for i := uint32(0); i < bitCount && int(i/8) < len(data); i++ {
		if data[i/8]&(1<<(i%8)) != 0 {
			run++
			if run > best {
				best = run
			}
		} else {
			run = 0
		}
	}
	return best
}

// --- inspection -------------------------------------------------------------

// CSW, TAR and Select are the register file as the target has it, for a test
// checking that a probe-side burst put the client's values back.
func (t *Target) CSW() uint32 {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.csw
}

func (t *Target) TAR() uint32 {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.tar
}

func (t *Target) Select() uint32 {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.selectValue
}

// CtrlStat is DP CTRL/STAT, including the sticky error bits.
func (t *Target) CtrlStat() uint32 {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.ctrlStat
}

// Sticky reports whether any sticky error bit is set.
func (t *Target) Sticky() bool {
	return t.CtrlStat()&ctrlStickyAny != 0
}

// Close satisfies [swd.BitBanger]. There is no hardware behind a simulated
// target, so there is nothing to release.
func (t *Target) Close() error { return nil }

var _ swd.BitBanger = (*Target)(nil)
