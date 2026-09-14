package simtarget

import (
	"encoding/binary"
	"sync"
)

// A simulated RP2040: SRAM, a bootrom holding the function table the flashing
// commands look up, an XIP-mapped flash array and the ARMv6-M debug registers.
//
// There is no CPU. The only code the flashing commands ever set running is the
// bootrom's debug trampoline, so "resume" here means "carry out the routine
// named in r7, then halt at the breakpoint the trampoline ends with" -- which is
// exactly the contract those commands rely on.

// RP2040 memory map, as far as this models it.
const (
	BootROMBase uint32 = 0x00000000
	BootROMSize uint32 = 16 * 1024

	SRAMBase uint32 = 0x20000000
	SRAMSize uint32 = 0x42000 // through the top of SRAM5, where the ROM stack goes

	FlashXIPBase uint32 = 0x10000000

	debugBase uint32 = 0xE000E000
	debugSize uint32 = 0x1000
)

// Bootrom entry points. The addresses are arbitrary; what matters is that the
// function table names them and that a call lands on one.
const (
	romTableAddr uint32 = 0x0100

	romTrampoline    uint32 = 0x0200
	romTrampolineEnd uint32 = 0x0210

	romConnectInternalFlash uint32 = 0x0300
	romFlashExitXIP         uint32 = 0x0310
	romFlashRangeErase      uint32 = 0x0320
	romFlashRangeProgram    uint32 = 0x0330
	romFlashFlushCache      uint32 = 0x0340
	romFlashEnterCmdXIP     uint32 = 0x0350
)

// ARMv6-M debug registers.
const (
	regAIRCR uint32 = 0xE000ED0C
	regDHCSR uint32 = 0xE000EDF0
	regDCRSR uint32 = 0xE000EDF4
	regDCRDR uint32 = 0xE000EDF8
	regDEMCR uint32 = 0xE000EDFC
)

const (
	dhcsrDbgKey    uint32 = 0xA05F << 16
	dhcsrCDebugEn  uint32 = 1 << 0
	dhcsrCHalt     uint32 = 1 << 1
	dhcsrCMaskInts uint32 = 1 << 3
	dhcsrSRegRdy   uint32 = 1 << 16
	dhcsrSHalt     uint32 = 1 << 17
	dhcsrSLockup   uint32 = 1 << 19
	dhcsrSResetSt  uint32 = 1 << 25

	dcrsrWrite uint32 = 1 << 16
	dcrsrMask  uint32 = 0x1F

	demcrVCCoreReset uint32 = 1 << 0

	aircrVectKey     uint32 = 0x05FA << 16
	aircrSysResetReq uint32 = 1 << 2
)

// RP2040 is the simulated chip: the debug registers as an MMIO device, plus the
// flash array behind the XIP window.
type RP2040 struct {
	mu sync.Mutex

	target *Target
	xip    *Region
	// sram is read directly rather than through the target, because a ROM call
	// runs from inside a transfer and the target's lock is already held.
	sram *Region

	// Flash is the simulated QSPI part's contents, and what the XIP window maps
	// when it is mapped at all.
	Flash []byte

	// Core state. There is no CPU, so these are only what the debug registers
	// read and write and what a ROM call takes its arguments from.
	regs   [18]uint32
	halted bool
	// lockup makes the next ROM call halt somewhere other than the
	// trampoline's breakpoint, for the "the routine did not run to completion"
	// path.
	lockup bool

	demcr   uint32
	resetSt bool // sticky S_RESET_ST, cleared by the read that sees it

	// Calls counts the ROM routines that actually ran, so a test can check that
	// a flashing session is not paying one call per packet.
	Calls int
	// XIPMapped is whether flash is in the memory map, which it is not between
	// flash_exit_xip and flash_enter_cmd_xip.
	XIPMapped bool

	// The two halves of a core register access: the value goes through DCRDR
	// and the register number through DCRSR.
	dcrdr    uint32
	selected uint32
}

// Core register indices, as DCRSR names them.
const (
	coreR0   = 0
	coreR7   = 7
	coreSP   = 13
	coreLR   = 14
	corePC   = 15
	coreXPSR = 16
	coreMSP  = 17
)

// NewRP2040 builds a target that looks enough like an RP2040 for the flashing
// vendor commands: bootrom, SRAM, XIP flash and the debug registers.
func NewRP2040(opts Options, flashSize int) (*Target, *RP2040) {
	bootrom := &Region{Base: BootROMBase, Data: make([]byte, BootROMSize), ReadOnly: true}
	sram := &Region{Base: SRAMBase, Data: make([]byte, SRAMSize)}

	flash := make([]byte, flashSize)
	for i := range flash {
		flash[i] = 0xFF // erased
	}
	xip := &Region{Base: FlashXIPBase, Data: flash}

	target := New(opts, bootrom, sram, xip)

	chip := &RP2040{
		target:    target,
		xip:       xip,
		sram:      sram,
		Flash:     flash,
		halted:    false,
		XIPMapped: true,
	}
	chip.layOutBootROM(bootrom.Data)
	target.AddMMIO(debugBase, debugSize, chip)

	return target, chip
}

// layOutBootROM writes the magic and the function table the flashing commands
// walk. A table entry is a two-character tag and a 16-bit address, and a zero
// tag ends it.
func (c *RP2040) layOutBootROM(rom []byte) {
	// 'M', 'u', then a version byte.
	binary.LittleEndian.PutUint32(rom[0x10:], 0x02_01754D)
	binary.LittleEndian.PutUint16(rom[0x14:], uint16(romTableAddr))

	entries := []struct {
		tag  string
		addr uint32
	}{
		{"DT", romTrampoline | 1}, // the Thumb bit is on in the table and masked off by the lookup
		{"DE", romTrampolineEnd | 1},
		{"IF", romConnectInternalFlash | 1},
		{"EX", romFlashExitXIP | 1},
		{"RE", romFlashRangeErase | 1},
		{"RP", romFlashRangeProgram | 1},
		{"FC", romFlashFlushCache | 1},
		{"CX", romFlashEnterCmdXIP | 1},
	}

	at := romTableAddr
	for _, e := range entries {
		rom[at] = e.tag[0]
		rom[at+1] = e.tag[1]
		binary.LittleEndian.PutUint16(rom[at+2:], uint16(e.addr))
		at += 4
	}
	// A zero tag ends the table.
	binary.LittleEndian.PutUint32(rom[at:], 0)
}

// Halted reports whether the core is stopped.
func (c *RP2040) Halted() bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.halted
}

// SetLockup makes the next ROM call halt away from the trampoline's own
// breakpoint, the way a fault or a stray breakpoint in the application would.
func (c *RP2040) SetLockup(v bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.lockup = v
}

// --- MMIO ------------------------------------------------------------------

// ReadWord answers a debug register read.
func (c *RP2040) ReadWord(addr uint32) (uint32, bool) {
	c.mu.Lock()
	defer c.mu.Unlock()

	switch addr {
	case regDHCSR:
		var v uint32
		if c.halted {
			v |= dhcsrSHalt
		}
		v |= dhcsrSRegRdy // a core register access settles in a few core clocks
		if c.resetSt {
			// Sticky, and cleared by the read that sees it -- which is how a
			// client tells "still in reset" from "just out of it".
			v |= dhcsrSResetSt
			c.resetSt = false
		}
		return v, true

	case regDCRSR:
		return 0, true

	case regDCRDR:
		return c.dcrdr, true

	case regDEMCR:
		return c.demcr, true

	case regAIRCR:
		return aircrVectKey, true
	}

	// Everything else in the debug block reads zero rather than faulting: the
	// commands under test only touch the registers above.
	return 0, true
}

// WriteWord answers a debug register write.
func (c *RP2040) WriteWord(addr uint32, value uint32) bool {
	c.mu.Lock()
	defer c.mu.Unlock()

	switch addr {
	case regDHCSR:
		if value&0xFFFF0000 != dhcsrDbgKey {
			return true // the wrong key: the write is ignored, as on real silicon
		}
		if value&dhcsrCDebugEn == 0 {
			return true
		}
		if value&dhcsrCHalt != 0 {
			c.halted = true
			return true
		}
		// C_HALT clear with debug enabled is a resume. What happens next is
		// whatever PC points at, and the only code these commands ever set
		// running is the bootrom's debug trampoline -- so a resume from there
		// carries out the routine in r7 and stops at the breakpoint the
		// trampoline ends with, and a resume from anywhere else just runs.
		if c.halted {
			if c.regs[corePC]&^1 == romTrampoline {
				c.runTrampoline()
			} else {
				c.halted = false
			}
		}
		return true

	case regDCRSR:
		reg := value & dcrsrMask
		if int(reg) >= len(c.regs) {
			return true
		}
		c.selected = reg
		if value&dcrsrWrite != 0 {
			c.regs[reg] = c.dcrdr
		} else {
			c.dcrdr = c.regs[reg]
		}
		return true

	case regDCRDR:
		c.dcrdr = value
		// A pending write takes its value from here, so keep the selected
		// register in step for the write-then-select order the commands use.
		return true

	case regDEMCR:
		c.demcr = value
		return true

	case regAIRCR:
		if value&0xFFFF0000 == aircrVectKey && value&aircrSysResetReq != 0 {
			c.reset()
		}
		return true
	}

	return true
}

func (c *RP2040) reset() {
	c.resetSt = true
	c.regs = [18]uint32{}
	c.lockup = false
	// A reset takes the QSPI interface back to its power-on state, so flash is
	// memory mapped again whatever a half-finished session left behind.
	c.XIPMapped = true
	c.xip.Unmapped = false
	// With the vector catch armed the core stops at the reset vector; without
	// it, it runs.
	c.halted = c.demcr&demcrVCCoreReset != 0
}

// runTrampoline carries out the routine named in r7 and halts at the
// trampoline's breakpoint, which is what "the call returned" means to a client.
func (c *RP2040) runTrampoline() {
	c.Calls++

	fn := c.regs[coreR7] &^ 1
	r0, r1, r2, r3 := c.regs[0], c.regs[1], c.regs[2], c.regs[3]

	switch fn {
	case romConnectInternalFlash, romFlashFlushCache:
		// Nothing to model: these only touch the QSPI interface.

	case romFlashExitXIP:
		c.XIPMapped = false
		c.xip.Unmapped = true

	case romFlashEnterCmdXIP:
		c.XIPMapped = true
		c.xip.Unmapped = false

	case romFlashRangeErase:
		c.eraseFlash(r0, r1)
		_ = r2 // block_size
		_ = r3 // block_cmd

	case romFlashRangeProgram:
		c.programFlash(r0, r1, r2)
	}

	c.regs[corePC] = romTrampolineEnd
	if c.lockup {
		// Somewhere that is not the breakpoint: the routine did not run to
		// completion and r0 is not a result.
		c.regs[corePC] = romTrampoline + 4
	}
	c.regs[coreR0] = 0
	c.halted = true
}

func (c *RP2040) eraseFlash(addr, count uint32) {
	if uint64(addr)+uint64(count) > uint64(len(c.Flash)) {
		return
	}
	for i := addr; i < addr+count; i++ {
		c.Flash[i] = 0xFF
	}
}

func (c *RP2040) programFlash(addr, src, count uint32) {
	if uint64(addr)+uint64(count) > uint64(len(c.Flash)) {
		return
	}
	// The source is target RAM, which the staging command filled over the wire.
	// This runs inside a transfer, so the target's lock is already held and the
	// region has to be reached directly.
	if src < c.sram.Base || uint64(src)+uint64(count) > uint64(c.sram.Base)+uint64(len(c.sram.Data)) {
		return
	}
	data := c.sram.Data[src-c.sram.Base : src-c.sram.Base+count]
	// Flash programming can only clear bits, which is why an erase has to come
	// first; modelling that is what makes a missing erase show up as corruption
	// rather than as a clean write.
	for i := range data {
		c.Flash[addr+uint32(i)] &= data[i]
	}
}
