package swd

// BitBanger is the pin-level SWD driver that every backend under drivers/
// implements.
//
// It knows about bits, turnarounds and a single ACK per transfer. It knows
// nothing about WAIT retries, posted AP reads, RDBUFF or match masks —
// probe/bitbang adds all of that and turns a BitBanger into a [Probe]. Keeping
// the split here means a new piece of GPIO hardware costs one file of pin
// wiggling and nothing else.
//
// None of these methods take a context or return an error: they are register
// pokes and busy loops, they cannot fail, and at 1.5 MHz the whole of a
// transfer is shorter than the check would be.
type BitBanger interface {
	// PortOn drives SWCLK low and SWDIO high, taking ownership of the lines.
	PortOn()
	// PortOff floats both lines.
	PortOff()

	// SetFrequencyHz sets the target SWCLK frequency. Backends approximate.
	SetFrequencyHz(hz uint32)
	// SwdConfigure sets the bus turnaround in SWCLK cycles (1 to 4) and
	// whether the data phase is clocked out on WAIT and FAULT responses.
	SwdConfigure(turnaround uint8, dataPhase bool)

	// SwjSequence clocks bitCount raw bits from data, LSB-first, with SWDIO
	// driven throughout. This is the switching-sequence primitive.
	SwjSequence(bitCount uint32, data []byte)
	// SwdWriteSequence clocks bitCount bits from data out of SWDIO.
	SwdWriteSequence(bitCount uint32, data []byte)
	// SwdReadSequence clocks bitCount bits in and packs them into data.
	SwdReadSequence(bitCount uint32, data []byte)

	// SwdTransfer runs one SWD transfer and returns the raw ACK, or the
	// protocol-error value if none came back. req is the packed SWD request:
	// bit 0 APnDP, bit 1 RnW, bits 2-3 the register address.
	//
	// data is the value to write, or where to put the value read; it may be nil
	// for a read whose result is being discarded. timestamp, when non-nil, is
	// called the moment the transfer completes.
	SwdTransfer(req uint8, data *uint32, timestamp func()) uint8

	// SwdioOutEnable makes the host drive SWDIO; SwdioOutDisable releases it so
	// the target can. Sequence callers have to bracket their own direction
	// changes; SwdTransfer handles its own.
	SwdioOutEnable()
	SwdioOutDisable()

	// The raw pin accessors, which exist for DAP_SWJ_Pins and for nothing else.
	SwclkSet()
	SwclkClr()
	SwclkIn() uint8
	SwdioSet()
	SwdioClr()
	SwdioIn() uint8

	// Close releases the underlying GPIO handle.
	Close() error
}
