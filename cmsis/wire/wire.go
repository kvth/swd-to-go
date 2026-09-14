// Package wire holds the CMSIS-DAP protocol constants.
//
// It is shared by cmsis/client and cmsis/server so the two halves cannot drift
// apart, and it deliberately contains nothing else: no encoding, no state, no
// dependency on swd. The protocol is documented at
// https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__Commands__gr.html
package wire

// Command IDs.
const (
	CmdInfo              = 0x00
	CmdHostStatus        = 0x01
	CmdConnect           = 0x02
	CmdDisconnect        = 0x03
	CmdTransferConfigure = 0x04
	CmdTransfer          = 0x05
	CmdTransferBlock     = 0x06
	CmdTransferAbort     = 0x07
	CmdWriteABORT        = 0x08
	CmdDelay             = 0x09
	CmdResetTarget       = 0x0a
	CmdSWJPins           = 0x10
	CmdSWJClock          = 0x11
	CmdSWJSequence       = 0x12
	CmdSWDConfigure      = 0x13
	CmdJTAGSequence      = 0x14
	CmdJTAGConfigure     = 0x15
	CmdJTAGIDCode        = 0x16
	CmdSWDSequence       = 0x1d

	CmdQueueCommands   = 0x7e
	CmdExecuteCommands = 0x7f

	// The vendor range is a particular probe's business. Nothing in this
	// module implements or interprets one.
	CmdVendor0  = 0x80
	CmdVendor31 = 0x9f

	CmdInvalid = 0xff
)

// Status bytes a command replies with.
const (
	StatusOK    = 0x00
	StatusError = 0xff
)

// DAP_Info IDs.
const (
	InfoVendor         = 0x01
	InfoProduct        = 0x02
	InfoSerialNumber   = 0x03
	InfoFirmwareVer    = 0x04
	InfoDeviceVendor   = 0x05
	InfoDeviceName     = 0x06
	InfoBoardVendor    = 0x07
	InfoBoardName      = 0x08
	InfoProductFWVer   = 0x09
	InfoCapabilities   = 0xf0
	InfoTimestampClock = 0xf1
	InfoPacketCount    = 0xfe
	InfoPacketSize     = 0xff
)

// DAP_ID_CAPABILITIES bits, first byte.
const (
	CapSWD            = 1 << 0
	CapJTAG           = 1 << 1
	CapSWOUART        = 1 << 2
	CapSWOManchester  = 1 << 3
	CapAtomicCommands = 1 << 4
	CapTimestampClock = 1 << 5
	CapSWOStream      = 1 << 6
	CapUART           = 1 << 7
)

// DAP_HostStatus types.
const (
	HostStatusConnected = 0
	HostStatusRunning   = 1
)

// Debug port selectors for DAP_Connect.
const (
	PortAutodetect = 0
	PortDisabled   = 0
	PortSWD        = 1
	PortJTAG       = 2
)

// DAP_SWJ_Pins bit positions.
const (
	SWJPinSWCLK  = 0
	SWJPinSWDIO  = 1
	SWJPinTDI    = 2
	SWJPinTDO    = 3
	SWJPinNTRST  = 5
	SWJPinNRESET = 7
)

// Transfer request bits.
const (
	TransferAPnDP      = 1 << 0
	TransferRnW        = 1 << 1
	TransferA2         = 1 << 2
	TransferA3         = 1 << 3
	TransferMatchValue = 1 << 4
	TransferMatchMask  = 1 << 5
	TransferTimestamp  = 1 << 7

	// TransferAddr masks out the two register-address bits.
	TransferAddr = TransferA2 | TransferA3
)

// Transfer response bits. The low three are the raw SWD ACK.
const (
	TransferOK       = 1 << 0
	TransferWait     = 1 << 1
	TransferFault    = 1 << 2
	TransferError    = 1 << 3
	TransferMismatch = 1 << 4
)

// Sequence info bits, shared by DAP_SWD_Sequence and DAP_JTAG_Sequence. A
// cycle count of zero means 64.
const (
	SeqClockMask = 0x3f
	SeqDataIn    = 0x80
)

// TimestampHz is the resolution reported through DAP_ID_TIMESTAMP_CLOCK.
const TimestampHz = 1_000_000
