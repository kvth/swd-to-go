package swd

import "context"

// Cmsis is the one thing a [Probe] may also implement, and only a probe that
// actually speaks CMSIS-DAP does.
//
// It is kept out of [Probe] because none of it can be faked: a bitbang driver
// has no vendor string, no firmware version, no status LEDs and no vendor
// command range, and a probe that stubbed them out would be lying to its
// caller rather than telling it there is nothing there. Type-assert for this
// and handle the failure.
//
// It is one interface rather than several because the alternative — an
// Identity, a HostStatus and a Vendor, each type-asserted separately — split a
// single capability ("this probe is a CMSIS-DAP probe") into three questions
// that in practice always have the same answer. Anything a probe can plausibly
// answer on its own belongs in [Probe], with [ErrNotImplemented] for the cases
// it cannot — [Probe.ResetTarget] and the pins are there for exactly that
// reason; anything that needs a DAP on the other end belongs here.
type Cmsis interface {
	// The DAP_Info strings.
	VendorID(ctx context.Context) (string, error)
	ProductID(ctx context.Context) (string, error)
	SerialNumber(ctx context.Context) (string, error)
	FirmwareVersion(ctx context.Context) (string, error)
	TargetVendor(ctx context.Context) (string, error)
	TargetName(ctx context.Context) (string, error)

	// SetConnected and SetRunning drive the probe's status LEDs.
	SetConnected(ctx context.Context, on bool) error
	SetRunning(ctx context.Context, on bool) error

	// VendorCommand sends one command in the ID_DAP_Vendor0..31 range and
	// returns the reply with the echoed command ID stripped. cmd is the ID and
	// request is what follows it, the same split the server side hands a
	// handler, so both ends of the escape hatch read the same way.
	//
	// What those commands mean is a particular probe's business, so nothing in
	// this module implements or interprets one; a caller that knows its probe
	// sends them through here.
	VendorCommand(ctx context.Context, cmd uint8, request []byte) ([]byte, error)

	// MaxPacketSize bounds a variable-length vendor command's request and
	// response. It is what the probe reported through DAP_ID_PACKET_SIZE,
	// narrowed by whatever the transport can carry.
	MaxPacketSize() int
}
