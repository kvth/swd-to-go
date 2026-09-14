package rp2040

import (
	"context"
	"errors"
	"fmt"

	"github.com/kvth/swd-to-go"
)

// errRescueFailed means the rescue port took the reset but did not let go of
// it: the power-up request or its acknowledge was still set afterwards.
var errRescueFailed = errors.New("rp2040: the rescue port did not release the reset")

// DP registers, by the A[3:2] field of a transfer request. Register 0 is
// DPIDR read and ABORT written.
const (
	dpIDR       uint8 = 0x00
	dpABORT     uint8 = 0x00
	dpCTRLSTAT  uint8 = 0x04
	dpSELECT    uint8 = 0x08
	abortAllClr       = 0x1E // every sticky-error clear bit

	ctrlCDBGPWRUPREQ uint32 = 1 << 28
	ctrlCDBGPWRUPACK uint32 = 1 << 29
	ctrlCSYSPWRUPREQ uint32 = 1 << 30
	ctrlCSYSPWRUPACK uint32 = 1 << 31

	ctrlPwrUpReq = ctrlCDBGPWRUPREQ | ctrlCSYSPWRUPREQ
	ctrlPwrUpAck = ctrlCDBGPWRUPACK | ctrlCSYSPWRUPACK
)

// Rescue resets an RP2040 and stops it in the bootrom before it runs anything
// out of flash.
//
// This is the way back in when the target is the problem: firmware that hangs,
// that reconfigures the QSPI pins, that parks a core somewhere the debug port
// cannot follow, or that disables the debug port outright. [Flasher.Reset] with
// haltAfter cannot help there, because the code being reset into is the thing
// going wrong; this stops the chip earlier than that.
//
// Instance 0xF on the multidrop wire is not a core's debug port. Its
// CDBGPWRUPREQ is wired to the chip's power-on state machine: asserting it
// resets the chip with a flag set in VREG_AND_POR_CHIP_RESET, and clearing it
// releases the reset, whereupon the bootrom sees that flag and halts.
//
// Nothing acknowledges the power-up request here, because the state machine is
// the thing being held in reset — which is why this cannot go through
// target/dp's Client, whose Init waits for an acknowledge that will never
// come.
//
// Afterwards the chip is freshly reset and nothing either side knew about it is
// true any more: not the debug port, not the AP, and not the target's RAM.
// Carry on with a fresh [Attach].
//
// Set the clock before calling this, with [swd.Probe.SetClock]; the rescue runs
// at whatever the probe is already set to.
func Rescue(ctx context.Context, probe swd.Probe) error {
	if err := probe.Connect(ctx); err != nil {
		return fmt.Errorf("connect: %w", err)
	}
	if err := swd.JTAGToDormant(ctx, probe); err != nil {
		return fmt.Errorf("JTAG to dormant: %w", err)
	}
	if err := swd.DormantToSWD(ctx, probe); err != nil {
		return fmt.Errorf("dormant to SWD: %w", err)
	}
	if err := swd.LineReset(ctx, probe); err != nil {
		return fmt.Errorf("line reset: %w", err)
	}
	if err := swd.SelectTarget(ctx, probe, TARGET_RESCUE); err != nil {
		return fmt.Errorf("select the rescue port: %w", err)
	}

	// DPIDR is the only register readable before the sticky bits are cleared,
	// and on a multidrop wire ADIv5.2 requires it be read first anyway. The
	// rescue port is one of the three debug ports on the chip and reports the
	// same DPIDR as the other two, so it gets the same check: an undriven
	// wire, or something that is not an RP2040, is not a chip to rescue.
	if err := checkIDR(ctx, probe); err != nil {
		return fmt.Errorf("rescue port: %w", err)
	}

	if err := dpWrite(ctx, probe, dpABORT, abortAllClr); err != nil {
		return fmt.Errorf("clear sticky errors: %w", err)
	}
	if err := dpWrite(ctx, probe, dpSELECT, 0); err != nil {
		return fmt.Errorf("write SELECT: %w", err)
	}

	// Assert: the chip goes into reset here and stays there.
	if err := dpWrite(ctx, probe, dpCTRLSTAT, ctrlPwrUpReq); err != nil {
		return fmt.Errorf("assert the chip reset: %w", err)
	}
	// Release: the bootrom runs from here and halts itself.
	if err := dpWrite(ctx, probe, dpCTRLSTAT, 0); err != nil {
		return fmt.Errorf("release the chip reset: %w", err)
	}

	ctrlStat, err := dpRead(ctx, probe, dpCTRLSTAT)
	if err != nil {
		return fmt.Errorf("read CTRL/STAT: %w", err)
	}
	// Every power request and acknowledge bit has to have gone, or the reset
	// is still asserted.
	if ctrlStat&(ctrlPwrUpReq|ctrlPwrUpAck) != 0 {
		return fmt.Errorf("%w (ctrl/stat 0x%08x)", errRescueFailed, ctrlStat)
	}

	// Leave the wire dormant, which is what the next bring-up expects to find
	// and what another debugger on this multidrop target needs.
	if err := swd.SWDToDormant(ctx, probe); err != nil {
		return fmt.Errorf("SWD to dormant: %w", err)
	}
	return nil
}

// RescueAndAttach rescues the chip and then brings up one of its cores, which
// is the whole recovery path in one call. Like [Attach] it sets no clock and
// initialises nothing above the wire, and its detach is never nil.
func RescueAndAttach(ctx context.Context, probe swd.Probe, core uint32) (detach func(context.Context) error, err error) {
	if err := Rescue(ctx, probe); err != nil {
		return func(context.Context) error { return nil }, err
	}
	return Attach(ctx, probe, core)
}

// dpRead and dpWrite are raw debug-port access, below target/dp's Client.
// Rescue
// needs them because the sequence it runs is precisely the one a debug-port
// client's bring-up refuses to do: request power and then take the request
// away again without ever seeing an acknowledge.
func dpRead(ctx context.Context, probe swd.Probe, reg uint8) (uint32, error) {
	_, data, err := probe.Transfer(ctx, []swd.Request{{Op: swd.OpRead, Reg: reg}})
	if err != nil {
		return 0, err
	}
	if len(data) == 0 {
		return 0, errors.New("rp2040: the probe returned no value for a read")
	}
	return data[0], nil
}

func dpWrite(ctx context.Context, probe swd.Probe, reg uint8, value uint32) error {
	_, _, err := probe.Transfer(ctx, []swd.Request{{Op: swd.OpWrite, Reg: reg, Data: value}})
	return err
}
