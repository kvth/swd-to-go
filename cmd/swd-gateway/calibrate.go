package main

import (
	"context"
	"encoding/binary"
	"log/slog"

	"github.com/kvth/swd-to-go"
)

// The vendor command block this gateway implements, in the CMSIS-DAP
// ID_DAP_Vendor0..31 range. Only one command is claimed; the rest of the range
// answers ID_DAP_Invalid, the same as a server with no handler at all.
//
//	0x91 Calibrate
//	  request:   (no arguments)
//	  response:  u8 status, u32 speed_coeff, u32 speed_offset
//
// The ID and the layout match the C gateway at cmsis-dap-tcp-gateway-rpi, so a
// client that knows that probe can talk to this one.
const cmdCalibrate = 0x91

// Vendor status bytes. Zero is success; the rest follow the C gateway's table,
// of which only this one can arise here.
const (
	vendorStatusOK        = 0x00
	vendorStatusNotMapped = 0x12 // the measurement produced nothing usable
)

// calibrateHandler answers the Calibrate command by re-measuring the driver's
// delay loop. Its HandleVendor is what goes in [server.Options.HandleVendor].
type calibrateHandler struct {
	cal swd.Calibrator
	log *slog.Logger
}

func (h *calibrateHandler) HandleVendor(ctx context.Context, cmd uint8, request, response []byte) (int, int, bool) {
	if cmd != cmdCalibrate {
		return 0, 0, false
	}
	// status byte plus two u32s.
	const respLen = 1 + 4 + 4
	if len(response) < respLen {
		return 0, 0, false
	}

	// Leave the two values zero unless the measurement actually succeeds, so a
	// client cannot mistake a failure for a calibration of zero.
	for i := 1; i < respLen; i++ {
		response[i] = 0
	}

	coeff, offset, err := h.cal.Calibrate()
	if err != nil {
		h.log.Warn("calibrate failed", "err", err)
		response[0] = vendorStatusNotMapped
		return 0, respLen, true
	}

	h.log.Info("calibrated", "speed_coeff", coeff, "speed_offset", offset,
		"max_khz", coeff/offset)

	response[0] = vendorStatusOK
	binary.LittleEndian.PutUint32(response[1:5], coeff)
	binary.LittleEndian.PutUint32(response[5:9], offset)
	return 0, respLen, true
}
