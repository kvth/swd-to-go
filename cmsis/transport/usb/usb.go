//go:build !no_libudev

// Package usb carries CMSIS-DAP packets to a probe over USB HID.
//
// Build with the no_libudev tag to leave this and its cgo HID dependency out of
// the binary; the rest of the module does not need it.
package usb

import (
	"context"
	"encoding/hex"
	"errors"
	"fmt"
	"log/slog"

	"github.com/cesanta/hid"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/cmsis/client"
)

// ErrDeviceNotFound is what [Open] reports when no HID device matches.
// Callers test for it with errors.Is.
var ErrDeviceNotFound = errors.New("usb: no matching CMSIS-DAP device")

// Device says which HID device a [Transport] was opened on.
//
// It is this package's own type and not the HID library's, so that asking
// which probe you got does not oblige you to import the HID library — or to
// keep importing the same one if this package ever changes which it uses.
//
// There is no serial number here because HID enumeration does not carry one.
// A probe's serial comes from the probe, over DAP_Info: [client.Probe] reports
// it through SerialNumber, and [Open] matches on it that way.
type Device struct {
	// VendorID and ProductID are the USB ids the device was matched on.
	VendorID  uint16
	ProductID uint16
	// Version is the bcdDevice release number the device reports.
	Version uint16

	// Manufacturer and Product are the USB string descriptors — the device's
	// own account of what it is. Either may be empty.
	Manufacturer string
	Product      string

	// Path is the platform's handle for the device: a /dev node on Linux, an
	// IOService path on macOS. It is what tells two otherwise identical probes
	// apart, and it is not stable across replugging.
	Path string
}

func deviceFrom(di *hid.DeviceInfo) Device {
	return Device{
		VendorID:     di.VendorID,
		ProductID:    di.ProductID,
		Version:      di.VersionNumber,
		Manufacturer: di.Manufacturer,
		Product:      di.Product,
		Path:         di.Path,
	}
}

// Transport is a [client.Transport] over a HID device.
type Transport struct {
	d   hid.Device
	dev Device
	log *slog.Logger
	// buf holds the outgoing report: one leading report-number byte that the
	// HID layer eats, then the CMSIS-DAP packet.
	buf []byte
}

// Device reports which HID device this transport was opened on.
func (t *Transport) Device() Device { return t.dev }

// SetLogger points this transport's packet tracing at a logger of the caller's
// choosing. Unset, it uses [slog.Default]. Every packet in and out is one
// record at [swd.LevelTrace].
func (t *Transport) SetLogger(l *slog.Logger) { t.log = l }

func (t *Transport) logger() *slog.Logger {
	if t.log != nil {
		return t.log
	}
	return slog.Default()
}

var _ client.Transport = (*Transport)(nil)

// Open finds a CMSIS-DAP probe by USB vendor and product id and returns a probe
// talking to it. An empty serial matches the first device that answers.
//
// Serial matching happens over CMSIS-DAP rather than over HID: the HID library
// does not surface a device's serial string, but every probe reports one
// through DAP_Info, so a candidate is opened, asked, and closed again if it is
// the wrong one.
func Open(ctx context.Context, vid, pid uint16, serial string) (*client.Probe, error) {
	devs, err := hid.Devices()
	if err != nil {
		return nil, fmt.Errorf("usb: failed to enumerate HID devices: %w", err)
	}

	var lastErr error
	for i, di := range devs {
		slog.Default().DebugContext(ctx, "HID device", "index", i,
			"vid", hex16(di.VendorID), "pid", hex16(di.ProductID), "path", di.Path)
		if di.VendorID != vid || di.ProductID != pid {
			continue
		}

		t, err := open(di)
		if err != nil {
			lastErr = err
			continue
		}
		p, err := client.New(ctx, t)
		if err != nil {
			lastErr = err
			continue
		}
		if serial == "" {
			return p, nil
		}

		got, err := p.SerialNumber(ctx)
		if err != nil {
			p.Close(ctx)
			lastErr = fmt.Errorf("usb: %s: could not read serial: %w", di.Path, err)
			continue
		}
		if got == serial {
			return p, nil
		}
		p.Close(ctx)
	}

	if lastErr != nil {
		return nil, lastErr
	}
	return nil, fmt.Errorf("usb: device %04x:%04x serial %q: %w", vid, pid, serial, ErrDeviceNotFound)
}

// New opens the first device matching vid and pid as a bare transport, without
// building a probe on top of it.
func New(vid, pid uint16) (*Transport, error) {
	devs, err := hid.Devices()
	if err != nil {
		return nil, fmt.Errorf("usb: failed to enumerate HID devices: %w", err)
	}
	for _, di := range devs {
		if di.VendorID == vid && di.ProductID == pid {
			return open(di)
		}
	}
	return nil, fmt.Errorf("usb: device %04x:%04x: %w", vid, pid, ErrDeviceNotFound)
}

func open(di *hid.DeviceInfo) (*Transport, error) {
	d, err := di.Open()
	if err != nil {
		return nil, fmt.Errorf("usb: failed to open %04x:%04x (%s): %w",
			di.VendorID, di.ProductID, di.Path, err)
	}
	dev := deviceFrom(di)
	slog.Default().Debug("usb: opened HID device",
		"vid", hex16(dev.VendorID), "pid", hex16(dev.ProductID), "path", dev.Path)
	return &Transport{d: d, dev: dev}, nil
}

// hex16 keeps USB ids readable in a log line.
func hex16(v uint16) string { return fmt.Sprintf("0x%04x", v) }

// MaxPacketSize returns zero: the HID layer handles its own report sizes, so
// whatever the probe reports through DAP_ID_PACKET_SIZE wins.
func (t *Transport) MaxPacketSize() int { return 0 }

func (t *Transport) Close() error {
	t.d.Close()
	return nil
}

func (t *Transport) Exchange(ctx context.Context, request, response []byte) (int, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if len(request) == 0 {
		return 0, fmt.Errorf("usb: empty request")
	}

	// One leading zero for the HID report number, which is not part of the
	// CMSIS-DAP packet.
	t.buf = append(t.buf[:0], 0)
	t.buf = append(t.buf, request...)

	if l := t.logger(); l.Enabled(ctx, swd.LevelTrace) {
		l.Log(ctx, swd.LevelTrace, "dap out", "packet", hex.EncodeToString(request))
	}
	if err := t.d.Write(t.buf); err != nil {
		return 0, fmt.Errorf("usb: write failed: %w", err)
	}

	select {
	case <-ctx.Done():
		return 0, ctx.Err()
	case resp, ok := <-t.d.ReadCh():
		if !ok {
			return 0, fmt.Errorf("usb: read failed: %w", t.d.ReadError())
		}
		if l := t.logger(); l.Enabled(ctx, swd.LevelTrace) {
			l.Log(ctx, swd.LevelTrace, "dap in", "packet", hex.EncodeToString(resp))
		}
		n := copy(response, resp)
		if n < len(resp) {
			return 0, fmt.Errorf("usb: response of %d bytes does not fit a %d byte buffer", len(resp), len(response))
		}
		return n, nil
	}
}
