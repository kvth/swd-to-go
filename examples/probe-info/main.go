// Command probe-info asks a probe what it is and what it can do.
//
// It is the example for where the line between swd.Probe and swd.Cmsis falls.
// Everything a probe can answer for itself — the pins included, since a probe
// that cannot reach them says so with swd.ErrNotImplemented — is on swd.Probe.
// Everything that needs a CMSIS-DAP on the other end to answer at all — the
// identity strings, the status LEDs, the vendor command range — is swd.Cmsis,
// which you type-assert for. A bit-banged probe is not one, and rather than
// stub out six methods with plausible-looking lies it simply does not claim to
// be.
//
// With -local it opens the board's own GPIO (needs a Raspberry Pi and root);
// otherwise it dials a CMSIS-DAP probe.
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"log"
	"os"
	"text/tabwriter"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/cmsis/transport/tcp"
	"github.com/kvth/swd-to-go/drivers/bcm2835"
	"github.com/kvth/swd-to-go/drivers/linuxgpiod"
	"github.com/kvth/swd-to-go/probe/bitbang"
)

func main() {
	var (
		addr  = flag.String("probe", "127.0.0.1:4441", "CMSIS-DAP probe address")
		local = flag.Bool("local", false, "open the board's own bit-banged port instead")

		driver   = flag.String("driver", "bcm2835", "with -local, the GPIO backend: bcm2835 (registers, Pi 1-4)\n\tor linuxgpiod (character device, any board)")
		swclk    = flag.String("swclk", "16", "SWCLK pin, with -local; a line number, or chip:line\n\tto drive it through the Linux GPIO character device (needed on a Pi 5)")
		swdioIn  = flag.String("swdio-in", "12", "SWDIO in pin, with -local")
		swdioOut = flag.String("swdio-out", "20", "SWDIO out pin, with -local")
		swdioDir = flag.String("swdio-dir", "21", "buffer direction pin with -local, or -1 for none")
	)
	flag.Parse()

	ctx := context.Background()

	var (
		probe swd.Probe
		err   error
	)
	if *local {
		var d swd.BitBanger
		p, perr := pins(*driver, *swclk, *swdioIn, *swdioOut, *swdioDir)
		if perr != nil {
			log.Fatal(perr)
		}
		d, err = openDriver(*driver, p)
		if err == nil {
			probe = bitbang.New(d)
		}
	} else {
		probe, err = tcp.Dial(ctx, *addr)
	}
	if err != nil {
		log.Fatal(err)
	}
	defer probe.Close(ctx)

	report(ctx, probe)
}

func report(ctx context.Context, probe swd.Probe) {
	w := tabwriter.NewWriter(os.Stdout, 0, 0, 2, ' ', 0)
	defer w.Flush()

	fmt.Fprintf(w, "type\t%T\n", probe)
	fmt.Fprintf(w, "max block\t%d words per transfer\n", probe.MaxBlockSize())

	// The pins are on swd.Probe: every probe is asked, and one that cannot
	// reach them answers swd.ErrNotImplemented rather than guessing.
	switch level, err := probe.ReadPins(ctx); {
	case errors.Is(err, swd.ErrNotImplemented):
		fmt.Fprintf(w, "pins\tnot reachable through this probe\n")
	case err != nil:
		fmt.Fprintf(w, "pins\t(%v)\n", err)
	default:
		fmt.Fprintf(w, "pins\tSWCLK=%d SWDIO=%d nRESET=%d\n",
			b2i(level&swd.PinSWCLK != 0), b2i(level&swd.PinSWDIO != 0),
			b2i(level&swd.PinNRESET != 0))
	}

	// Everything else needs a CMSIS-DAP on the far end, so it is one assertion
	// for the lot rather than one per question.
	dap, ok := probe.(swd.Cmsis)
	if !ok {
		fmt.Fprintf(w, "CMSIS-DAP\tno (this probe is local hardware)\n")
		return
	}

	fmt.Fprintf(w, "CMSIS-DAP\tyes, up to %d bytes per packet\n", dap.MaxPacketSize())
	for _, field := range []struct {
		name string
		get  func(context.Context) (string, error)
	}{
		{"vendor", dap.VendorID},
		{"product", dap.ProductID},
		{"serial", dap.SerialNumber},
		{"firmware", dap.FirmwareVersion},
		{"target vendor", dap.TargetVendor},
		{"target name", dap.TargetName},
	} {
		value, err := field.get(ctx)
		if err != nil {
			value = "(" + err.Error() + ")"
		}
		fmt.Fprintf(w, "%s\t%s\n", field.name, value)
	}
}

func b2i(b bool) int {
	if b {
		return 1
	}
	return 0
}

// The GPIO backends -driver picks between.
const (
	driverBCM2835    = "bcm2835"
	driverLinuxGPIOD = "linuxgpiod"
)

// pins parses the four pin flags against the backend that was asked for.
//
// The two have to agree: -driver bcm2835 drives the SoC's own GPIO and takes
// bare line numbers, -driver linuxgpiod takes chip:line for every pin, and a
// pin written the other way is refused rather than reinterpreted. There is no
// default chip, because the pin that would land on it is the one worth being
// sure about.
func pins(driver, swclk, swdioIn, swdioOut, swdioDir string) ([4]linuxgpiod.Pin, error) {
	var out [4]linuxgpiod.Pin
	if driver != driverBCM2835 && driver != driverLinuxGPIOD {
		return out, fmt.Errorf("unknown -driver %q, want %s or %s", driver, driverBCM2835, driverLinuxGPIOD)
	}

	for i, f := range []struct{ name, spec string }{
		{"swclk", swclk}, {"swdio-in", swdioIn}, {"swdio-out", swdioOut}, {"swdio-dir", swdioDir},
	} {
		p, err := linuxgpiod.ParsePin(f.spec)
		if err != nil {
			return out, fmt.Errorf("-%s: %w", f.name, err)
		}
		// A direction pin given as -1 is not wired, and is on no chip.
		if p.Line >= 0 {
			switch {
			case driver == driverBCM2835 && p.Chip != "":
				return out, fmt.Errorf("-%s is %q, but -driver %s takes a bare line number; "+
					"-driver %s is the one with chips to choose between",
					f.name, f.spec, driverBCM2835, driverLinuxGPIOD)
			case driver == driverLinuxGPIOD && p.Chip == "":
				return out, fmt.Errorf("-%s is %q, but -driver %s needs every pin to name its "+
					"chip: write it as chip:line", f.name, f.spec, driverLinuxGPIOD)
			}
		}
		out[i] = p
	}
	return out, nil
}

// openDriver opens the named backend with the wiring the pins describe -- one
// bidirectional pin on its own, one behind a buffer, or the split pair.
//
// bcm2835 writes the BCM GPIO registers directly, which is fast and is Pi 1-4
// only. linuxgpiod goes through the Linux GPIO character device, which is a
// syscall per edge and works anywhere, a Pi 5 included. Nothing below this
// function can tell which it got.
//
// It also calibrates. A driver arrives uncalibrated because measuring clocks
// SWCLK, and only the caller knows when that is safe; here it is, right after
// opening and before anything is attached.
func openDriver(driver string, p [4]linuxgpiod.Pin) (swd.BitBanger, error) {
	var d interface {
		swd.BitBanger
		swd.Calibrator
	}

	swclk, swdioIn, swdioOut, swdioDir := p[0], p[1], p[2], p[3]
	chardev := driver == driverLinuxGPIOD
	var err error

	switch {
	case chardev && swdioIn == swdioOut && swdioDir.Line < 0:
		d, err = linuxgpiod.OpenBidirPins(swclk, swdioIn)
	case chardev && swdioIn == swdioOut:
		d, err = linuxgpiod.OpenBidirBufferedPins(swclk, swdioIn, swdioDir)
	case chardev:
		d, err = linuxgpiod.OpenSplitPins(swclk, swdioIn, swdioOut, swdioDir)

	case swdioIn == swdioOut && swdioDir.Line < 0:
		d, err = bcm2835.OpenBidir(swclk.Line, swdioIn.Line)
	case swdioIn == swdioOut:
		d, err = bcm2835.OpenBidirBuffered(swclk.Line, swdioIn.Line, swdioDir.Line)
	default:
		d, err = bcm2835.OpenSplit(swclk.Line, swdioIn.Line, swdioOut.Line, swdioDir.Line)
	}
	if err != nil {
		return nil, err
	}

	if _, _, err := d.Calibrate(); err != nil {
		d.Close()
		return nil, fmt.Errorf("calibrate: %w", err)
	}
	return d, nil
}
