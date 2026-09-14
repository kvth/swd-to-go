// Command rtt-local tails a target's RTT channel over locally bit-banged SWD,
// with no CMSIS-DAP anywhere in the path.
//
// This is the example worth reading first. There is no probe process, no
// socket, no packet encoding and no decoding: a GPIO driver, the transfer
// engine on top of it, and the RTT layer on top of that, all in one process.
// Every transfer goes from the RTT code straight to the wire.
//
// Compare examples/rtt-tcp, which is the same program against a probe over a
// socket. The only line that differs is where the probe comes from.
//
// Needs a Raspberry Pi, and root or membership of the gpio group.
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/drivers/bcm2835"
	"github.com/kvth/swd-to-go/drivers/linuxgpiod"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
	"github.com/kvth/swd-to-go/target/rp2040"
	"github.com/kvth/swd-to-go/target/rtt"
)

func main() {
	var (
		driver   = flag.String("driver", "bcm2835", "the GPIO backend: bcm2835 (registers, Pi 1-4) or\n\tlinuxgpiod (character device, any board)")
		swclk    = flag.String("swclk", "16", "SWCLK pin; a line number, or chip:line to drive it\n\tthrough the Linux GPIO character device (needed on a Pi 5)")
		swdioIn  = flag.String("swdio-in", "12", "SWDIO in pin; set equal to -swdio-out for a single bidirectional pin")
		swdioOut = flag.String("swdio-out", "20", "SWDIO out pin")
		swdioDir = flag.String("swdio-dir", "21", "buffer direction pin, or -1 for a bare bidirectional pin")
		clock    = flag.Uint("clock", 1_500_000, "SWD clock in Hz")
		channel  = flag.Int("channel", 1, "RTT up channel to read")
		searchAt = flag.Uint("search", 0x20000000, "where to start looking for the RTT control block")
	)
	flag.Parse()

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	if err := run(ctx, wiring{*driver, *swclk, *swdioIn, *swdioOut, *swdioDir},
		uint32(*clock), *channel, uint32(*searchAt)); err != nil {
		log.Fatal(err)
	}
}

// wiring is the backend and the pin flags, gathered up, as written.
type wiring struct {
	driver                             string
	swclk, swdioIn, swdioOut, swdioDir string
}

func run(ctx context.Context, w wiring, clockHz uint32, channel int, searchAt uint32) error {
	// A GPIO driver: pins, bits and turnarounds, nothing else.
	p, err := pins(w.driver, w.swclk, w.swdioIn, w.swdioOut, w.swdioDir)
	if err != nil {
		return err
	}
	d, err := openDriver(w.driver, p)
	if err != nil {
		return fmt.Errorf("open GPIO: %w", err)
	}
	defer d.Close()

	// The transfer engine on top of it. This is a swd.Probe, the same interface
	// examples/rtt-tcp gets from a socket.
	probe := bitbang.New(d)
	defer probe.Close(ctx)

	// ...and from here on, nothing knows or cares which kind of probe it is.
	if clockHz != 0 {
		if err := probe.SetClock(ctx, clockHz); err != nil {
			return fmt.Errorf("set clock: %w", err)
		}
	}

	// Attach is the bring-up and the presence check in one, and it stops
	// there. A tool that only wanted to know whether a target is on the wire
	// would stop here too.
	detach, err := rp2040.Attach(ctx, probe, rp2040.TARGET_CORE_0)
	if err != nil {
		return err
	}
	defer detach(context.WithoutCancel(ctx))

	// The debug port and the MEM-AP are separate steps, taken only by work
	// that needs to reach target memory.
	dpc := dp.New(probe)
	if err := dpc.Init(ctx); err != nil {
		return fmt.Errorf("init DP: %w", err)
	}
	apc := mem.New(dpc, 0)
	if err := apc.Init(ctx); err != nil {
		return fmt.Errorf("init MEM-AP: %w", err)
	}

	return tail(ctx, apc, channel, searchAt)
}

func tail(ctx context.Context, apc *mem.Client, channel int, searchAt uint32) error {
	cbAddr, err := rtt.FindRTTControlBlock(ctx, apc, searchAt, 0x2000, 0x200)
	if err != nil {
		return fmt.Errorf("find RTT control block: %w", err)
	}
	fmt.Fprintf(os.Stderr, "RTT control block at 0x%08X\n", cbAddr)

	r, err := rtt.New(ctx, apc, cbAddr)
	if err != nil {
		return err
	}

	// The channel's own name and size, which is the quickest way to tell a
	// control block that is really there from an address that happened to
	// spell one out.
	up, down := r.Channels()
	if info, err := r.Channel(ctx, rtt.Up, channel); err == nil {
		fmt.Fprintf(os.Stderr, "up channel %d %q, %d bytes (%d up, %d down declared)\n",
			channel, info.Name, info.Size, up, down)
	}

	for ctx.Err() == nil {
		data, err := r.Read(ctx, channel)
		if err != nil {
			return err
		}
		if len(data) > 0 {
			os.Stdout.Write(data)
			// There may be more waiting; do not sleep before asking again.
			continue
		}
		select {
		case <-ctx.Done():
		case <-time.After(10 * time.Millisecond):
		}
	}
	return nil
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
