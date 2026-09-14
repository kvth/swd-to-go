// Command flash-rp2040 writes an image to an RP2040's flash through its
// bootrom, over a probe of either kind.
//
// This is the example for the layers stacking: the Flasher batches a whole ROM
// call's register setup into one transfer list and moves bulk data with block
// transfers, and it does that identically whether the probe underneath is a
// socket or a GPIO pin — the batching lives above swd.Probe, so both get it.
//
// With -local it drives GPIO directly and needs a Raspberry Pi plus root;
// otherwise it dials a CMSIS-DAP probe, where the same batching is what keeps
// the packet count down over the link.
//
// The flags follow OpenOCD's `program` command, since that is the vocabulary
// anyone flashing one of these already has: -preverify skips the write when the
// image is already there, -verify reads it back afterwards, -reset/-run leave
// the target going. -rescue is the way in when the code on the board is what is
// stopping you.
package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"syscall"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/cmsis/transport/tcp"
	"github.com/kvth/swd-to-go/drivers/bcm2835"
	"github.com/kvth/swd-to-go/drivers/linuxgpiod"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
	"github.com/kvth/swd-to-go/target/rp2040"
)

func main() {
	var (
		image     = flag.String("image", "", "binary image to write (required)")
		offset    = flag.Uint("offset", 0, "flash offset to write it at")
		verify    = flag.Bool("verify", true, "read the image back and compare afterwards")
		preverify = flag.Bool("preverify", false, "read it back first and skip the write if it is already there")
		erase     = flag.String("erase", "auto", "which sectors to erase: auto, all or none")
		run_      = flag.Bool("run", true, "reset the target and let it run afterwards")
		rescue    = flag.Bool("rescue", false, "reset the chip into the bootrom first, for a board whose firmware locks the debug port out")

		addr  = flag.String("probe", "127.0.0.1:4441", "CMSIS-DAP probe address")
		local = flag.Bool("local", false, "use the board's own bit-banged port instead")
		clock = flag.Uint("clock", 1_500_000, "SWD clock in Hz")

		driver   = flag.String("driver", "bcm2835", "with -local, the GPIO backend: bcm2835 (registers, Pi 1-4)\n\tor linuxgpiod (character device, any board)")
		swclk    = flag.String("swclk", "16", "SWCLK pin, with -local; a line number, or chip:line\n\tto drive it through the Linux GPIO character device (needed on a Pi 5)")
		swdioIn  = flag.String("swdio-in", "12", "SWDIO in pin, with -local")
		swdioOut = flag.String("swdio-out", "20", "SWDIO out pin, with -local")
		swdioDir = flag.String("swdio-dir", "21", "buffer direction pin with -local, or -1 for none")
	)
	flag.Parse()

	if *image == "" {
		flag.Usage()
		log.Fatal("an -image to write is required")
	}

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	data, err := os.ReadFile(*image)
	if err != nil {
		log.Fatal(err)
	}

	open := func() (swd.Probe, error) { return tcp.Dial(ctx, *addr) }
	if *local {
		open = func() (swd.Probe, error) {
			p, perr := pins(*driver, *swclk, *swdioIn, *swdioOut, *swdioDir)
			if perr != nil {
				return nil, perr
			}
			d, err := openDriver(*driver, p)
			if err != nil {
				return nil, err
			}
			return bitbang.New(d), nil
		}
	}

	mode, err := eraseMode(*erase)
	if err != nil {
		log.Fatal(err)
	}

	opts := rp2040.Options{
		Offset:    uint32(*offset),
		Erase:     mode,
		PreVerify: *preverify,
		Verify:    *verify,
		Reset:     *run_,
		Run:       *run_,
		Progress:  progress,
	}
	if err := flash(ctx, open, uint32(*clock), data, opts, *rescue); err != nil {
		log.Fatal(err)
	}
}

func eraseMode(s string) (rp2040.EraseMode, error) {
	switch s {
	case "auto":
		return rp2040.EraseAuto, nil
	case "all":
		return rp2040.EraseAll, nil
	case "none":
		return rp2040.EraseNone, nil
	}
	return 0, fmt.Errorf("-erase must be auto, all or none, not %q", s)
}

func flash(ctx context.Context, open func() (swd.Probe, error),
	clockHz uint32, image []byte, opts rp2040.Options, rescue bool) error {

	probe, err := open()
	if err != nil {
		return err
	}
	defer probe.Close(ctx)

	// The clock is set once, here, and everything below runs at it: neither
	// Rescue nor Attach touches it.
	if clockHz != 0 {
		if err := probe.SetClock(ctx, clockHz); err != nil {
			return fmt.Errorf("set clock: %w", err)
		}
	}

	// A rescue resets the chip and stops it in the bootrom, so it has to come
	// before the bring-up rather than through it: nothing known about the
	// target survives one.
	if rescue {
		if err := rp2040.Rescue(ctx, probe); err != nil {
			return fmt.Errorf("rescue: %w", err)
		}
		fmt.Fprintln(os.Stderr, "rescued: the chip is stopped in the bootrom")
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

	// AP 0: both cores see the same bootrom and the same SRAM, so either can
	// flash. The zeros are the default ROM stack and bounce buffer.
	f := rp2040.NewFlasher(probe, dpc, apc, 0, 0, 0, 0)

	res, err := f.FlashImage(ctx, image, opts)
	if err != nil {
		return err
	}

	fmt.Fprintf(os.Stderr, "\n%v\n", res)
	if opts.Reset {
		if opts.Run {
			fmt.Fprintln(os.Stderr, "target running")
		} else {
			fmt.Fprintln(os.Stderr, "target halted at the reset vector")
		}
	}
	return nil
}

// progress redraws one line rather than scrolling, since these phases are
// hundreds of chunks long.
func progress(phase string, done, total int) {
	if total <= 0 {
		return
	}
	fmt.Fprintf(os.Stderr, "\r%-10s %3d%% (%d/%d)", phase, done*100/total, done, total)
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
