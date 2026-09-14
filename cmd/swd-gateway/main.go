// Command swd-gateway bit-bangs SWD over Raspberry Pi GPIO and serves it as a
// CMSIS-DAP probe over TCP or a Unix socket.
//
// The wire protocol is the one OpenOCD's "cmsis-dap backend tcp" speaks, so
// OpenOCD can point straight at it.
package main

import (
	"context"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"strconv"
	"syscall"

	"github.com/spf13/cobra"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/cmsis/server"
	servertcp "github.com/kvth/swd-to-go/cmsis/server/tcp"
	"github.com/kvth/swd-to-go/drivers/bcm2835"
	"github.com/kvth/swd-to-go/drivers/linuxgpiod"
	"github.com/kvth/swd-to-go/probe/bitbang"
)

// The GPIO backends --driver picks between.
const (
	driverBCM2835    = "bcm2835"
	driverLinuxGPIOD = "linuxgpiod"
)

// Defaults, matching the C gateway this replaces.
const (
	defaultPort = 4441

	defaultSWCLK    = 16
	defaultSWDIOIn  = 12
	defaultSWDIOOut = 20
	defaultSWDIODir = 21

	defaultClockHz = 1_000_000
)

type options struct {
	bind       string
	port       int
	unixSocket string
	logLevel   string

	driver    string
	listChips bool

	// The pin flags, as given: "16", or "gpiochip2:5" for a pin on a chip of
	// its own. Parsed by options.pin and options.pinNumber below.
	swclk    string
	swdio    string
	swdioIn  string
	swdioOut string
	swdioDir string

	// Settled by checkWiring: which wiring the flags describe.
	bidir bool

	clockHz     uint32
	speedCoeff  uint32
	speedOffset uint32

	// Which of the wiring and clock flags the user actually gave, since for
	// several of them the default and an explicit value mean different things.
	gaveSWDIO      bool
	gaveSWDIOSplit bool
	gaveSWDIODir   bool
	gaveCoeffs     bool
}

const longHelp = `Bitbanged SWD <-> CMSIS-DAP gateway. Drives SWCLK/SWDIO over GPIO and serves
CMSIS-DAP over TCP or a Unix domain socket.

Backends

  --driver bcm2835 (the default) writes the BCM283x/BCM2711 GPIO registers
  through /dev/gpiomem. That is a store instruction per edge and megahertz of
  SWCLK, and it is Pi 1-4 and CM1/CM3/CM4 only. Its pins are bare line numbers:

    --driver bcm2835 --swclk 16 --swdio-in 12 --swdio-out 20 --swdio-dir 21

  --driver linuxgpiod uses the Linux GPIO character device, which works on any
  board -- including a Pi 5, Pi 500 or CM5, where the GPIO lives on an RP1
  southbridge that the register backend cannot reach at all. It costs a syscall
  per edge, so expect hundreds of kilohertz rather than megahertz. Its pins are
  written chip:line, every one of them:

    --driver linuxgpiod --swclk gpiochip0:16 --swdio-in gpiochip0:12 \
      --swdio-out gpiochip0:20 --swdio-dir gpiochip0:21

  The chip half is a number, a name, a path or a label, so 0:16, gpiochip0:16,
  /dev/gpiochip0:16 and pinctrl-rp1:16 are the same pin. --list-gpiochips prints
  what this machine has and exits: the header pins are gpiochip0 labelled
  pinctrl-rp1 on a Pi 5 and pinctrl-bcm2711 on a Pi 4, but that has moved
  between kernel versions, which is what the listing is for.

  A pin written the other backend's way is refused rather than guessed at. There
  is no default chip either: every pin says where it is, which is the reason one
  cannot quietly end up on the wrong controller.

Pins on more than one chip

  Since each pin names its own chip, they do not all have to be on the same one
  -- the data pins on the SoC and the buffer direction line on an I2C expander,
  say:

    --swclk gpiochip0:16 --swdio-in gpiochip0:12 --swdio-out gpiochip0:20 \
      --swdio-dir gpiochip2:5

  Where that costs something is said plainly at startup. A direction line
  elsewhere is free, because a turnaround was always an ioctl of its own. SWDIO
  elsewhere is not: SWDIO and SWCLK share one ioctl only while they are in the
  same line request, so splitting them makes a written bit three syscalls
  instead of two and the gateway logs a warning saying so.

SWDIO wiring

  The default is the split wiring: separate input and output pins behind a
  buffer whose direction is driven by --swdio-dir.

  Pass --swdio instead for a single bidirectional pin, where the GPIO itself is
  flipped between input and output. It cannot be combined with
  --swdio-in/--swdio-out. On its own it drives no direction line at all; add
  --swdio-dir when that one data pin sits behind a buffer that needs one.

  The three are OpenOCD's "adapter gpio swdio" with, respectively, neither
  swdio_dir nor swdio_in; swdio_dir; and both.

  Each wiring is constructed with exactly the pins it has, so no pin operation
  ever tests how the board is wired.

SWD clock

  The clock starts at --clock and stays there only until the client picks one:
  OpenOCD sends its own during init, in response to "adapter speed".

  Clock edges are paced by a counted delay loop, never by sleeping or polling a
  clock: a time.Now() call per edge costs more than the edge does. A requested
  clock in kHz becomes ceil(speed_coeff / khz) - speed_offset loop iterations
  per SWCLK half period, so the fastest reachable clock is
  speed_coeff / speed_offset kHz.

  Both backends report the same pair, so a chip:line run is directly comparable
  to a register-backend one: what changes is the offset, which is an order of
  magnitude larger when an edge is a syscall, and says exactly where the ceiling
  is.

  These are the same two numbers as OpenOCD's "bcm2835gpio speed_coeffs" -- but
  they describe the cost of this binary's compiled loop, not a C one's, so
  values measured for the C gateway do not carry over. That is why this
  measures its own at startup rather than shipping a table of defaults; the
  measurement takes well under a second. Pass --speed-coeff and --speed-offset
  together to use values you measured yourself and skip it.

  Calibration is also available at runtime as the Calibrate vendor command
  (0x91), which re-measures, applies the result at the clock currently in
  effect, and reports the two numbers back. It is the only vendor command this
  gateway implements; the rest of the ID_DAP_Vendor0..31 range answers
  ID_DAP_Invalid.`

func main() {
	var opt options

	root := &cobra.Command{
		Use:           "swd-gateway",
		Short:         "Bitbanged SWD to CMSIS-DAP gateway",
		Long:          longHelp,
		Args:          cobra.NoArgs,
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			if opt.listChips {
				return listChips()
			}
			f := cmd.Flags()
			opt.gaveSWDIO = f.Changed("swdio")
			opt.gaveSWDIOSplit = f.Changed("swdio-in") || f.Changed("swdio-out")
			opt.gaveSWDIODir = f.Changed("swdio-dir")
			opt.gaveCoeffs = f.Changed("speed-coeff") || f.Changed("speed-offset")
			return run(cmd.Context(), &opt)
		},
	}

	f := root.Flags()
	f.StringVarP(&opt.bind, "bind", "b", "", "address to listen on (default: all interfaces)")
	f.IntVarP(&opt.port, "port", "p", defaultPort, "TCP port to listen on")
	f.StringVar(&opt.unixSocket, "unix-socket", "",
		"listen on this Unix domain socket instead of TCP, for a client on the same "+
			"machine; skips the TCP/IP stack entirely, and is not reachable from OpenOCD, "+
			"which needs a real network endpoint")
	f.StringVarP(&opt.logLevel, "log-level", "l", "info", "error, warn, info or debug")

	f.StringVar(&opt.driver, "driver", driverBCM2835,
		"GPIO backend: bcm2835 (registers, Pi 1-4) or linuxgpiod (character device, any board)")
	f.BoolVar(&opt.listChips, "list-gpiochips", false, "list this machine's GPIO chips and exit")

	f.StringVar(&opt.swclk, "swclk", itoa(defaultSWCLK), "SWCLK pin (BCM numbering), or chip:line")
	f.StringVar(&opt.swdio, "swdio", itoa(defaultSWDIOOut), "single bidirectional SWDIO pin, wired straight to the target")
	f.StringVar(&opt.swdioIn, "swdio-in", itoa(defaultSWDIOIn), "SWDIO in, split wiring")
	f.StringVar(&opt.swdioOut, "swdio-out", itoa(defaultSWDIOOut), "SWDIO out, split wiring")
	f.StringVar(&opt.swdioDir, "swdio-dir", itoa(defaultSWDIODir), "buffer direction pin")

	f.Uint32Var(&opt.clockHz, "clock", defaultClockHz, "startup SWD clock, in Hz")
	f.Uint32Var(&opt.speedCoeff, "speed-coeff", 0, "delay-loop iterations per kHz (default: measured at startup)")
	f.Uint32Var(&opt.speedOffset, "speed-offset", 0, "fixed per-transition cost, in the same iterations (default: measured at startup)")

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	if err := root.ExecuteContext(ctx); err != nil {
		fmt.Fprintln(os.Stderr, "swd-gateway:", err)
		os.Exit(1)
	}
}

func run(ctx context.Context, opt *options) error {
	log, err := newLogger(opt.logLevel)
	if err != nil {
		return err
	}
	// Nothing this binary uses logs through slog.Default today. Pointing it
	// here anyway means that if something ever does, it lands in the same
	// stream and the same format as everything else -- which under systemd is
	// the difference between a record the journal can sort by priority and one
	// it cannot.
	slog.SetDefault(log)

	if err := checkDriver(opt); err != nil {
		return err
	}
	if err := checkWiring(opt); err != nil {
		return err
	}
	if err := validateCoeffs(opt); err != nil {
		return err
	}

	d, wiring, err := openDriver(opt)
	if err != nil {
		return err
	}
	defer d.Close()

	if err := calibrate(opt, d, log); err != nil {
		return err
	}

	probe := bitbang.New(d)
	defer probe.Close(ctx)

	if err := probe.SetClock(ctx, opt.clockHz); err != nil {
		return fmt.Errorf("set startup clock: %w", err)
	}

	log.Info("swd gateway",
		"pins", wiring,
		"clock_hz", opt.clockHz,
	)
	if g, ok := d.(*linuxgpiod.Driver); ok && !g.Fused() {
		log.Warn("swdio and swclk are on different gpiochips, so they cannot share an ioctl; "+
			"a written bit costs three syscalls instead of two",
			"pins", wiring)
	}

	// The server is a packet codec in front of the probe; all the SWD logic is
	// in the probe itself, which is why this is three lines.
	dap := server.New(ctx, probe, server.Options{
		Product:      "swd-to-go gateway",
		HandleVendor: (&calibrateHandler{cal: d, log: log}).HandleVendor,
	})

	srv, where := newServer(opt, dap, log)
	log.Info("listening", "on", where)

	if err := srv.Start(ctx); err != nil && ctx.Err() == nil {
		return err
	}
	return nil
}

// Driver is what every backend and wiring is: a bit-banged SWD driver that can
// calibrate its own delay loop.
//
// This interface costs nothing on the wire. It is used to hand the driver to
// bitbang.New once and to reach Calibrate; every pin operation inside a
// transfer is a concrete call on the driver's own type, which is why the
// backends duplicate the protocol and each wiring is its own type.
type Driver interface {
	swd.BitBanger
	swd.Calibrator
}

// openDriver picks the backend and the wiring.
//
// Each constructor takes exactly the pins its wiring has -- there is no config
// struct with a field that does not apply, and no driver carrying a flag saying
// which of its fields are real. None of them calibrates: that happens in
// calibrate() below, once the caller has had its say.
//
// The two backends are interchangeable from here on, because both are a
// swd.BitBanger and a swd.Calibrator and nothing below this function knows
// which one it got.
func openDriver(opt *options) (Driver, string, error) {
	if opt.driver == driverLinuxGPIOD {
		d, err := openGpiod(opt)
		if err != nil {
			return nil, "", err
		}
		// Where each pin actually landed, not what was asked for: naming the
		// wrong chip is the whole risk of this backend.
		return d, d.Wiring(), nil
	}
	d, err := openBCM(opt)
	if err != nil {
		return nil, "", err
	}
	return d, opt.wiring() + " on /dev/gpiomem", nil
}

func openGpiod(opt *options) (*linuxgpiod.Driver, error) {
	pins, err := opt.pins()
	if err != nil {
		return nil, err
	}
	switch {
	case opt.bidir && opt.gaveSWDIODir:
		return linuxgpiod.OpenBidirBufferedPins(pins.swclk, pins.swdio, pins.swdioDir)
	case opt.bidir:
		return linuxgpiod.OpenBidirPins(pins.swclk, pins.swdio)
	default:
		return linuxgpiod.OpenSplitPins(pins.swclk, pins.swdioIn, pins.swdioOut, pins.swdioDir)
	}
}

func openBCM(opt *options) (Driver, error) {
	n, err := opt.pinNumbers()
	if err != nil {
		return nil, err
	}
	switch {
	case opt.bidir && opt.gaveSWDIODir:
		return bcm2835.OpenBidirBuffered(n.swclk, n.swdio, n.swdioDir)
	case opt.bidir:
		return bcm2835.OpenBidir(n.swclk, n.swdio)
	default:
		return bcm2835.OpenSplit(n.swclk, n.swdioIn, n.swdioOut, n.swdioDir)
	}
}

// pinSet and pinNumberSet are the pin flags once parsed, in the two shapes the
// two backends take them in.
type pinSet struct {
	swclk, swdio, swdioIn, swdioOut, swdioDir linuxgpiod.Pin
}

type pinNumberSet struct {
	swclk, swdio, swdioIn, swdioOut, swdioDir int
}

// pinFlag is one pin flag: what it is called and what was passed to it.
type pinFlag struct{ name, spec string }

// usedPins lists the pin flags this wiring actually reads.
//
// Only these are parsed, and only these decide anything. A value left in a flag
// the wiring does not use -- --swdio-in still holding its default while --swdio
// selects the one-pin wiring -- must not fail the run or pick a backend, which
// is what parsing all five unconditionally would do.
func (opt *options) usedPins() []pinFlag {
	out := []pinFlag{{"swclk", opt.swclk}}
	if opt.bidir {
		out = append(out, pinFlag{"swdio", opt.swdio})
		if opt.gaveSWDIODir {
			out = append(out, pinFlag{"swdio-dir", opt.swdioDir})
		}
		return out
	}
	return append(out,
		pinFlag{"swdio-in", opt.swdioIn},
		pinFlag{"swdio-out", opt.swdioOut},
		pinFlag{"swdio-dir", opt.swdioDir})
}

// checkDriver rejects a backend that does not exist, before anything touches
// hardware.
func checkDriver(opt *options) error {
	switch opt.driver {
	case driverBCM2835, driverLinuxGPIOD:
		return nil
	}
	return fmt.Errorf("unknown --driver %q, want %s (GPIO registers, Pi 1-4) or %s "+
		"(the Linux GPIO character device, any board)", opt.driver, driverBCM2835, driverLinuxGPIOD)
}

// pins parses the flags for the character-device backend, where every pin is
// written chip:line.
//
// A bare line number is refused rather than defaulted onto some chip this
// program picked. There is no default chip, because the pin that would land on
// it is exactly the one worth being sure about.
func (opt *options) pins() (pinSet, error) {
	var out pinSet
	for _, f := range opt.usedPins() {
		p, err := linuxgpiod.ParsePin(f.spec)
		if err != nil {
			return pinSet{}, fmt.Errorf("--%s: %w", f.name, err)
		}
		if p.Chip == "" {
			return pinSet{}, fmt.Errorf("--%s is %q, but --driver %s needs every pin to name "+
				"its chip: write it as chip:line (--list-gpiochips shows what there is)",
				f.name, f.spec, driverLinuxGPIOD)
		}
		switch f.name {
		case "swclk":
			out.swclk = p
		case "swdio":
			out.swdio = p
		case "swdio-in":
			out.swdioIn = p
		case "swdio-out":
			out.swdioOut = p
		case "swdio-dir":
			out.swdioDir = p
		}
	}
	return out, nil
}

// pinNumbers parses the flags for the register backend, which drives the SoC's
// own GPIO and has no chips to choose between.
func (opt *options) pinNumbers() (pinNumberSet, error) {
	var out pinNumberSet
	for _, f := range opt.usedPins() {
		p, err := linuxgpiod.ParsePin(f.spec)
		if err != nil {
			return pinNumberSet{}, fmt.Errorf("--%s: %w", f.name, err)
		}
		if p.Chip != "" {
			return pinNumberSet{}, fmt.Errorf("--%s is %q, but --driver %s drives the SoC's own "+
				"GPIO and takes a bare line number; --driver %s is the one with chips to choose "+
				"between", f.name, f.spec, driverBCM2835, driverLinuxGPIOD)
		}
		switch f.name {
		case "swclk":
			out.swclk = p.Line
		case "swdio":
			out.swdio = p.Line
		case "swdio-in":
			out.swdioIn = p.Line
		case "swdio-out":
			out.swdioOut = p.Line
		case "swdio-dir":
			out.swdioDir = p.Line
		}
	}
	return out, nil
}

// calibrate settles the delay loop before any target is attached, which is the
// safe moment for it: measuring clocks SWCLK, and doing that mid-session would
// put spurious edges on a live bus.
// wiring describes the pins for the log line. The character-device backend
// describes its own, because only it knows which chip each pin resolved to.
func (opt *options) wiring() string {
	switch {
	case opt.bidir && opt.gaveSWDIODir:
		return fmt.Sprintf("swclk %s, bidirectional swdio %s behind a buffer, dir %s",
			opt.swclk, opt.swdio, opt.swdioDir)
	case opt.bidir:
		return fmt.Sprintf("swclk %s, bidirectional swdio %s, no direction pin",
			opt.swclk, opt.swdio)
	default:
		return fmt.Sprintf("swclk %s, swdio in %s out %s, dir %s",
			opt.swclk, opt.swdioIn, opt.swdioOut, opt.swdioDir)
	}
}

// listChips answers --list-gpiochips. Which chip the header pins are on has
// moved between kernel versions and differs between boards, and guessing a
// number is how a gateway ends up wiggling something that is not the target.
func listChips() error {
	chips, err := linuxgpiod.Chips()
	if err != nil {
		return err
	}
	if len(chips) == 0 {
		return fmt.Errorf("no GPIO chips this user can open (need root, or membership of the gpio group)")
	}
	for _, c := range chips {
		fmt.Printf("%-12s %-24s %3d lines  %s\n", c.Name, c.Label, c.Lines, c.Path)
	}
	return nil
}

func itoa(n int) string { return strconv.Itoa(n) }

func calibrate(opt *options, d Driver, log *slog.Logger) error {
	if opt.gaveCoeffs {
		if err := d.SetSpeedCoeffs(opt.speedCoeff, opt.speedOffset); err != nil {
			return err
		}
	} else if _, _, err := d.Calibrate(); err != nil {
		return fmt.Errorf("calibrate: %w", err)
	}

	coeff, offset := d.SpeedCoeffs()
	if offset == 0 {
		return fmt.Errorf("calibration produced nothing usable")
	}
	source := "measured at startup"
	if opt.gaveCoeffs {
		source = "flags"
	}
	log.Info("swd clock calibration", "source", source,
		"speed_coeff", coeff, "speed_offset", offset, "max_khz", coeff/offset)
	return nil
}

// checkWiring rejects the flag combinations that cannot mean anything, and
// settles which wiring was asked for. The pin-level checks are the driver's own.
func checkWiring(opt *options) error {
	if opt.gaveSWDIO && opt.gaveSWDIOSplit {
		return fmt.Errorf("--swdio is one bidirectional pin; it cannot be combined with --swdio-in/--swdio-out")
	}
	opt.bidir = opt.gaveSWDIO
	return nil
}

// validateCoeffs checks the calibration flags before anything touches the
// hardware, so a typo fails immediately rather than after the GPIO is claimed.
func validateCoeffs(opt *options) error {
	if !opt.gaveCoeffs {
		return nil
	}
	if opt.speedCoeff == 0 || opt.speedOffset == 0 {
		return fmt.Errorf("--speed-coeff and --speed-offset go together; give both or neither")
	}
	if opt.speedOffset >= opt.speedCoeff {
		// The fastest reachable clock is coeff/offset kHz, so an offset at or
		// above the coefficient asks for an unbounded one.
		return fmt.Errorf("--speed-offset (%d) must be below --speed-coeff (%d)",
			opt.speedOffset, opt.speedCoeff)
	}
	return nil
}

func newServer(opt *options, dap servertcp.Handler, log *slog.Logger) (*servertcp.Server, string) {
	// The TCP server predates log/slog and takes a *log.Logger; route it into
	// the same handler at debug level so one -l flag governs everything.
	legacy := slog.NewLogLogger(log.Handler(), slog.LevelDebug)

	if opt.unixSocket != "" {
		return servertcp.NewUnixServer(opt.unixSocket, dap, legacy), "unix " + opt.unixSocket
	}
	addr := fmt.Sprintf("%s:%d", opt.bind, opt.port)
	return servertcp.NewServer(addr, dap, legacy), "tcp " + addr
}

// newLogger builds the logger everything here shares, at the level named on
// the command line, writing to stderr.
//
// Under systemd it writes for the journal instead of for a terminal: no
// timestamp of its own, and a syslog priority on the front of each line so
// that `journalctl -p` can tell the records apart. See journal.go.
func newLogger(level string) (*slog.Logger, error) {
	var l slog.Level
	switch level {
	case "error":
		l = slog.LevelError
	case "warn":
		l = slog.LevelWarn
	case "info":
		l = slog.LevelInfo
	case "debug":
		l = slog.LevelDebug
	default:
		return nil, fmt.Errorf("unknown log level %q, want error, warn, info or debug", level)
	}

	if underJournal(os.Stderr) {
		return slog.New(newJournalHandler(os.Stderr, l)), nil
	}
	return slog.New(slog.NewTextHandler(os.Stderr, &slog.HandlerOptions{Level: l})), nil
}
