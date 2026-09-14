// Command rtt-tcp tails a target's RTT channel through a CMSIS-DAP probe
// reached over a socket — swd-gateway on a Pi, OpenOCD, or any other probe
// speaking the same framing.
//
// It is examples/rtt-local with one line changed: the probe comes from
// tcp.Dial instead of from a GPIO driver. Everything after that — bring-up, DP,
// MEM-AP, RTT — is byte for byte the same code, because it is written against
// swd.Probe and cannot tell the two apart.
//
// Needs nothing but something at the other end of the socket.
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
	"github.com/kvth/swd-to-go/cmsis/transport/tcp"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
	"github.com/kvth/swd-to-go/target/rp2040"
	"github.com/kvth/swd-to-go/target/rtt"
)

func main() {
	var (
		addr     = flag.String("probe", "127.0.0.1:4441", "CMSIS-DAP probe address")
		unix     = flag.String("unix", "", "Unix socket path, instead of -probe")
		clock    = flag.Uint("clock", 1_500_000, "SWD clock in Hz")
		channel  = flag.Int("channel", 1, "RTT up channel to read")
		searchAt = flag.Uint("search", 0x20000000, "where to start looking for the RTT control block")
	)
	flag.Parse()

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer cancel()

	if err := run(ctx, *addr, *unix, uint32(*clock), *channel, uint32(*searchAt)); err != nil {
		log.Fatal(err)
	}
}

func run(ctx context.Context, addr, unix string, clockHz uint32, channel int, searchAt uint32) error {
	// The one line that differs from examples/rtt-local.
	var (
		probe swd.Probe
		err   error
	)
	if unix != "" {
		probe, err = tcp.DialUnix(ctx, unix)
	} else {
		probe, err = tcp.Dial(ctx, addr)
	}
	if err != nil {
		return err
	}
	defer probe.Close(ctx)

	// A CMSIS-DAP probe can say what it is. A bit-banged one cannot, which is
	// why this is a type assertion and not a method on swd.Probe.
	if id, ok := probe.(swd.Cmsis); ok {
		if product, err := id.ProductID(ctx); err == nil {
			fmt.Fprintf(os.Stderr, "probe: %s\n", product)
		}
	}

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
			continue
		}
		select {
		case <-ctx.Done():
		case <-time.After(10 * time.Millisecond):
		}
	}
	return nil
}
