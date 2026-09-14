package tcp_test

import (
	"bytes"
	"context"
	"io"
	"log"
	"net"
	"path/filepath"
	"testing"
	"time"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/cmsis/server"
	servertcp "github.com/kvth/swd-to-go/cmsis/server/tcp"
	clienttcp "github.com/kvth/swd-to-go/cmsis/transport/tcp"
	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
	clientrp2040 "github.com/kvth/swd-to-go/target/rp2040"
	clientrtt "github.com/kvth/swd-to-go/target/rtt"
)

// These run the whole stack over a real socket: the client's framing, the
// server's, the DAP command processing and the transfer layer, against a
// simulated debug port and MEM-AP. No Pi, no wire and no board.

const (
	ramBase = 0x20000000
	ramSize = 0x8000
)

var layout = simtarget.RTTLayout{
	CBAddr:   ramBase + 0x1000,
	NameAddr: ramBase + 0x0100,
	UpAddr:   ramBase + 0x2000,
	DownAddr: ramBase + 0x3000,
	BufSize:  1024,
}

// serve starts a gateway on addr and returns a client connected to it, with the
// debug port brought up and the MEM-AP ready.
func serve(t *testing.T, network, addr string, target *simtarget.Target) (swd.Probe, *dp.Client, *mem.Client) {
	t.Helper()

	d := server.New(context.Background(), bitbang.New(target), server.Options{})

	quiet := log.New(io.Discard, "", 0)
	var srv *servertcp.Server
	if network == "unix" {
		srv = servertcp.NewUnixServer(addr, d, quiet)
	} else {
		srv = servertcp.NewServer(addr, d, quiet)
	}

	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() {
		defer close(done)
		_ = srv.Start(ctx)
	}()
	t.Cleanup(func() {
		cancel()
		<-done
	})

	client := dial(t, network, addr)

	bg := context.Background()
	if err := client.Connect(bg); err != nil {
		t.Fatalf("DAP_Connect: %v", err)
	}

	dpc := dp.New(client)
	if err := dpc.Init(bg); err != nil {
		t.Fatalf("DP init: %v", err)
	}
	if err := dpc.SetDbgPower(bg, true, true); err != nil {
		t.Fatalf("power up: %v", err)
	}

	memc := mem.New(dpc, 0)
	if err := memc.Init(bg); err != nil {
		t.Fatalf("MEM-AP init: %v", err)
	}

	return client, dpc, memc
}

func dial(t *testing.T, network, addr string) swd.Probe {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for {
		c, err := clienttcp.DialNet(context.Background(), network, addr)
		if err == nil {
			return c
		}
		if time.Now().After(deadline) {
			t.Fatalf("dial %s %s: %v", network, addr, err)
		}
		time.Sleep(20 * time.Millisecond)
	}
}

// freeAddr asks the kernel for a port nothing else is on.
func freeAddr(t *testing.T) string {
	t.Helper()
	l, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("reserve a port: %v", err)
	}
	addr := l.Addr().String()
	l.Close()
	return addr
}

func TestTargetMemoryOverTCP(t *testing.T) {
	target := simtarget.New(simtarget.Options{},
		&simtarget.Region{Base: ramBase, Data: make([]byte, ramSize)})

	_, _, memc := serve(t, "tcp", freeAddr(t), target)
	ctx := context.Background()

	// A run long enough to cross the MEM-AP's 1kB auto-increment boundary,
	// which is where the client has to reload TAR rather than let it carry.
	words := make([]uint32, 512)
	for i := range words {
		words[i] = uint32(i*2654435761) | 1
	}
	if err := memc.WriteTargetMem(ctx, ramBase+0x300, words); err != nil {
		t.Fatalf("WriteTargetMem: %v", err)
	}

	got, err := memc.ReadTargetMem(ctx, ramBase+0x300, len(words))
	if err != nil {
		t.Fatalf("ReadTargetMem: %v", err)
	}
	for i := range words {
		if got[i] != words[i] {
			t.Fatalf("word %d = 0x%08x, want 0x%08x", i, got[i], words[i])
		}
	}
}

func TestUnalignedTargetMemoryOverTCP(t *testing.T) {
	target := simtarget.New(simtarget.Options{},
		&simtarget.Region{Base: ramBase, Data: make([]byte, ramSize)})

	_, _, memc := serve(t, "tcp", freeAddr(t), target)
	ctx := context.Background()

	want := []byte("an unaligned run of bytes")
	if err := memc.WriteTargetMemBytes(ctx, ramBase+0x401, want); err != nil {
		t.Fatalf("WriteTargetMemBytes: %v", err)
	}

	got, err := memc.ReadTargetMemBytes(ctx, ramBase+0x401, len(want))
	if err != nil {
		t.Fatalf("ReadTargetMemBytes: %v", err)
	}
	if !bytes.Equal(got, want) {
		t.Fatalf("read %q, want %q", got, want)
	}
}

func TestRTTOverTCP(t *testing.T) {
	target := simtarget.New(simtarget.Options{},
		&simtarget.Region{Base: ramBase, Data: make([]byte, ramSize)})

	block := simtarget.NewRTTBlock(target, layout, "Terminal")
	block.StartEcho(nil)
	t.Cleanup(block.Stop)

	_, _, memc := serve(t, "tcp", freeAddr(t), target)
	ctx := context.Background()

	addr, err := clientrtt.FindRTTControlBlock(ctx, memc, ramBase, ramSize, 0x200)
	if err != nil {
		t.Fatalf("FindRTTControlBlock: %v", err)
	}
	if addr != layout.CBAddr {
		t.Fatalf("control block at 0x%08x, want 0x%08x", addr, layout.CBAddr)
	}

	r, err := clientrtt.New(ctx, memc, addr)
	if err != nil {
		t.Fatalf("NewRTT: %v", err)
	}

	if n, err := r.Write(ctx, 0, []byte("over tcp")); err != nil || n != len("over tcp") {
		t.Fatalf("Write: n=%d err=%v", n, err)
	}

	// The simulated firmware answers what it is sent, in upper case.
	var got []byte
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) && len(got) < len("OVER TCP") {
		chunk, err := r.Read(ctx, 0)
		if err != nil {
			t.Fatalf("Read: %v", err)
		}
		got = append(got, chunk...)
	}
	if string(got) != "OVER TCP" {
		t.Fatalf("read %q, want %q", got, "OVER TCP")
	}
}

func TestRTTOverAUnixSocket(t *testing.T) {
	target := simtarget.New(simtarget.Options{},
		&simtarget.Region{Base: ramBase, Data: make([]byte, ramSize)})

	block := simtarget.NewRTTBlock(target, layout, "Terminal")
	block.StartEcho(nil)
	t.Cleanup(block.Stop)

	// A client on the same machine can skip the TCP/IP stack entirely.
	_, _, memc := serve(t, "unix", filepath.Join(t.TempDir(), "gateway.sock"), target)
	ctx := context.Background()

	r, err := clientrtt.New(ctx, memc, layout.CBAddr)
	if err != nil {
		t.Fatalf("NewRTT: %v", err)
	}
	if _, err := r.Write(ctx, 0, []byte("unix")); err != nil {
		t.Fatalf("Write: %v", err)
	}

	var got []byte
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) && len(got) < len("UNIX") {
		chunk, err := r.Read(ctx, 0)
		if err != nil {
			t.Fatalf("Read: %v", err)
		}
		got = append(got, chunk...)
	}
	if string(got) != "UNIX" {
		t.Fatalf("read %q, want %q", got, "UNIX")
	}
}

func TestRP2040FlashOverTCP(t *testing.T) {
	target, chip := simtarget.NewRP2040(simtarget.Options{}, 256*1024)

	client, dpc, memc := serve(t, "tcp", freeAddr(t), target)
	ctx := context.Background()

	f := clientrp2040.NewFlasher(client, dpc, memc, 0, 0, 0, 64*1024)
	if err := f.Halt(ctx, time.Second); err != nil {
		t.Fatalf("Halt: %v", err)
	}
	if err := f.LoadJumpTable(ctx); err != nil {
		t.Fatalf("LoadJumpTable: %v", err)
	}
	if f.Trampoline() == 0 {
		t.Fatal("no debug trampoline in the jump table")
	}

	image := make([]byte, 8*1024)
	for i := range image {
		image[i] = byte(i*13 + 5)
	}

	if err := f.Prep(ctx); err != nil {
		t.Fatalf("Prep: %v", err)
	}
	if err := f.Erase(ctx, 0, len(image), nil); err != nil {
		t.Fatalf("Erase: %v", err)
	}
	if err := f.Program(ctx, 0, image, nil); err != nil {
		t.Fatalf("Program: %v", err)
	}
	if err := f.Finish(ctx); err != nil {
		t.Fatalf("Finish: %v", err)
	}

	if !bytes.Equal(chip.Flash[:len(image)], image) {
		t.Fatal("flash contents do not match the image")
	}
	// Verify reads back through the XIP window, which Finish is what restores.
	if err := f.Verify(ctx, 0, image, nil); err != nil {
		t.Fatalf("Verify: %v", err)
	}
}

func TestOnlyOneClientAtATime(t *testing.T) {
	target := simtarget.New(simtarget.Options{},
		&simtarget.Region{Base: ramBase, Data: make([]byte, ramSize)})

	addr := freeAddr(t)
	serve(t, "tcp", addr, target)

	// The gateway drives one wire, so it takes one client: a second connection
	// is dropped rather than interleaved with the first.
	conn, err := net.DialTimeout("tcp", addr, 2*time.Second)
	if err != nil {
		t.Fatalf("second dial: %v", err)
	}
	defer conn.Close()

	conn.SetReadDeadline(time.Now().Add(2 * time.Second))
	if _, err := conn.Read(make([]byte, 1)); err == nil {
		t.Fatal("the server kept a second client connected")
	}
}
