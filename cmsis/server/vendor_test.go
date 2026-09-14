package server_test

import (
	"context"
	"testing"

	"github.com/kvth/swd-to-go/cmsis/server"
	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
)

// stubVendor claims exactly one command and echoes a fixed reply, so the test
// can check the server's framing around a handler rather than any particular
// command's meaning.
type stubVendor struct {
	claim uint8
	calls int
}

func (v *stubVendor) HandleVendor(_ context.Context, cmd uint8, request, response []byte) (int, int, bool) {
	if cmd != v.claim {
		return 0, 0, false
	}
	v.calls++
	// Consume one request byte, produce three response bytes.
	response[0] = 0x00
	response[1] = request[0]
	response[2] = 0xAB
	return 1, 3, true
}

func newServerWithVendor(handle func(context.Context, uint8, []byte, []byte) (int, int, bool)) *server.Server {
	target := simtarget.New(simtarget.Options{},
		&simtarget.Region{Base: 0x20000000, Data: make([]byte, 4096)})
	return server.New(context.Background(), bitbang.New(target),
		server.Options{HandleVendor: handle})
}

func TestHandleVendorAnswersItsCommand(t *testing.T) {
	v := &stubVendor{claim: 0x91}
	d := newServerWithVendor(v.HandleVendor)
	resp := make([]byte, 64)

	consumed, produced := d.ProcessCmd([]byte{0x91, 0x42}, resp)

	// The command ID counts towards both halves on top of what the handler
	// reported, exactly as it does for a built-in command.
	if consumed != 2 || produced != 4 {
		t.Fatalf("consumed=%d produced=%d, want 2 and 4", consumed, produced)
	}
	if resp[0] != 0x91 {
		t.Errorf("echoed 0x%02x, want 0x91", resp[0])
	}
	if resp[1] != 0x00 || resp[2] != 0x42 || resp[3] != 0xAB {
		t.Errorf("payload = % x, want 00 42 ab", resp[1:4])
	}
	if v.calls != 1 {
		t.Errorf("handler called %d times, want 1", v.calls)
	}
}

func TestVendorCommandTheHandlerDeclinesIsInvalid(t *testing.T) {
	v := &stubVendor{claim: 0x91}
	d := newServerWithVendor(v.HandleVendor)
	resp := make([]byte, 64)

	// A different command in the same range. The handler does not claim it, so
	// the server refuses it rather than inventing a meaning.
	consumed, produced := d.ProcessCmd([]byte{0x92, 0x01}, resp)
	if consumed != 1 || produced != 1 || resp[0] != 0xFF {
		t.Fatalf("consumed=%d produced=%d resp[0]=0x%02x, want 1, 1 and 0xff",
			consumed, produced, resp[0])
	}
}

// A batch is where a miscounted vendor command does real damage: everything
// after it is read from the wrong offset.
func TestVendorCommandInABatch(t *testing.T) {
	v := &stubVendor{claim: 0x91}
	d := newServerWithVendor(v.HandleVendor)
	resp := make([]byte, 64)

	request := []byte{
		0x7F, 3,
		0x91, 0x42, // the vendor command: 2 request bytes
		0x02, 1, // DAP_Connect
		0x03, // DAP_Disconnect
	}
	consumed, _ := d.ProcessCmd(request, resp)
	if consumed != len(request) {
		t.Fatalf("consumed %d of %d request bytes", consumed, len(request))
	}
	// Each command echoed its own ID, which it only can if the one before it
	// reported its length correctly.
	for i, want := range []byte{0x91, 0x02, 0x03} {
		got := resp[2]
		switch i {
		case 1:
			got = resp[6] // after the vendor command's 4 response bytes
		case 2:
			got = resp[8]
		}
		if got != want {
			t.Errorf("response %d echoed 0x%02x, want 0x%02x", i, got, want)
		}
	}
}

// Without a handler the whole range is refused, which is the default and what
// a CMSIS-DAP implementation that is not a particular probe owes its clients.
func TestVendorRangeRefusedWithoutAHandler(t *testing.T) {
	d := newServerWithVendor(nil)
	resp := make([]byte, 64)

	for _, cmd := range []byte{0x80, 0x91, 0x9F} {
		consumed, produced := d.ProcessCmd([]byte{cmd, 0x01}, resp)
		if consumed != 1 || produced != 1 || resp[0] != 0xFF {
			t.Errorf("0x%02x: consumed=%d produced=%d resp[0]=0x%02x, want 1, 1 and 0xff",
				cmd, consumed, produced, resp[0])
		}
	}
}

// stubVendor deliberately does not check the length of what it was handed,
// which is the mistake a real handler makes eventually. The server has to
// contain it rather than let it take the process down.
func TestHandleVendorCannotOverrunTheResponse(t *testing.T) {
	v := &stubVendor{claim: 0x91}
	d := newServerWithVendor(v.HandleVendor)

	// Room for the echoed ID and one byte, but the handler writes three.
	resp := make([]byte, 2)
	consumed, produced := d.ProcessCmd([]byte{0x91, 0x42}, resp)
	if produced > len(resp) {
		t.Fatalf("produced %d bytes into a %d-byte response", produced, len(resp))
	}
	if consumed != 1 || resp[0] != 0xFF {
		t.Fatalf("consumed=%d resp[0]=0x%02x, want 1 and 0xff", consumed, resp[0])
	}
}
