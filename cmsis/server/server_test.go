package server_test

import (
	"context"
	"encoding/binary"
	"testing"

	"github.com/kvth/swd-to-go/cmsis/server"
	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
)

// newDAP builds the full stack the tests exercise: a simulated target, the
// bitbang engine driving it, and a CMSIS-DAP server in front of that. What the
// tests poke is packets in and target memory out, which is exactly the path a
// real debugger takes.
func newDAP(t *testing.T, regions ...*simtarget.Region) (*server.Server, *simtarget.Target) {
	t.Helper()
	target := simtarget.New(simtarget.Options{}, regions...)
	return server.New(context.Background(), bitbang.New(target), server.Options{}), target
}

func ram() *simtarget.Region {
	return &simtarget.Region{Base: 0x20000000, Data: make([]byte, 4096)}
}

// connect runs DAP_Connect the way a client would, which is what puts the port
// in SWD mode.
func connect(t *testing.T, d *server.Server) {
	t.Helper()
	resp := make([]byte, 1024)
	consumed, produced := d.ProcessCmd([]byte{0x02, 1}, resp)
	if consumed != 2 || produced != 2 || resp[1] != 1 {
		t.Fatalf("DAP_Connect: consumed=%d produced=%d port=%d", consumed, produced, resp[1])
	}
}

// writeReq builds a DAP_Transfer request for one register write.
func writeReq(ap bool, reg uint8, value uint32) []byte {
	req := reg & 0x0C
	if ap {
		req |= 1
	}
	out := []byte{0x05, 0x00, 0x01, req, 0, 0, 0, 0}
	binary.LittleEndian.PutUint32(out[4:], value)
	return out
}

// Each command reports how much of the request it consumed and how much
// response it produced, and the command ID counts towards both. Getting that
// wrong is invisible for a single command -- the transport eats the whole packet
// either way -- and breaks every batched one, so it is worth pinning down per
// command.
func TestCommandLengthAccounting(t *testing.T) {
	d, _ := newDAP(t, ram())
	resp := make([]byte, 1024)

	for _, c := range []struct {
		name               string
		request            []byte
		consumed, produced int
	}{
		{"DAP_Info", []byte{0x00, 0xFF}, 2, 4},  // id, field -> id, len, u16
		{"DAP_Connect", []byte{0x02, 1}, 2, 2},  // id, port -> id, port
		{"DAP_Disconnect", []byte{0x03}, 1, 2},  // id -> id, status
		{"DAP_Delay", []byte{0x09, 0, 0}, 3, 2}, // id, u16 -> id, status
		{"DAP_SWJ_Clock", []byte{0x11, 0x40, 0x42, 0x0F, 0}, 5, 2},
		{"DAP_SWD_Configure", []byte{0x13, 0}, 2, 2},
		{"DAP_TransferConfigure", []byte{0x04, 0, 0, 0, 0, 0}, 6, 2},
	} {
		consumed, produced := d.ProcessCmd(c.request, resp)
		if consumed != c.consumed || produced != c.produced {
			t.Errorf("%s: consumed=%d produced=%d, want %d and %d",
				c.name, consumed, produced, c.consumed, c.produced)
		}
	}
}

func TestExecuteCommandsBatch(t *testing.T) {
	d, _ := newDAP(t, ram())
	resp := make([]byte, 1024)

	// Three commands of different request lengths packed back to back. Each one
	// is read from where the previous one stopped, so a miscounted length
	// derails everything after it.
	request := []byte{
		0x7F, 3,
		0x02, 1, // DAP_Connect
		0x13, 0, // DAP_SWD_Configure
		0x09, 0, 0, // DAP_Delay
	}
	consumed, produced := d.ProcessCmd(request, resp)

	if consumed != len(request) {
		t.Fatalf("consumed %d of %d request bytes", consumed, len(request))
	}
	if produced != 2+3*2 {
		t.Fatalf("produced %d response bytes, want %d", produced, 2+3*2)
	}
	if resp[0] != 0x7F || resp[1] != 3 {
		t.Fatalf("batch header = % x", resp[:2])
	}
	// Each command echoed its own ID, which it only can if the one before it
	// reported its length correctly.
	for i, want := range []byte{0x02, 0x13, 0x09} {
		if got := resp[2+2*i]; got != want {
			t.Errorf("response %d echoed 0x%02x, want 0x%02x", i, got, want)
		}
	}
}

func TestBatchedTransfersReachTheTarget(t *testing.T) {
	d, target := newDAP(t, ram())
	connect(t, d)
	resp := make([]byte, 1024)

	// A SELECT write and a TAR write in one packet.
	sel := writeReq(false, 0x08, 0x00000000)
	tar := writeReq(true, 0x04, 0x20000100)

	request := append([]byte{0x7F, 2}, sel...)
	request = append(request, tar...)

	if consumed, _ := d.ProcessCmd(request, resp); consumed != len(request) {
		t.Fatalf("consumed %d of %d request bytes", consumed, len(request))
	}
	if got := target.TAR(); got != 0x20000100 {
		t.Fatalf("TAR = 0x%08x, want 0x20000100 -- the second command in the batch did not land",
			got)
	}
}

func TestQueueCommandsBehavesLikeExecute(t *testing.T) {
	d, _ := newDAP(t, ram())
	resp := make([]byte, 1024)

	consumed, produced := d.ProcessCmd([]byte{0x7E, 2, 0x02, 1, 0x03}, resp)
	if consumed != 5 || produced != 2+2*2 {
		t.Fatalf("consumed=%d produced=%d, want 5 and %d", consumed, produced, 2+2*2)
	}
}

func TestBatchStopsWhenTheResponsePacketRunsOut(t *testing.T) {
	d, _ := newDAP(t, ram())

	// Room for the batch header and two of the three two-byte responses.
	resp := make([]byte, 2+2*2)
	request := []byte{0x7F, 3, 0x03, 0x03, 0x03}

	_, produced := d.ProcessCmd(request, resp)
	if produced > len(resp) {
		t.Fatalf("produced %d bytes into a %d-byte response", produced, len(resp))
	}
}

func TestVendorCommandsAreNotClaimed(t *testing.T) {
	d, _ := newDAP(t, ram())
	resp := make([]byte, 1024)

	// What a vendor command means is a particular probe's business. This is a
	// CMSIS-DAP implementation, so the whole range answers ID_DAP_Invalid.
	for _, cmd := range []byte{0x80, 0x88, 0x9F} {
		consumed, produced := d.ProcessCmd([]byte{cmd, 0x01}, resp)
		if consumed != 1 || produced != 1 || resp[0] != 0xFF {
			t.Errorf("0x%02x: consumed=%d produced=%d resp[0]=0x%02x, want 1, 1 and 0xff",
				cmd, consumed, produced, resp[0])
		}
	}
}

func TestUnknownCommandIsInvalid(t *testing.T) {
	d, _ := newDAP(t, ram())
	resp := make([]byte, 1024)

	consumed, produced := d.ProcessCmd([]byte{0x77}, resp)
	if consumed != 1 || produced != 1 || resp[0] != 0xFF {
		t.Fatalf("consumed=%d produced=%d resp[0]=0x%02x", consumed, produced, resp[0])
	}
}

func TestEmptyRequestIsRefused(t *testing.T) {
	d, _ := newDAP(t, ram())

	if consumed, produced := d.ProcessCmd(nil, make([]byte, 16)); consumed != 0 || produced != 0 {
		t.Fatalf("consumed=%d produced=%d, want 0 and 0", consumed, produced)
	}
}

func TestTransferReadsAndWritesTheTarget(t *testing.T) {
	d, target := newDAP(t, ram())
	connect(t, d)
	resp := make([]byte, 1024)

	d.ProcessCmd(writeReq(false, 0x08, 0), resp)         // SELECT: AP 0, bank 0
	d.ProcessCmd(writeReq(true, 0x00, 0x23000052), resp) // CSW: word, auto-increment
	d.ProcessCmd(writeReq(true, 0x04, 0x20000000), resp) // TAR
	d.ProcessCmd(writeReq(true, 0x0C, 0xDEADBEEF), resp) // DRW

	if got := binary.LittleEndian.Uint32(target.ReadMem(0x20000000, 4)); got != 0xDEADBEEF {
		t.Fatalf("target memory = 0x%08x, want 0xdeadbeef", got)
	}

	// Read it back: an AP read is posted, so the value comes from DP RDBUFF.
	d.ProcessCmd(writeReq(true, 0x04, 0x20000000), resp)
	readAP := []byte{0x05, 0x00, 0x01, 0x01 | 0x02 | 0x0C} // AP read of DRW
	if _, produced := d.ProcessCmd(readAP, resp); produced < 7 {
		t.Fatalf("AP read produced %d bytes", produced)
	}
	if got := binary.LittleEndian.Uint32(resp[3:]); got != 0xDEADBEEF {
		t.Fatalf("read back 0x%08x, want 0xdeadbeef", got)
	}
}

func TestTransferBlockWritesARun(t *testing.T) {
	d, target := newDAP(t, ram())
	connect(t, d)
	resp := make([]byte, 1024)

	d.ProcessCmd(writeReq(false, 0x08, 0), resp)
	d.ProcessCmd(writeReq(true, 0x00, 0x23000052), resp)
	d.ProcessCmd(writeReq(true, 0x04, 0x20000200), resp)

	// DAP_TransferBlock: id, index, u16 count, request, then the data.
	block := []byte{0x06, 0x00, 0x04, 0x00, 0x01 | 0x0C}
	for i := 0; i < 4; i++ {
		var word [4]byte
		binary.LittleEndian.PutUint32(word[:], uint32(0x1000+i))
		block = append(block, word[:]...)
	}

	if _, produced := d.ProcessCmd(block, resp); produced < 4 {
		t.Fatalf("TransferBlock produced %d bytes", produced)
	}
	if count := binary.LittleEndian.Uint16(resp[1:]); count != 4 {
		t.Fatalf("TransferBlock completed %d of 4 transfers", count)
	}

	for i := 0; i < 4; i++ {
		got := binary.LittleEndian.Uint32(target.ReadMem(0x20000200+uint32(4*i), 4))
		if want := uint32(0x1000 + i); got != want {
			t.Errorf("word %d = 0x%08x, want 0x%08x", i, got, want)
		}
	}
}
