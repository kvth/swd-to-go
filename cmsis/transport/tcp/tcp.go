// Package tcp carries CMSIS-DAP packets over a stream socket.
//
// The framing is the eight-byte "DAP\0" header cmsis/server/tcp speaks:
// signature, payload length, packet type, one reserved byte. It is needed
// because TCP has no message boundaries of its own, and a full-size block
// transfer arrives in whatever pieces the network feels like.
package tcp

import (
	"bytes"
	"context"
	"encoding/binary"
	"fmt"
	"io"
	"net"
	"time"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/cmsis/client"
)

// HeaderLen is the framing in front of every payload.
const HeaderLen = 8

const (
	signature     = 0x00504144 // "DAP\0", little endian
	typeRequest   = 0x01
	typeResponse  = 0x02
	defaultDialTO = 3 * time.Second
	defaultIOTO   = 1 * time.Second
)

// Transport is a [client.Transport] over a stream connection.
type Transport struct {
	conn    net.Conn
	timeout time.Duration
	hdr     [HeaderLen]byte
}

var _ client.Transport = (*Transport)(nil)

// Dial opens a probe over TCP. It is the one-liner most callers want; use
// [NewTransport] if you need to configure the connection first.
func Dial(ctx context.Context, addr string) (*client.Probe, error) {
	return DialNet(ctx, "tcp", addr)
}

// DialUnix opens a probe over a Unix domain socket, which is what a caller on
// the same machine uses to skip the TCP/IP stack. The framing is identical.
func DialUnix(ctx context.Context, path string) (*client.Probe, error) {
	return DialNet(ctx, "unix", path)
}

// DialNet opens a probe over any network net.Dial understands.
func DialNet(ctx context.Context, network, addr string) (*client.Probe, error) {
	var d net.Dialer
	d.Timeout = defaultDialTO
	conn, err := d.DialContext(ctx, network, addr)
	if err != nil {
		return nil, fmt.Errorf("tcp: failed to connect to %s %s: %w", network, addr, err)
	}
	return client.New(ctx, New(conn))
}

// New wraps an already-open connection.
func New(conn net.Conn) *Transport {
	return &Transport{conn: conn, timeout: defaultIOTO}
}

// SetTimeout sets the deadline applied to each read and write. Zero disables
// deadlines, which is what you want if the far end can be slow on purpose.
func (t *Transport) SetTimeout(d time.Duration) { t.timeout = d }

// MaxPacketSize returns zero: the framing imposes no limit of its own, so
// whatever the probe reports wins.
func (t *Transport) MaxPacketSize() int { return 0 }

func (t *Transport) Close() error { return t.conn.Close() }

func (t *Transport) Exchange(ctx context.Context, request, response []byte) (int, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if len(request) == 0 {
		return 0, fmt.Errorf("tcp: empty request")
	}
	if len(request) > 0xffff {
		return 0, fmt.Errorf("tcp: request of %d bytes does not fit the framing", len(request))
	}

	var packet bytes.Buffer
	packet.Grow(HeaderLen + len(request))
	binary.Write(&packet, binary.LittleEndian, uint32(signature))
	binary.Write(&packet, binary.LittleEndian, uint16(len(request)))
	packet.WriteByte(typeRequest)
	packet.WriteByte(0)
	packet.Write(request)

	t.setDeadline(ctx, true)
	if _, err := t.conn.Write(packet.Bytes()); err != nil {
		return 0, fmt.Errorf("tcp: write failed: %w", err)
	}

	// Read the header first, then exactly the payload it declares.
	t.setDeadline(ctx, false)
	if _, err := io.ReadFull(t.conn, t.hdr[:]); err != nil {
		return 0, fmt.Errorf("tcp: read failed: %w", err)
	}
	if got := binary.LittleEndian.Uint32(t.hdr[0:4]); got != signature {
		return 0, fmt.Errorf("tcp: bad signature 0x%08x", got)
	}
	if got := t.hdr[6]; got != typeResponse {
		return 0, fmt.Errorf("tcp: unexpected packet type 0x%02x", got)
	}

	length := int(binary.LittleEndian.Uint16(t.hdr[4:6]))
	if length == 0 {
		return 0, fmt.Errorf("tcp: empty response payload")
	}
	if length > len(response) {
		// Drain it anyway, or the stream is left misaligned and every packet
		// after this one is garbage.
		io.CopyN(io.Discard, t.conn, int64(length))
		return 0, fmt.Errorf("tcp: response of %d bytes does not fit a %d byte buffer", length, len(response))
	}
	if _, err := io.ReadFull(t.conn, response[:length]); err != nil {
		return 0, fmt.Errorf("tcp: read failed: %w", err)
	}
	return length, nil
}

// setDeadline applies whichever of the context deadline and the transport
// timeout comes first, so a cancelled context actually unblocks the socket.
func (t *Transport) setDeadline(ctx context.Context, write bool) {
	var deadline time.Time
	if t.timeout > 0 {
		deadline = time.Now().Add(t.timeout)
	}
	if d, ok := ctx.Deadline(); ok && (deadline.IsZero() || d.Before(deadline)) {
		deadline = d
	}
	if write {
		t.conn.SetWriteDeadline(deadline)
		return
	}
	t.conn.SetReadDeadline(deadline)
}

// Interface check against the layer above, so a change to swd.Probe that this
// package's Dial no longer satisfies fails here rather than at a call site.
var _ swd.Probe = (*client.Probe)(nil)
