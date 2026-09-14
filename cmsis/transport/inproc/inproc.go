// Package inproc carries CMSIS-DAP packets to a server in the same process,
// with no socket and no copy.
//
// It exists to test the packet codec against the server end to end, and to let
// a program hold a CMSIS-DAP view of a probe it already has. It is not the way
// to drive locally wired hardware: for that, hand the bitbang probe to the
// layers above directly and skip the encoding entirely.
package inproc

import (
	"context"
	"fmt"

	"github.com/kvth/swd-to-go/cmsis/client"
)

// Server is the part of cmsis/server.Server this transport needs.
type Server interface {
	ProcessCmd(request, response []byte) (consumed, produced int)
	MaxPacketSize() int
}

// Transport is a [client.Transport] that calls straight into a server.
type Transport struct {
	srv Server
}

var _ client.Transport = (*Transport)(nil)

// Open builds a probe talking to a server in this process.
func Open(ctx context.Context, srv Server) (*client.Probe, error) {
	return client.New(ctx, New(srv))
}

// New wraps a server as a transport.
func New(srv Server) *Transport { return &Transport{srv: srv} }

func (t *Transport) MaxPacketSize() int { return t.srv.MaxPacketSize() }

func (t *Transport) Close() error { return nil }

func (t *Transport) Exchange(ctx context.Context, request, response []byte) (int, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	consumed, produced := t.srv.ProcessCmd(request, response)
	if consumed <= 0 {
		return 0, fmt.Errorf("inproc: server consumed %d request bytes", consumed)
	}
	if produced <= 0 || produced > len(response) {
		return 0, fmt.Errorf("inproc: server produced %d response bytes into a %d byte buffer", produced, len(response))
	}
	return produced, nil
}
