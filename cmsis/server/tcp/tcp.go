// Package tcp serves a cmsis/server over a stream socket, so a debugger such
// as OpenOCD can drive a probe this process holds.
//
// The framing is the eight-byte "DAP\0" header cmsis/transport/tcp speaks.
// One client is served at a time: a probe is a single piece of wire, and the
// second connection would interleave its transfers with the first one's.
package tcp

import (
	"bytes"
	"context"
	"encoding/binary"
	"errors"
	"io"
	"io/fs"
	"log"
	"net"
	"os"
	"sync"
	"syscall"
	"time"
)

// Handler processes one CMSIS-DAP packet. cmsis/server.Server is one.
type Handler interface {
	ProcessCmd(request []byte, response []byte) (consumed int, produced int)
}

const (
	DAPPktSize = 1024

	dapPktHdrSignature = 0x00504144 // "DAP\0" in LE
	dapPktTypeRequest  = 0x01
	dapPktTypeResponse = 0x02

	DefaultPort = 4441

	keepAliveIdle     = 1 * time.Second
	keepAliveInterval = 1 * time.Second
	keepAliveCount    = 10
)

const dapTotalPktSize = 8 + DAPPktSize

type packetHeader struct {
	Signature  uint32
	Length     uint16
	PacketType uint8
	Reserved   uint8
}

type msgbuf struct {
	data []byte
}

func newMsgbuf() *msgbuf {
	return &msgbuf{
		data: make([]byte, 0, 3*dapTotalPktSize),
	}
}

func (b *msgbuf) reset() {
	b.data = b.data[:0]
}

func (b *msgbuf) add(conn net.Conn) error {
	tmp := make([]byte, 4096)

	n, err := conn.Read(tmp)
	if err != nil {
		if ne, ok := err.(net.Error); ok && ne.Timeout() {
			return nil
		}
		return err
	}
	if n == 0 {
		return io.EOF
	}

	if len(b.data)+n > cap(b.data) {
		return syscall.ENOSPC
	}

	b.data = append(b.data, tmp[:n]...)
	return nil
}

func (b *msgbuf) parse() (hdr packetHeader, payload []byte, totalLen int, err error) {
	const hdrLen = 8

	if len(b.data) < hdrLen {
		return hdr, nil, 0, errWouldBlock
	}

	hdr.Signature = binary.LittleEndian.Uint32(b.data[0:4])
	hdr.Length = binary.LittleEndian.Uint16(b.data[4:6])
	hdr.PacketType = b.data[6]
	hdr.Reserved = b.data[7]

	if hdr.Signature != dapPktHdrSignature {
		return hdr, nil, 0, errors.New("invalid CMSIS-DAP TCP signature")
	}
	if hdr.PacketType != dapPktTypeRequest {
		return hdr, nil, 0, errors.New("invalid CMSIS-DAP TCP packet type")
	}
	if hdr.Length > DAPPktSize {
		return hdr, nil, 0, errors.New("request payload too large")
	}

	totalLen = hdrLen + int(hdr.Length)
	if len(b.data) < totalLen {
		return hdr, nil, 0, errWouldBlock
	}

	payload = b.data[hdrLen:totalLen]
	return hdr, payload, totalLen, nil
}

func (b *msgbuf) consume(n int) {
	if n <= 0 {
		return
	}
	if n >= len(b.data) {
		b.data = b.data[:0]
		return
	}
	copy(b.data, b.data[n:])
	b.data = b.data[:len(b.data)-n]
}

var errWouldBlock = errors.New("need more data")

type Server struct {
	// network and addr are what net.Listen is given: "tcp" with a host:port,
	// or "unix" with a socket path. Both are fixed by the constructor, so
	// Start never has to second-guess them.
	network string
	addr    string
	logger  *log.Logger

	handler Handler

	mu         sync.Mutex
	clientConn net.Conn
}

// NewServer serves the CMSIS-DAP TCP protocol on a TCP address.
func NewServer(addr string, d Handler, logger *log.Logger) *Server {
	if addr == "" {
		addr = ":4441"
	}
	return newServer("tcp", addr, d, logger)
}

// NewUnixServer serves the same protocol on a Unix domain socket, for a client
// on the same machine that wants to skip the TCP/IP stack. OpenOCD's
// "cmsis-dap backend tcp" needs a real network endpoint and cannot reach one of
// these; a Go client dialling "unix" can.
//
// A stale socket left behind by a process that did not shut down cleanly is
// removed at Start, so the path is taken over rather than refused.
func NewUnixServer(path string, d Handler, logger *log.Logger) *Server {
	return newServer("unix", path, d, logger)
}

func newServer(network, addr string, d Handler, logger *log.Logger) *Server {
	if logger == nil {
		logger = log.Default()
	}
	return &Server{
		network: network,
		addr:    addr,
		logger:  logger,
		handler: d,
	}
}

func (s *Server) Start(ctx context.Context) error {
	network := s.network

	lc := net.ListenConfig{
		Control: func(network, address string, c syscall.RawConn) error {
			if network == "unix" {
				return nil
			}
			var ctrlErr error
			if err := c.Control(func(fd uintptr) {
				ctrlErr = syscall.SetsockoptInt(int(fd), syscall.SOL_SOCKET, syscall.SO_REUSEADDR, 1)
			}); err != nil {
				return err
			}
			return ctrlErr
		},
	}

	if network == "unix" {
		// A socket file outlives the process that made it, so a crash would
		// otherwise leave the path permanently unbindable.
		if err := os.Remove(s.addr); err != nil && !errors.Is(err, fs.ErrNotExist) {
			return err
		}
	}

	ln, err := lc.Listen(ctx, network, s.addr)
	if err != nil {
		return err
	}
	defer ln.Close()

	s.logger.Printf("cmsis_dap_tcp: listening on %s %s", network, s.addr)

	go func() {
		<-ctx.Done()
		_ = ln.Close()
		s.closeClient()
	}()

	for {
		conn, err := ln.Accept()
		if err != nil {
			if ctx.Err() != nil {
				return nil
			}
			if isTemporary(err) {
				s.logger.Printf("cmsis_dap_tcp: accept temporary error: %v", err)
				time.Sleep(time.Second)
				continue
			}
			return err
		}

		if !s.trySetClient(conn) {
			s.logger.Printf("cmsis_dap_tcp: dropping new connection from %s, another client already connected", conn.RemoteAddr())
			_ = conn.Close()
			continue
		}

		s.logger.Printf("cmsis_dap_tcp: client connected %s", conn.RemoteAddr())

		go func() {
			defer s.closeClientConn(conn)
			if err := s.handleClient(ctx, conn); err != nil && !errors.Is(err, io.EOF) && ctx.Err() == nil {
				s.logger.Printf("cmsis_dap_tcp: client %s disconnected with error: %v", conn.RemoteAddr(), err)
			} else {
				s.logger.Printf("cmsis_dap_tcp: client disconnected %s", conn.RemoteAddr())
			}
		}()
	}
}

func (s *Server) handleClient(ctx context.Context, conn net.Conn) error {
	if tc, ok := conn.(*net.TCPConn); ok {
		_ = tc.SetKeepAlive(true)
		_ = tc.SetKeepAlivePeriod(keepAliveIdle)

		raw, err := tc.SyscallConn()
		if err == nil {
			_ = raw.Control(func(fd uintptr) {
				_ = syscall.SetsockoptInt(int(fd), syscall.IPPROTO_TCP, syscall.TCP_KEEPIDLE, int(keepAliveIdle/time.Second))
				_ = syscall.SetsockoptInt(int(fd), syscall.IPPROTO_TCP, syscall.TCP_KEEPINTVL, int(keepAliveInterval/time.Second))
				_ = syscall.SetsockoptInt(int(fd), syscall.IPPROTO_TCP, syscall.TCP_KEEPCNT, keepAliveCount)
			})
		}
	}

	buf := newMsgbuf()
	response := make([]byte, DAPPktSize)

	for {
		select {
		case <-ctx.Done():
			return nil
		default:
		}

		if err := conn.SetReadDeadline(time.Now().Add(30 * time.Second)); err != nil {
			return err
		}

		err := buf.add(conn)
		if err != nil {
			if errors.Is(err, syscall.ENOSPC) {
				return err
			}
			if ne, ok := err.(net.Error); ok && ne.Timeout() {
				continue
			}
			return err
		}

		for {
			_, payload, totalLen, err := buf.parse()
			if err != nil {
				if errors.Is(err, errWouldBlock) {
					break
				}
				return err
			}

			respLen, err := s.processDAPRequest(payload, response)
			buf.consume(totalLen)
			if err != nil {
				return err
			}

			if err := sendDAPResponse(conn, response[:respLen]); err != nil {
				return err
			}
		}
	}
}

func (s *Server) processDAPRequest(request []byte, response []byte) (int, error) {
	// ProcessCmd returns:
	//   upper 16 bits: request bytes consumed
	//   lower 16 bits: response bytes produced
	requestLen, responseLen := s.handler.ProcessCmd(request, response)
	s.logger.Printf("cmsis_dap_tcp: processed command req=%d resp=%d cmd=0x%02X", requestLen, responseLen, firstByte(request))

	if responseLen < 0 || responseLen > len(response) {
		return 0, errors.New("invalid response length from DAP")
	}

	return responseLen, nil
}

func sendDAPResponse(conn net.Conn, payload []byte) error {
	if len(payload) > DAPPktSize {
		return errors.New("response too large for buffer")
	}

	var hdr [8]byte
	binary.LittleEndian.PutUint32(hdr[0:4], dapPktHdrSignature)
	binary.LittleEndian.PutUint16(hdr[4:6], uint16(len(payload)))
	hdr[6] = dapPktTypeResponse
	hdr[7] = 0

	packet := bytes.NewBuffer(make([]byte, 0, len(hdr)+len(payload)))
	packet.Write(hdr[:])
	packet.Write(payload)

	remaining := packet.Bytes()
	for len(remaining) > 0 {
		n, err := conn.Write(remaining)
		if err != nil {
			if errors.Is(err, syscall.EINTR) {
				continue
			}
			return err
		}
		remaining = remaining[n:]
	}

	return nil
}

func (s *Server) trySetClient(conn net.Conn) bool {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.clientConn != nil {
		return false
	}
	s.clientConn = conn
	return true
}

func (s *Server) closeClient() {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.clientConn != nil {
		_ = s.clientConn.Close()
		s.clientConn = nil
	}
}

func (s *Server) closeClientConn(conn net.Conn) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.clientConn == conn {
		_ = s.clientConn.Close()
		s.clientConn = nil
	}
}

func isTemporary(err error) bool {
	var ne net.Error
	if errors.As(err, &ne) {
		return ne.Temporary()
	}
	return false
}

func firstByte(b []byte) byte {
	if len(b) == 0 {
		return 0
	}
	return b[0]
}
