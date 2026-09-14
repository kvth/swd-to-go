package main

import (
	"bytes"
	"context"
	"io"
	"log/slog"
	"os"
	"strconv"
	"strings"
	"sync"
	"syscall"
)

// Running as a systemd unit, stderr is a pipe to the journal rather than a
// terminal, and the journal reads two things off it that a terminal does not.
//
// It stamps every line itself, so a timestamp of ours is a duplicate that
// `journalctl` shows twice. And it reads a leading "<N>" on a line as that
// line's syslog priority, which is the only way a plain stderr logger can tell
// it the difference between an error and a debug line — without one, every
// record lands at the unit's default priority and `journalctl -p err` finds
// nothing. The convention is sd-daemon's; see sd-daemon(3).
//
// So: under the journal, drop the timestamp and write the priority. Anywhere
// else, keep the timestamp and write no priority, because "<6>" in front of
// every line on a terminal is noise.

// journalPrefixes maps a slog level onto the syslog priority the journal reads
// off the front of a line. Levels below Debug — swd.LevelTrace — are debug too;
// the journal has nothing finer.
var journalPrefixes = []struct {
	min    slog.Level
	prefix string
}{
	{slog.LevelError, "<3>"}, // LOG_ERR
	{slog.LevelWarn, "<4>"},  // LOG_WARNING
	{slog.LevelInfo, "<6>"},  // LOG_INFO
	{slog.LevelDebug, "<7>"}, // LOG_DEBUG
}

func journalPrefix(l slog.Level) string {
	for _, p := range journalPrefixes {
		if l >= p.min {
			return p.prefix
		}
	}
	return "<7>"
}

// underJournal reports whether f is the stream systemd connected to the
// journal for this unit.
//
// systemd sets JOURNAL_STREAM to the device and inode of that stream. Matching
// it against the file we are about to log to is what distinguishes "this
// process is a unit and stderr goes to the journal" from "this process is a
// unit but the operator redirected stderr somewhere else", which is a
// distinction an environment variable alone cannot make.
func underJournal(f *os.File) bool {
	spec := os.Getenv("JOURNAL_STREAM")
	if spec == "" {
		return false
	}
	devText, inoText, ok := strings.Cut(spec, ":")
	if !ok {
		return false
	}
	dev, err := strconv.ParseUint(devText, 10, 64)
	if err != nil {
		return false
	}
	ino, err := strconv.ParseUint(inoText, 10, 64)
	if err != nil {
		return false
	}

	fi, err := f.Stat()
	if err != nil {
		return false
	}
	st, ok := fi.Sys().(*syscall.Stat_t)
	if !ok {
		return false
	}
	return uint64(st.Dev) == dev && uint64(st.Ino) == ino
}

// journalHandler writes each record with its syslog priority in front.
//
// The inner handler formats into a buffer rather than to the stream, so that
// the priority and the record it belongs to reach the journal as one write.
// Two writes would be two lines the moment anything else logged in between,
// and a "<3>" on a line of its own is a line of its own.
type journalHandler struct {
	inner slog.Handler
	out   io.Writer

	// Shared with every handler WithAttrs and WithGroup derive from this one:
	// they format into the same buffer, so they have to take the same lock.
	mu  *sync.Mutex
	buf *bytes.Buffer
}

// newJournalHandler wraps out in a handler that prefixes priorities. The
// records carry no timestamp, the journal having its own.
func newJournalHandler(out io.Writer, level slog.Leveler) slog.Handler {
	buf := &bytes.Buffer{}
	h := &journalHandler{out: out, mu: &sync.Mutex{}, buf: buf}
	h.inner = slog.NewTextHandler(buf, &slog.HandlerOptions{
		Level: level,
		ReplaceAttr: func(groups []string, a slog.Attr) slog.Attr {
			if len(groups) == 0 && a.Key == slog.TimeKey {
				return slog.Attr{}
			}
			return a
		},
	})
	return h
}

func (h *journalHandler) Enabled(ctx context.Context, l slog.Level) bool {
	return h.inner.Enabled(ctx, l)
}

func (h *journalHandler) Handle(ctx context.Context, r slog.Record) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	h.buf.Reset()
	h.buf.WriteString(journalPrefix(r.Level))
	if err := h.inner.Handle(ctx, r); err != nil {
		return err
	}
	_, err := h.out.Write(h.buf.Bytes())
	return err
}

func (h *journalHandler) WithAttrs(attrs []slog.Attr) slog.Handler {
	return &journalHandler{inner: h.inner.WithAttrs(attrs), out: h.out, mu: h.mu, buf: h.buf}
}

func (h *journalHandler) WithGroup(name string) slog.Handler {
	return &journalHandler{inner: h.inner.WithGroup(name), out: h.out, mu: h.mu, buf: h.buf}
}
