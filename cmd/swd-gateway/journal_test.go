package main

import (
	"bytes"
	"context"
	"fmt"
	"io/fs"
	"log/slog"
	"os"
	"strings"
	"syscall"
	"testing"

	"github.com/kvth/swd-to-go"
)

func TestJournalHandlerPrefixesPriority(t *testing.T) {
	var out bytes.Buffer
	l := slog.New(newJournalHandler(&out, swd.LevelTrace))

	l.Error("boom", "code", 1)
	l.Warn("careful")
	l.Info("listening", "on", ":4441")
	l.Debug("detail")
	l.Log(context.Background(), swd.LevelTrace, "wire")
	l.With("unit", "swd").Info("grouped")

	lines := strings.Split(strings.TrimSuffix(out.String(), "\n"), "\n")
	want := []string{"<3>", "<4>", "<6>", "<7>", "<7>", "<6>"}
	if len(lines) != len(want) {
		t.Fatalf("got %d lines, want %d:\n%s", len(lines), len(want), out.String())
	}
	for i, w := range want {
		if !strings.HasPrefix(lines[i], w) {
			t.Errorf("line %d = %q, want prefix %q", i, lines[i], w)
		}
		// The journal stamps its own time; ours would be a duplicate.
		if strings.Contains(lines[i], "time=") {
			t.Errorf("line %d carries a timestamp: %q", i, lines[i])
		}
	}
	if !strings.Contains(lines[5], `unit=swd`) {
		t.Errorf("WithAttrs lost its attrs: %q", lines[5])
	}
}

func TestUnderJournal(t *testing.T) {
	t.Setenv("JOURNAL_STREAM", "")
	if underJournal(os.Stderr) {
		t.Error("no JOURNAL_STREAM, but reported as under the journal")
	}

	// A stream whose device and inode systemd claims are its own.
	f, err := os.CreateTemp(t.TempDir(), "stream")
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	fi, err := f.Stat()
	if err != nil {
		t.Fatal(err)
	}
	st := sysStat(t, fi)
	t.Setenv("JOURNAL_STREAM", st)
	if !underJournal(f) {
		t.Errorf("JOURNAL_STREAM=%s does not match the file it describes", st)
	}

	t.Setenv("JOURNAL_STREAM", "1:2")
	if underJournal(f) {
		t.Error("a JOURNAL_STREAM for a different stream matched")
	}
	t.Setenv("JOURNAL_STREAM", "nonsense")
	if underJournal(f) {
		t.Error("a malformed JOURNAL_STREAM matched")
	}
}

// sysStat spells a file's device and inode the way systemd spells
// JOURNAL_STREAM.
func sysStat(t *testing.T, fi fs.FileInfo) string {
	t.Helper()
	st, ok := fi.Sys().(*syscall.Stat_t)
	if !ok {
		t.Skip("no syscall.Stat_t on this platform")
	}
	return fmt.Sprintf("%d:%d", uint64(st.Dev), uint64(st.Ino))
}
