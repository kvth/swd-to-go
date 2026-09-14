package mem_test

import (
	"bytes"
	"context"
	"log/slog"
	"strings"
	"testing"

	"github.com/kvth/swd-to-go"
	"github.com/kvth/swd-to-go/internal/simtarget"
	"github.com/kvth/swd-to-go/probe/bitbang"
	"github.com/kvth/swd-to-go/target/dp"
	"github.com/kvth/swd-to-go/target/mem"
)

// The register trace is the only thing these two packages log, and it is worth
// a test because nothing else would notice it breaking: an Enabled guard that
// stopped matching, or a level that drifted above the handler's, would show up
// as silence and pass every other test in the module.
func TestTrace(t *testing.T) {
	ctx := context.Background()
	target := simtarget.New(simtarget.Options{}, &simtarget.Region{Base: ramBase, Data: make([]byte, ramSize)})
	probe := bitbang.New(target)
	if err := probe.Connect(ctx); err != nil {
		t.Fatal(err)
	}

	var buf bytes.Buffer
	l := slog.New(slog.NewTextHandler(&buf, &slog.HandlerOptions{Level: swd.LevelTrace}))

	dpc := dp.New(probe)
	dpc.SetLogger(l)
	if err := dpc.Init(ctx); err != nil {
		t.Fatal(err)
	}
	if err := dpc.SetDbgPower(ctx, true, true); err != nil {
		t.Fatal(err)
	}
	apc := mem.New(dpc, 0)
	apc.SetLogger(l)
	if err := apc.Init(ctx); err != nil {
		t.Fatal(err)
	}
	if _, err := apc.ReadTargetReg(ctx, ramBase); err != nil {
		t.Fatal(err)
	}

	out := buf.String()
	for _, want := range []string{`level=DEBUG-4`, `msg="dp write"`, `msg="read reg" addr=0x20000000`} {
		if !strings.Contains(out, want) {
			t.Errorf("missing %q in:\n%s", want, out)
		}
	}
	// Nothing here is loud enough for an ordinary handler to see.
	var quiet bytes.Buffer
	dpc.SetLogger(slog.New(slog.NewTextHandler(&quiet, &slog.HandlerOptions{Level: slog.LevelInfo})))
	apc.SetLogger(slog.New(slog.NewTextHandler(&quiet, &slog.HandlerOptions{Level: slog.LevelInfo})))
	if _, err := apc.ReadTargetReg(ctx, ramBase); err != nil {
		t.Fatal(err)
	}
	if quiet.Len() != 0 {
		t.Errorf("logged at Info or above:\n%s", quiet.String())
	}
}
