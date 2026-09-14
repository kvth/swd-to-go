package swd

import "log/slog"

// LevelTrace is the [log/slog] level this module's wire-level tracing logs at:
// every DP and MEM-AP register access, and every CMSIS-DAP packet in and out.
//
// It sits below [slog.LevelDebug] because it is orders of magnitude noisier
// than anything else here — one flash write is tens of thousands of these — so
// a caller that wants to see a driver's ordinary chatter should not have to
// take the register trace with it.
//
// Every call site guards itself with [slog.Logger.Enabled], so a handler that
// does not accept this level costs the hot path a comparison and no
// formatting. Turning it on means a handler that does accept it:
//
//	slog.SetDefault(slog.New(slog.NewTextHandler(os.Stderr,
//		&slog.HandlerOptions{Level: swd.LevelTrace})))
//
// Nothing in this module logs above [slog.LevelDebug]. A library that wrote to
// a caller's log at Info because it opened a device successfully would be
// deciding something that is the caller's to decide; what goes wrong comes
// back as an error instead.
const LevelTrace slog.Level = slog.LevelDebug - 4
