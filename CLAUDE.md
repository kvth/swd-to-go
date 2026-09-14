# swd-to-go

A Go SWD stack for ARM debug ports. See [README.md](README.md) for the
architecture (the "three altitudes" diagram) and package layout — that is the
authoritative description of how the code is shaped and should stay in sync
with it.

## Working on this repo

- `make` builds the native binaries into the repo root (not `bin/` — keep it
  that way; the root is the point).
- `make pi` / `make pi32` cross-build into `dist/<platform>/`, gitignored.
- The Makefile is intentionally flat: one copied rule per binary/platform
  combination rather than a loop or pattern rule over a list. When adding a
  new `cmd/` or `examples/` program, add its own three-line rule (and one line
  each to `pi`/`pi32`/`clean`) rather than reintroducing a loop.
- `make check` (fmt-check, vet, test) before calling anything done.
- `make docs` runs `godoc` locally for a preview. It does not publish
  anything — see below.

## Publishing

The module path is `github.com/kvth/swd-to-go`, matching the remote. Godoc
links in comments use that full path (`[github.com/kvth/swd-to-go/target/dp]`)
because pkg.go.dev resolves them by import path.

- Pushing a tag (e.g. `v0.1.0`) is enough for **pkg.go.dev to index and
  publish the docs automatically** — no manual "build docs" or "deploy docs"
  step. The repo has to be public. `make docs` is only for reading them
  locally.
- The licence is Apache 2.0 (`LICENSE`), and `NOTICE` is part of complying
  with it: it records the files derived from the Mongoose OS `mos` tool and
  the third-party licences. Keep it in step when a derived file changes or a
  dependency is added. `target/dp/dp.go`, `target/mem/memap.go`, `probe.go`
  and `cmsis/client/client.go` carry in-source notices; do not drop them.

## Scope

**Build what was asked, at the size it was asked. Nothing next to it.**

A request to check a value is a comparison, not a validator. A request to
report a failure is an error, not a sentinel and a taxonomy. A request for a
number is a constant, not a type with accessors. If something adjacent looks
worth doing, say so in a sentence and leave it undone -- the cost of a
suggestion is one line, and the cost of an unasked-for abstraction is that
someone has to read it, keep it true, and eventually ask for it to be removed.

This is not a style preference. Every layer added on spec has to be unwound
later by the person who did not ask for it, and unwinding is slower than
writing. Things this repo has already had removed for exactly this reason:

- a `DPIDRValue` type with six accessors and a `Check` method, where the check
  is `idr != dpidr`
- `CheckMultidrop`, `CheckTargetID` and a TARGETID decoder, where TARGETSEL
  already asserts the identity by addressing it
- an `ErrNotPresent` sentinel, where wrapping the error keeps more information
  and costs nothing

Concretely:

- Prefer a plain `uint32`, `string` or `bool` over a named type, unless the
  type has behaviour that more than one caller needs.
- Prefer wrapping an error with `%w` over translating it into a new one.
  Translating discards the chain; wrapping never does.
- Prefer an unexported constant over an exported one. Export it when something
  outside the package needs it, not in case it might.
- A helper called from one place is that place's code. Inline it unless the
  name is doing real explaining.
- When a fix and a nearby improvement both suggest themselves, do the fix.

## Performance

Performance matters everywhere, but it is paramount in the low-level
bit-banging routines (the actual SWD clocking/transport code). At that
altitude:

- Avoid interfaces and dynamic dispatch — they add indirection and prevent
  inlining on the hot path. Use concrete types and direct calls instead.
- Reserve interfaces/dynamic dispatch for larger, coarser-grained compound
  operations (e.g. `swd.Probe` and friends), where the call overhead is
  negligible relative to the work being done.

## Ideas for how this project should grow

Loose notes, not commitments — revisit and prune as the project moves:

- Keep the layering strict: nothing above `swd.Probe` should ever import a
  concrete driver or transport package directly. If a target/ or cmsis/
  package needs to reach into `drivers/` or `cmsis/transport/`, that is a
  design smell worth stopping for.
- There is deliberately only one optional probe interface, `swd.Cmsis`: the
  capabilities only a real CMSIS-DAP probe has (`DAP_Info` strings, status
  LEDs, the vendor command range and its packet size). It is type-asserted,
  never stubbed. Anything a probe can plausibly answer for itself belongs on
  `swd.Probe`, returning `swd.ErrNotImplemented` where it cannot — a handful of
  small optional interfaces each asserted separately is worse than one method
  that says "no". `ResetTarget` is on `swd.Probe` for exactly this reason: a
  bitbang probe has no device-specific sequence, so it reports
  `swd.ErrNotImplemented` rather than forcing every caller through a `Cmsis`
  assertion first. `swd.Calibrator` is a *driver* capability, not a probe one,
  and sits on `swd.BitBanger` implementations.
- **Return structs, accept interfaces.** `dp.New` and `mem.New` return
  `*dp.Client` and `*mem.Client` -- a constructor hands back the concrete
  thing it made. But no layer above takes one: each declares its own interface
  naming only the methods it calls, and takes that. `mem.DebugPort` is the
  five DP calls a MEM-AP makes; `rtt.Memory` is the four memory calls RTT
  makes; `rp2040.DebugPort` and `rp2040.Memory` are the same for flashing.
  The interface belongs to the consumer, not to the implementation.

  This is what keeps the higher layers off the implementation, and it is not
  theoretical: `target/mem` does not import `target/dp`, and `target/rtt`
  imports neither. Only `rp2040.Attach` -- the wiring point, which constructs
  both -- imports them at all. Do not replace one of these parameters with a
  concrete type to save an indirect call: at this altitude every call is a
  round trip on the wire, and the dispatch is free by comparison. (The
  bit-banging layer is where that trade-off runs the other way -- see
  Performance above.)

  Do not put a wide interface back in `dp` or `mem` describing the whole of
  what the `Client` there does. That is what the type itself is for; an
  interface that mirrors it method for method has one implementation and one
  purpose, which is to be mirrored again the next time someone changes the
  type. If a new consumer needs three methods, it declares those three.
- A one-method interface that exists only to receive a callback is a func
  field, not an interface. `server.Options.HandleVendor` is a
  `func(ctx, cmd uint8, request, response []byte) (consumed, produced int, ok
  bool)`, so a caller passes a closure or a method value and nothing has to
  declare that it implements anything. Keep interfaces for the cases with more
  than one method, or more than one implementation a caller picks between.
- Logging is `log/slog` and only `log/slog`; there is no logging dependency in
  `go.mod` and there should not be one again. Two levels, both quiet by
  default: `swd.LevelTrace` for the per-register and per-packet trace, and
  `slog.LevelDebug` for one-off facts. Nothing logs at Info or above -- a
  library writing to a caller's log on success is deciding something that is
  the caller's to decide. Guard every trace call with `Enabled`, the way the
  existing ones do, so a handler that does not want it costs the transfer path
  a comparison. `target/mem`'s trace_test.go is what notices if that stops
  working, since a broken guard shows up as silence and passes everything else.
- `rp2040.Attach` is the bring-up and nothing else, and it stays that way. No
  clock (the caller sets it on the probe), no debug-port power-up, no MEM-AP.
  It returns a detach func, never nil, so a caller can defer it before
  checking the error. The reason is the polling case: a board that multiplexes
  one SWD wire between a dozen targets asks "is anything there" far more often
  than it asks for anything else, and the answer is one transfer -- the DPIDR
  read Attach already has to do, because a multidrop target is not selected
  until its DPIDR is read. Do not fold `dp.Init` or `mem.Init` back in to make
  the call site shorter; that is several round trips per poll per position.
  Neither it nor `Rescue` sets the clock.
- That DPIDR is checked, not just read: an undriven wire floats to a rail and
  answers 0x00000000 or 0xffffffff, which is a successful transfer carrying
  nothing. `rp2040.checkIDR` compares the raw word against `rp2040.DPIDR`,
  0x0bc12477, which all three of the chip's ports report. One integer
  comparison -- keep it that way. `dp.Client.GetIDR` returns a plain uint32 for
  the same reason: whether a DPIDR is the right one is a question only the
  caller can answer, so target/dp does not wrap the value in a type that
  pretends otherwise.
- Nothing reads TARGETID back, and nothing should. DPIDR names the debug port
  (designer ARM, ARM's part number for the SW-DP design); TARGETID names the
  chip -- and the `TARGET_*` constants *are* TARGETID values (Raspberry Pi
  0x493, part 0x1002, instance in the top nibble). TARGETSEL is written with
  the TARGETID that must answer and the rest of the wire stays quiet, so the
  identity has already been asserted by addressing before DPIDR is read at
  all. Reading it back costs a SELECT write, the read, and a SELECT write to
  restore, and tells you what you already knew.
- A failed DPIDR read is wrapped and not translated, so the cause stays in the
  chain: `errors.As` for a `*swd.TransferError` is how a caller polling a muxed
  bus tells a position with nothing in it (the wire answered, badly) from a
  probe that has died (it never got that far). There is no sentinel for this
  and there should not be one -- wrap, do not replace.
- A `dp.Client`/`mem.Client` shadow is one target's state. Across a muxed wire
  that means one pair per target, not one pair shared -- see the "One client
  per target" sections in both package comments, which are the long answer.
  `Invalidate` on each is for the caller who shares one anyway, and for
  something else writing those registers on the same target.
- `internal/simtarget` is what makes the test suite hardware-free. Any new
  target layer (a new chip family, say) is worth landing with a simulated
  counterpart before real hardware, the same way `rp2040` did.
- CI: `make check` on push/PR is the natural first workflow; a release job
  that builds `pi`/`pi32` artifacts on tag push is the natural second one.
  Neither exists yet.
