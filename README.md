# swd-to-go

A Go SWD stack for ARM debug ports: bitbanged SWD over GPIO, a CMSIS-DAP client
and server, and DP, MEM-AP, RTT and RP2040 layers on top.

Everything above the wire is written against one interface, `swd.Probe`. A
locally bitbanged port and a CMSIS-DAP probe on the far end of a cable are both
one of those, so the layers above cannot tell them apart and nothing has to be
wrapped to make them fit.

The CMSIS-DAP wire protocol here is the one OpenOCD's `cmsis-dap backend tcp`
speaks, so OpenOCD can point straight at `swd-gateway`.

## The three altitudes

```
target/  dp · mem · rp2040 · rtt          written against swd.Probe
   ↑
swd.Probe ──────────────┬───────────────────────┐
   ↑                    ↑                       ↑
probe/bitbang     cmsis/client            (anything else)
   ↑                    ↑
swd.BitBanger     cmsis/transport  usb · tcp · inproc
   ↑
   ├── drivers/bcm2835      GPIO registers, mmap'ed  (Pi 1-4)
   └── drivers/linuxgpiod   the kernel's GPIO chardev (anything, Pi 5 included)
```

`swd.BitBanger` is pins: bits, turnarounds, one ACK per transfer. `swd.Probe` is
transactions: WAIT retries, posted AP reads collected from RDBUFF, match-value
polling, block transfers. `probe/bitbang` is the engine that turns the first
into the second, and it is the only place that logic exists.

`cmsis/server` is the mirror of `cmsis/client`: the client turns Probe calls
into packets, the server turns packets back into Probe calls. Neither has any
SWD logic of its own, which is why a server in front of a bitbang probe
re-exports a locally wired target for OpenOCD in three lines.

## Using it

```go
// A locally bitbanged port, over the BCM GPIO registers...
d, err := bcm2835.OpenSplit(16, 12, 20, 21) // swclk, swdio in/out, dir

// ...or the same pins through the kernel's GPIO character device, which is
// what a Pi 5 needs and what works on a board that is not a Pi at all.
d, err := linuxgpiod.OpenSplit("gpiochip0", 16, 12, 20, 21)

d.Calibrate()
probe := bitbang.New(d)

// ...or a CMSIS-DAP probe over TCP, a Unix socket, or USB HID.
probe, err := tcp.Dial(ctx, "127.0.0.1:4441")
probe, err := usb.Open(ctx, vid, pid, serial)
```

From there on the code is identical whichever one you picked:

```go
defer probe.Close(ctx)
probe.SetClock(ctx, 1_500_000)

// Bring-up: dormant, SWD, line reset, multidrop TARGETSEL, DPIDR read.
detach, err := rp2040.Attach(ctx, probe, rp2040.TARGET_CORE_0)
defer detach(context.WithoutCancel(ctx))

// The debug port and the MEM-AP are separate steps. Skip them if all you
// wanted was the line above.
dpc := dp.New(probe)
dpc.Init(ctx)
apc := mem.New(dpc, 0)
apc.Init(ctx)
```

`rp2040.Attach` is the sequence spelled out — `swd.JTAGToDormant`,
`swd.DormantToSWD`, `swd.LineReset`, `swd.SelectTarget` and a DPIDR read — for a
target this package does not have a bring-up for.

### Polling a bus

`Attach` is deliberately the bring-up and nothing else, because a lot of work
does not need anything above it:

```go
for _, pos := range positions {
	mux.Select(ctx, pos)                                  // your own hardware

	detach, err := rp2040.Attach(ctx, probe, rp2040.TARGET_CORE_0)
	present[pos] = err == nil                             // that is the check
	if err != nil || !hasWork(pos) {
		detach(ctx)
		continue
	}

	dpc[pos].Init(ctx)                                    // only now
	apc[pos].Init(ctx)
	// ...RTT, flashing, whatever this position had queued
	detach(ctx)
}
```

**The presence check is `Attach` itself, and there is nothing cheaper.** The
DPIDR read at the end of it is not an extra: a multidrop target does not
consider itself selected until its DPIDR has been read, and a DP coming out of
a line reset ignores everything else until that read happens. So the cheapest
*legal* thing to do after the switching sequences is also the thing that proves
a target is on the wire — one transfer, and you were going to pay for it
anyway. Powering the debug domains up and probing the MEM-AP costs several
round trips more, which is why they are yours to call and not folded in.

**The value is checked, not just the transfer.** An undriven SWD line floats to
one rail or the other, so a read from an empty position comes back all-zeros or
all-ones — a transfer that "succeeded". So `Attach` compares the word: it has to
equal `0x0bc12477`, which all three ports report.

Note what that value actually says, because it is not what it looks like:
designer **ARM**, part `0xbc` — ARM's number for the SW-DP design, which every
licensee of that design reports alike. DPIDR describes the debug port, not the
chip. The register that names the chip is TARGETID, and the `TARGET_*`
constants here *are* TARGETID values:

```
0x01002927
  |  |  ||
  |  |  |+- bit 0, read as one
  |  |  +-- TDESIGNER 0x493 — JEP106 continuation 9, identity 0x13: Raspberry Pi
  |  +----- TPARTNO 0x1002: RP2040
  +-------- TINSTANCE: core 0, core 1 (0x1), or the rescue port (0xf)
```

TARGETSEL is written with the TARGETID that should answer and every other target
on the wire stays quiet, so **a target that answered has already asserted which
chip and which core it is** — nothing reads TARGETID back, because the identity
check is the addressing. The DPIDR comparison sits on top of that.

The top nibble of that DPIDR is a revision. A stepping that moved it would be
rejected; the error prints the value read, so widening the constant is the fix.

`core` is checked too, against the three constants above, before the wire is
touched — a TARGETSEL nobody answers looks exactly like an empty position, so a
typo would otherwise come back as "nothing is plugged in" and be believed.

**Telling an empty position from a broken probe.** A failed read is wrapped as
it is, so the cause is still in the chain. A transfer that came back without a
usable ACK is the wire answering — an ordinary result for a position with
nothing in it — while a dead socket or a cancelled context is not:

```go
_, err := rp2040.Attach(ctx, probe, rp2040.TARGET_CORE_0)

var te *swd.TransferError
switch {
case err == nil:              // present
case errors.As(err, &te):     // nothing in this position
default:                      // the probe or the link is broken
}
```

`Rescue` applies the same check to the rescue port, and neither it nor `Attach`
sets the clock — do that once on the probe before either.

**One `dp.Client` and one `mem.Client` per position, not one shared.** Both keep
a shadow of what they last wrote — `SELECT` in the debug port, `CSW` and `TAR`
in the MEM-AP — and that shadow is *that target's* state. Share one across a
mux and it describes a chip that is no longer listening: the next access skips a
`TAR` write it needed and lands at the wrong address, quietly, with a plausible
value. Sharing can be made safe with `Invalidate` or a re-`Init` after every
switch, but that throws away exactly the saved writes the shadow exists for, and
re-runs the MEM-AP's enable check every time. Per-position clients keep their
shadows, so a target revisited a moment later is still described by the one it
left behind. They are an allocation each and no I/O, so holding one per
position costs nothing.


## Packages

### The interface

| package | what it is |
|---|---|
| `swd` | `Probe`, `BitBanger`, the transfer types, and the switching sequences (line reset, JTAG↔dormant↔SWD, multidrop `TARGETSEL`) |

`Probe` covers everything a probe can answer for itself, the SWJ pins and
`DAP_ResetTarget` included: a probe whose link does not carry the pins, or that
has no device-specific reset sequence of its own, returns
`swd.ErrNotImplemented` rather than forcing every caller through a type
assertion.

There is one optional interface on top of it, `swd.Cmsis`, type-asserted rather
than stubbed. It is the whole of what only a real CMSIS-DAP probe can answer —
the `DAP_Info` strings, the status LEDs, the `ID_DAP_Vendor0..31` escape hatch
and the packet size that bounds it. A bitbang driver has none of that and does
not pretend to: it is one question ("is there a DAP on the other end?"), so it
is one assertion rather than one per capability. `swd.Calibrator` is the other
optional interface,
and it is a *driver* one — for a `BitBanger` that paces clock edges with a
counted loop.

### Drivers — implement `swd.BitBanger`

| package | what it is |
|---|---|
| `drivers/bcm2835` | GPIO registers `mmap`'ed from `/dev/gpiomem`, three SWDIO wirings, one type each. **Pi 1-4 and CM1/CM3/CM4 only** — see below |
| `drivers/linuxgpiod` | the Linux GPIO character device — the v2 uAPI `libgpiod` wraps, spoken directly. Any board, **including the Pi 5** |

Both are a `swd.BitBanger` and a `swd.Calibrator`, and nothing above them can
tell which one it got.

Three constructors, one per wiring, each taking exactly the pins that wiring
has — no config struct with a field that does not apply, no sentinel for "not
wired", no enum where a `bool` says it:

```go
bcm2835.OpenBidir(swclk, swdio int)                        // one bidirectional pin
bcm2835.OpenBidirBuffered(swclk, swdio, swdioDir int)      // ...behind a buffer
bcm2835.OpenSplit(swclk, swdioIn, swdioOut, swdioDir int)  // separate in/out pair
```

A driver arrives **uncalibrated**: the delay loop's cost is unknown, so
`SetFrequencyHz` cannot pace anything yet. Call `Calibrate()` — or
`SetSpeedCoeffs()` with numbers measured earlier — before setting a clock.
It is not done at `Open` because measuring clocks SWCLK for a fraction of a
second, and only the caller knows when that is safe.

```go
d, err := bcm2835.OpenSplit(16, 12, 20, 21)
coeff, offset, err := d.Calibrate()   // before a target is attached
```

`drivers/linuxgpiod` has the same three, each taking a chip as well, and a
`...Pins` variant of each for a wiring whose pins are not all on one chip:

```go
linuxgpiod.OpenBidir("gpiochip0", swclk, swdio)
linuxgpiod.OpenBidirBuffered("gpiochip0", swclk, swdio, swdioDir)
linuxgpiod.OpenSplit("pinctrl-rp1", swclk, swdioIn, swdioOut, swdioDir)

linuxgpiod.OpenSplitPins(                     // the direction line elsewhere
    linuxgpiod.Pin{"pinctrl-rp1", 16},
    linuxgpiod.Pin{"pinctrl-rp1", 12},
    linuxgpiod.Pin{"pinctrl-rp1", 20},
    linuxgpiod.Pin{"gpiochip2", 5},
)
```

Those are OpenOCD's `adapter gpio swdio` with, respectively, neither
`swdio_dir` nor `swdio_in`; `swdio_dir`; and both. Clock edges are paced by a
counted delay loop calibrated at startup — nothing here sleeps or polls a wall
clock per edge, because a `time.Now()` call costs more than the edge does.

**The wirings are separate types on purpose.** Folding them into one behind a
flag puts a direction test on every turnaround; hiding them behind an interface
puts a dynamic call on every pin operation. Since the maximum reachable clock is
`speed_coeff / speed_offset`, anything that inflates the per-edge cost lowers
the top speed the hardware can be driven at — calibration measures that cost, it
does not remove it. As it stands, `DriverSplit.SwdioOutEnable` compiles to a
single masked store inlined into `SwdTransfer`.

What the three genuinely share sits on an unexported `core` they embed by value.
That is a plain struct embed, not an interface, so it costs nothing: a promoted
method compiles to the same instruction stream as a free function taking
`*core`. What it does not cover is `SwdTransfer`, which calls the direction
switches and so exists once per wiring. A fix to one belongs in all three, and
`drivers/bcm2835` cannot check that: its fake register block remembers only the
last write to `GPSET0` and `GPCLR0`, so clock edges are not countable and a copy
clocking the wrong number of recovery cycles would pass unnoticed.
`drivers/linuxgpiod` records every ioctl and does make that comparison, in
`TestEveryWiringClocksTheSameTransfer`.

The per-edge cost is worth knowing before optimising anything here. Each pin is
resolved to its registers and masks once at open rather than per edge, but
`go test ./drivers/bcm2835 -bench .` shows that is not where the time goes: the
whole address computation is under a nanosecond, and the memory barrier after
every register write is around ten times everything else in an edge put
together. It cannot go — without it the CPU's write buffer may merge two stores
to `GPSET0` and swallow a clock edge — so the ceiling on this driver's SWD clock
is how fast the machine retires a release store.

#### Which boards

`drivers/bcm2835` is the BCM2835 register layout, which BCM2836, BCM2837 and
BCM2711 all kept:

| supported | not supported |
|---|---|
| Pi 1, 2, 3, 4 · Pi Zero, Zero 2 W · CM1, CM3, CM3+, CM4 | **Pi 5, Pi 500, CM5** |

The Pi 5 family puts its GPIO on an **RP1 southbridge across PCIe** — different
register layout, and no `/dev/gpiomem` at all (it exposes `/dev/gpiomem0..4`).
Opening this driver there fails with a message saying so rather than writing to
the wrong addresses. `drivers/linuxgpiod` below is what runs there.

There is still no RP1 *register* driver here, and writing one is a different
proposition from the generic driver. `/dev/gpiomem0` is mmap-able, but it is a
different register file (`IO_BANK0`, `SYS_RIO0`, `PADS_BANK0`), every access is a
PCIe transaction rather than a local one. It should not be written without an
RP1 to measure on: the calibration would absorb the higher per-access cost
silently, so a wrong
register offset would look like a slow clock rather than an error.

### `drivers/linuxgpiod` — the generic one

The Linux GPIO character device: ask the kernel for lines by number on a named
`/dev/gpiochipN` and drive them. No register layout, no board-specific address,
nothing to be wrong about a particular chip — it runs on a Pi 5, on a Pi 1-4,
and on hardware that is not a Raspberry Pi.

```bash
./swd-gateway --list-gpiochips         # which chip are the header pins on?
```

The trade is one syscall per edge instead of one store:

| | per edge | reachable SWCLK | boards |
|---|---|---|---|
| `drivers/bcm2835` | ~30–60 ns | megahertz | Pi 1-4, CM1/CM3/CM4 |
| `drivers/linuxgpiod` | ~1–3 µs | hundreds of kHz | anything with a gpiochip |

Those two rows are an order-of-magnitude expectation, not a measurement taken
here — the second one has never been run on a Pi. What the driver reports after
`Calibrate` is the measurement, and `speed_coeff / speed_offset` is where its
ceiling actually is on the board in front of you. Either way it is the right
driver on a Pi 5 and the wrong one on a Pi 4.

**It talks the kernel uAPI directly rather than linking `libgpiod`.** libgpiod
v2 is a wrapper over exactly these ioctls, and going straight at them keeps the
whole module pure Go — which is what makes `make pi` a cross-build with no
toolchain — sidesteps the libgpiod v1-versus-v2 split that otherwise costs a
slab of compatibility shims, and leaves the ioctl payload ours to shape. That
last one is where the speed is:

> **Every pin a wiring uses is claimed in one `GPIO_V2_GET_LINE` request**, so
> they share a single `SET_VALUES` bitmap and one ioctl drives any combination
> of them. A written SWD bit is therefore *SWDIO := bit and SWCLK := low*, then
> *SWCLK := high* — two syscalls, where a driver claiming each signal as its
> own libgpiod line pays two or three. A read bit is still three, because
> sampling SWDIO cannot fold into either clock edge.

A turnaround is one ioctl too. On the split wiring it is a value write to the
direction line; on the bidirectional wirings it is a `SET_CONFIG` naming only
SWDIO — the kernel leaves any line whose flags carry no direction alone, so
flipping SWDIO provably cannot disturb SWCLK sharing the same request.

#### Pins on more than one chip

Since every pin names its own chip, they do not all have to be on the same one.
The `...Pins` constructors take a `Pin` per signal, and the gateway says it
inline as `chip:line`:

```bash
sudo ./swd-gateway --driver linuxgpiod \
  --swclk gpiochip0:16 --swdio-in gpiochip0:12 \
  --swdio-out gpiochip0:20 --swdio-dir gpiochip2:5
```

That is for the board whose data pins are on the SoC and whose buffer direction
line is on an I2C or SPI expander. What it costs depends on which signal moved,
and the driver says which it got rather than being quietly slower:

| moved to another chip | cost |
|---|---|
| the direction line | **nothing** — a turnaround was always an ioctl of its own |
| SWDIO | **a third of the clock** — it can no longer share SWCLK's ioctl, so a written bit is three syscalls |

`Driver.Fused()` reports which one a driver ended up as, and the gateway logs a
warning when it is the slow one. One chip named two ways — `0` and
`/dev/gpiochip0`, or a label — is one chip: opened once and claimed together,
since two requests on one chip would be refused for any line the first already
holds. `linuxgpiod.ParsePin` is the one parser for `chip:line`, so the gateway
and the examples cannot disagree about what a pin spelling means.

**There is one type here and three constructors, where `bcm2835` has three
types.** The reason `bcm2835` splits them is that a branch or an indirect call
costs a real fraction of a 30 ns edge. Against a syscall it does not, and
`go test ./drivers/linuxgpiod -bench .` is in the repo so that this is a
measurement rather than an assertion:

```
BenchmarkIoctlFloor        263 ns/op     one syscall, rejected at the fd lookup
BenchmarkWriteBit          8.7 ns/op     everything a written bit costs but the syscalls
BenchmarkDispatchDirect    0.37 ns/op    a concrete call
BenchmarkDispatchInterface 2.85 ns/op    through an interface
BenchmarkDispatchGeneric   2.65 ns/op    through a generic type parameter
```

Two of those syscalls is what a bit costs in the kernel; this driver's own work
is under 2% of it. The third and fourth lines are also the answer to whether
`bcm2835` could have been written generically instead: **Go generics are not a
zero-cost abstraction here.** A method call on a type parameter goes through the
instantiation's dictionary, which is an indirect call — the same 2.7 ns an
interface costs, not the 0.37 ns of the concrete call. Genericity is affordable
where an edge is a syscall and not where an edge is a store, which is why the
two drivers are shaped differently rather than shaped the same.

An ioctl can fail where a store cannot, and `swd.BitBanger` has nowhere to
return an error — at these rates a returned error per bit would cost more than
the bit. The first failure is kept and reported by `Driver.Err()`; the transfer
carries on and comes back with whatever the wire looked like. `Err()` is what
separates "the target is not answering" from "the GPIO went away".

### Probes — implement `swd.Probe`

| package | what it is |
|---|---|
| `probe/bitbang` | the transfer engine: any `swd.BitBanger` becomes a `swd.Probe` |
| `cmsis/client` | the CMSIS-DAP packet codec, over any `Transport` |
| `cmsis/transport/tcp` | packets over TCP or a Unix socket |
| `cmsis/transport/usb` | packets over USB HID (build tag `no_libudev` leaves it out) |
| `cmsis/transport/inproc` | packets straight into a `cmsis/server` in the same process |

### CMSIS-DAP server — consumes a `swd.Probe`

| package | what it is |
|---|---|
| `cmsis/wire` | the protocol constants, shared by both halves so they cannot drift |
| `cmsis/server` | command processing, including `DAP_ExecuteCommands` / `DAP_QueueCommands` batching |
| `cmsis/server/tcp` | the server, over TCP or a Unix domain socket |

`cmsis/server` is a CMSIS-DAP implementation and nothing more. The
`ID_DAP_Vendor0..31` range answers `ID_DAP_Invalid`: what a vendor command means
is a particular probe's business, not this library's. Where the probe behind it
cannot do what a command asks, the reply is whatever the protocol already has
for that case rather than a stub that lies:

- `DAP_SWJ_Pins` against a probe whose pins answer `swd.ErrNotImplemented` is
  `DAP_ERROR` — reporting every line low would be a guess.
- `DAP_ResetTarget` against a probe with no device-specific sequence is OK with
  the "no device-specific reset" bit clear, which is exactly what a real probe
  without one says.
- `DAP_HostStatus` against a probe that is not a `swd.Cmsis` is OK: there is
  nothing to light up, and that is what a probe without LEDs answers.
- JTAG is not implemented at all.

### Target layers — consume a `swd.Probe`

| package | what it is |
|---|---|
| `target/dp` | debug port: power-up, `SELECT`, AP register access |
| `target/mem` | MEM-AP: target memory, word and byte granularity — an unaligned range is served by the words containing it, one transfer rather than one TAR write and one DRW access per byte. Shadows `CSW` and `TAR` so a register write that would change nothing never goes out — polling one address costs one round trip rather than two, and a block run reloads `TAR` only where the AP's auto-increment wraps |
| `target/rtt` | SEGGER RTT against a target's control block. The block is read once at attach and its fixed parts kept, so a poll is one descriptor read; the scan that finds it overlaps its chunks, steps over unmapped memory, and checks a candidate's header before believing the ID string |
| `target/rp2040` | RP2040 bring-up (`Attach` does the dormant/SWD dance, the multidrop `TARGETSEL` and the DPIDR read that completes it, and returns a detach func — it sets no clock and initialises nothing above the wire), `Rescue` for a board whose firmware has locked the debug port out, and bootrom flashing — `Flasher` batches a ROM call's register setup into one transfer list and moves bulk data with block transfers, and `Flasher.FlashImage` is the whole write with the preverify, verify and reset steps OpenOCD's `program` has |

**Each of these takes an interface it declares itself, and returns a struct.**
`dp.New` hands back a `*dp.Client` and `mem.New` a `*mem.Client` — a
constructor returns the concrete thing it made. Nothing above takes one:
`mem.DebugPort` names the five DP calls a MEM-AP makes, `rtt.Memory` the four
memory calls RTT makes, `rp2040.DebugPort` and `rp2040.Memory` the same for
flashing. The interface belongs to the package that calls through it, narrowed
to what that package calls.

The point is that the coupling is to the calls and not to the type, and the
import graph says so: `target/mem` does not import `target/dp`, and
`target/rtt` imports neither of them. Only `rp2040.Attach`, which constructs
both, imports either. Substituting something else for a MEM-AP — a simulator,
a cache, a second AP — is a matter of satisfying four methods, not of
reimplementing `mem.Client`.

Dispatch through those interfaces is free here in a way it is not at the pins:
every call below is a round trip on the wire, so the 2.5 ns an interface costs
is lost in a transfer that takes microseconds. That is the same measurement
that argues the other way one layer down.

### internal/simtarget

A simulated debug port, MEM-AP and RP2040 over ordinary memory: bootrom with its
function table, SRAM, an XIP-mapped flash array the bootrom routines actually
program, and a SEGGER RTT control block whose channel answers what it is sent.
It implements `swd.BitBanger`, so everything above the pins is production code.

## Commands

`swd-gateway` bit-bangs SWD over GPIO and serves it as a CMSIS-DAP probe over
TCP or a Unix socket, so OpenOCD can point straight at it. It is a Go
replacement for the C gateway at
[`cmsis-dap-tcp-gateway-rpi`](https://github.com/kvth/cmsis-dap-tcp-gateway-rpi), with the same
flags for the parts it keeps.

```bash
sudo ./swd-gateway --port 4441 --swclk 16 --swdio-in 12 --swdio-out 20 --swdio-dir 21
```

```bash
sudo ./swd-gateway --port 4441 --swclk 16 --swdio 20
```

The first is the split wiring: separate SWDIO input and output pins behind a
buffer turned round by a direction line. The second is one bidirectional pin
wired straight to the target, where the GPIO itself is flipped and no direction
line is driven at all. Adding `--swdio-dir` to the second gives the third: one
pin behind a buffer. Each picks a different constructor.

**`--driver linuxgpiod` swaps the backend underneath all three**: the pins are
driven through the Linux GPIO character device instead of the BCM registers,
which is what a Pi 5 needs and what makes the gateway run on hardware that is
not a Pi. Everything above the driver — the wiring flags, the calibration, the
vendor command, the CMSIS-DAP server — is unchanged, because the two drivers are
the same two interfaces.

```bash
sudo ./swd-gateway --list-gpiochips
sudo ./swd-gateway --driver linuxgpiod --port 4441 \
  --swclk gpiochip0:16 --swdio-in gpiochip0:12 \
  --swdio-out gpiochip0:20 --swdio-dir gpiochip0:21
```

**The backend is chosen, not inferred, and the pin format has to match it.**
`--driver bcm2835` (the default) takes bare line numbers; `--driver linuxgpiod`
takes `chip:line` for every pin. A pin written the other way is refused rather
than reinterpreted:

```
$ swd-gateway --driver linuxgpiod --swclk 16 ...
--swclk is "16", but --driver linuxgpiod needs every pin to name its chip:
write it as chip:line (--list-gpiochips shows what there is)
```

The chip half is a number, a name, a path or a label, so `0:16`, `gpiochip0:16`,
`/dev/gpiochip0:16` and `pinctrl-rp1:16` are the same pin. It is worth listing
rather than guessing: which chip carries the header pins has moved between
kernel versions, and a wrong guess drives something else. There is no default
chip — every pin says where it is, which is the reason one cannot quietly end up
on the wrong controller. The gateway logs where each pin actually resolved to,
rather than what was asked for.

Clock edges are paced by a counted delay loop whose cost this binary measures
for itself at startup, in well under a second. The two numbers it measures are
OpenOCD's `speed_coeff` and `speed_offset` and can be passed in with
`--speed-coeff` / `--speed-offset` to skip the measurement — but they describe
this Go binary's compiled loop, so values measured for the C gateway do not
carry over. Nor do values measured for the other backend: an edge that is a
syscall has an offset an order of magnitude larger, which is the calibration
saying where the ceiling is.

Calibration is also the one vendor command the gateway implements, at the same
ID and wire format as the C one:

```
0x91 Calibrate
  request:   (no arguments)
  response:  u8 status, u32 speed_coeff, u32 speed_offset
```

The rest of the `ID_DAP_Vendor0..31` range answers `ID_DAP_Invalid`. The
library itself implements none of it: `server.Options.HandleVendor` is where
a program that owns both ends of the link says what its probe means by one.

`swd-gateway --help` has the rest.

## examples/

Small complete programs, one per workflow — see [examples/](examples) for what
each one shows. The pair worth reading together is `rtt-local` and `rtt-tcp`:
the same program, tailing an RTT channel, differing only in where the probe
comes from. `rtt-local` goes from the RTT layer straight to the wire with no
CMSIS-DAP framing anywhere in the path; `rtt-tcp` reaches a probe over a
socket. Everything between the probe and the output is identical code.

`flash-rp2040` writes an image through the bootrom. Its flags are OpenOCD's
`program` vocabulary, because that is the one anyone flashing an RP2040 already
has:

```bash
./flash-rp2040 -image firmware.bin -preverify -verify -run
```

`-preverify` reads the range back first and skips the write entirely when the
image is already there; `-verify` reads it back afterwards; `-erase auto` (the
default) clears only the sectors that actually changed, and skips the erase of
one that is already blank. `-rescue` comes first when the board is the problem:
it resets the chip through the multidrop instance `0xF`, whose `CDBGPWRUPREQ` is
wired to the power-on state machine rather than to a core, and stops it in the
bootrom before anything runs out of flash. That is the way back onto a board
whose firmware hangs, reconfigures the QSPI pins or disables the debug port —
`-run`'s ordinary reset cannot help there, because the code being reset into is
what is going wrong.

## Building

```bash
make
```

Binaries land in the repo root. `make pi` cross-builds for a 64-bit Pi and
`make pi32` for a 32-bit one, into `dist/linux-arm64` and `dist/linux-armv7`;
both are pure Go and need no toolchain, because they leave out the USB HID
transport, which is the one cgo thing here (`-tags no_libudev`).

Root — or membership of the `gpio` group — is needed for `/dev/gpiomem` with the
register backend, and for `/dev/gpiochipN` with `chip:line` pins.

## Logging

The library logs through `log/slog` and nothing else — there is no logging
dependency in `go.mod`, and nothing here writes to a global of its own.

Two levels, and both are quiet by default:

| level | what |
|---|---|
| `swd.LevelTrace` (`slog.LevelDebug - 4`) | every DP and MEM-AP register access, and every CMSIS-DAP packet in and out |
| `slog.LevelDebug` | the handful of one-off facts — which HID device was opened |

Nothing logs at Info or above. A library that wrote to your log because it
opened a device successfully would be deciding something that is yours to
decide; what actually goes wrong comes back as an `error`.

The trace level is separate from Debug because it is orders of magnitude
noisier — one flash write is tens of thousands of records — so wanting to see a
driver's ordinary chatter should not oblige you to take the register trace with
it. Every trace call site is behind `slog.Logger.Enabled`, so a handler that
does not accept the level costs the transfer path a comparison and no
formatting.

```go
slog.SetDefault(slog.New(slog.NewTextHandler(os.Stderr,
	&slog.HandlerOptions{Level: swd.LevelTrace})))
```

```
level=DEBUG-4 msg="dp read" reg=DPCTRLSTAT value=0xf0000000
level=DEBUG-4 msg="ap read" reg=CSW value=0x00000052
level=DEBUG-4 msg="read reg" addr=0x20000000 value=0x00000000
```

`dp.Client`, `mem.Client` and `usb.Transport` each take a logger of their own
through `SetLogger` for callers who would rather not route this through the
default.

## Testing

```bash
make check
```

That is `gofmt`, `go vet` and `go test ./...`.

`probe` drives the same simulated target through both `swd.Probe`
implementations and requires the same answers, which is what pins the
interchangeability claim down. `cmsis/server/tcp` runs the client and the server
against each other over a real socket, TCP and Unix both, and drives target
memory, RTT and an RP2040 flashing session through the whole stack. No Pi, no
SWD wire and no board.

`target/rp2040` flashes the simulated chip: the preverify and auto-erase paths
are checked by asserting that a target already holding the image takes no
bootrom calls at all, and the rescue sequence by requiring it to fail on a wire
where only a core answers. `target/mem` and `target/rtt` count transfers — the
shadow's whole job, and the cached control-block header's, is to keep them off
the wire, so what they are worth is the thing to assert.

`drivers/bcm2835` stands a driver up over a heap buffer instead of a real
`/dev/gpiomem` mmap, through the same pin resolution production uses, so the
wiring and the calibration arithmetic need no Pi. Two things it cannot reach,
both because the fake block stores mask writes rather than recording them: a
transfer never reads back what it drove, so only the "no valid ACK" path runs,
and clock edges are not countable. `go test ./drivers/bcm2835 -bench .` is where
the cost of an edge is, and why there is nothing to win by tidying around it.

`drivers/linuxgpiod` stands a driver up over a fake gpiochip: not a stub, but a
model that tracks each line's direction and level, applies a `SET_CONFIG`
payload the way `linereq_set_config()` does — including leaving lines with no
direction flag alone — and refuses a `SET_VALUES` naming a line that is an
input, which is the kernel rule the driver's write mask exists to keep. Because
it records every ioctl, it can assert the things the `bcm2835` fake cannot:
that a written bit costs exactly two syscalls and carries both lines in the
first of them, and that each path through `SwdTransfer` clocks exactly the
number of SWCLK cycles the SWD specification says it does — 46 for a transfer
with a data phase, 13 for a WAIT without one. A recovery loop that ran 32 times
instead of 33 fails that test and no other.

The multi-chip wirings are tested against a world of two fake chips that share
an ordering counter, so "SWDIO was written after SWCLK fell" is a question the
tests can ask across two requests. A transfer over a wiring spread across two
chips is required to clock exactly the same 46 cycles as one on a single chip.

What neither driver's tests can reach is a real wire. The uAPI structs and
ioctl numbers are checked against the kernel's own arithmetic (`TestIoctlNumbers`
recomputes them from the `_IOC` encoding rather than trusting the constants),
but nothing here has driven a real `/dev/gpiochipN`, which needs a board.
