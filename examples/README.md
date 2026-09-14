# Examples

Each of these is a small, complete program showing one workflow. They all build
with a plain `go build ./examples/...`; the ones that touch real hardware say so.

| example | what it shows |
|---|---|
| [rtt-local](rtt-local) | tail an RTT channel over locally bit-banged SWD, in process, with no CMSIS-DAP framing anywhere |
| [rtt-tcp](rtt-tcp) | the same thing against a CMSIS-DAP probe over TCP |
| [probe-info](probe-info) | ask a probe what it is, and whether there is a CMSIS-DAP on the other end |
| [flash-rp2040](flash-rp2040) | write an image to an RP2040's flash through its bootrom |

The point of the first two is that they are the same program. The only
difference is one constructor:

```go
probe = bitbang.New(driver)              // rtt-local
probe, err = tcp.Dial(ctx, addr)         // rtt-tcp
```

Everything after that — bring-up, DP, MEM-AP, RTT — is identical, because it is
written against [`swd.Probe`](../probe.go) and cannot tell the two apart.
`rtt-local` in particular encodes nothing: transfers go straight from the RTT
layer to the wire, with no packets in between.

## Running

`rtt-local` and `flash-rp2040` drive GPIO directly, so they need root or
membership of the `gpio` group:

```bash
sudo ./rtt-local --swclk 16 --swdio-in 12 --swdio-out 20 --swdio-dir 21
sudo ./rtt-local --swclk 16 --swdio-in 20 --swdio-out 20 --swdio-dir -1   # one bidirectional pin
sudo ./rtt-local --swclk 16 --swdio-in 20 --swdio-out 20 --swdio-dir 21   # ...behind a buffer
```

Setting `-swdio-in` equal to `-swdio-out` selects a single bidirectional pin;
`-swdio-dir -1` says no direction line is wired.

`-driver bcm2835`, the default, writes the BCM GPIO registers and is a Pi 1-4.
`-driver linuxgpiod` goes through the Linux GPIO character device instead —
slower, and the only thing that works on a Pi 5 or on a board that is not a Pi.
Its pins are written `chip:line`, every one of them, and a pin written the other
backend's way is refused rather than reinterpreted:

```bash
sudo ./swd-gateway --list-gpiochips                              # which chip?
sudo ./rtt-local -driver linuxgpiod --swclk gpiochip0:16 --swdio-in gpiochip0:12 \
  --swdio-out gpiochip0:20 --swdio-dir gpiochip0:21
```

`rtt-tcp` and `probe-info` only need something at the other end of the socket —
`swd-gateway` on a Pi, OpenOCD, or any other CMSIS-DAP TCP probe:

```bash
./rtt-tcp --probe 192.168.1.50:4441
```
