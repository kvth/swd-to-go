package main

import (
	"strings"
	"testing"
)

// --driver chooses the backend and the pin flags say which lines; the two have
// to agree, and disagreeing is an error rather than something to reinterpret.
// That is small enough to get wrong quietly -- a pin that silently lands on the
// wrong chip drives the wrong thing -- so it is pinned down here rather than
// left to a board. How a pin is spelled is linuxgpiod.ParsePin's business and
// is tested there.

// --driver says which backend runs, and the pin format has to match it. Neither
// is inferred from the other: a pin written the wrong way is refused rather
// than quietly taken to mean a different backend.
func TestDriverNamesAreChecked(t *testing.T) {
	for _, c := range []struct {
		driver  string
		wantErr bool
	}{
		{driver: "bcm2835"},
		{driver: "linuxgpiod"},
		{driver: "gpiod", wantErr: true},
		{driver: "bcm2835gpio", wantErr: true},
		{driver: "", wantErr: true},
	} {
		err := checkDriver(&options{driver: c.driver})
		if c.wantErr && err == nil {
			t.Errorf("--driver %q was accepted", c.driver)
		}
		if !c.wantErr && err != nil {
			t.Errorf("--driver %q: %v", c.driver, err)
		}
	}
}

// The register backend has no chips to choose between, so a chip:line pin is an
// error and not something to drop half of.
func TestRegisterBackendRefusesAChipInAPin(t *testing.T) {
	opt := &options{driver: driverBCM2835,
		swclk: "16", swdioIn: "12", swdioOut: "20", swdioDir: "gpiochip2:5"}
	_, err := opt.pinNumbers()
	if err == nil {
		t.Fatal("a chip:line pin was accepted by the register backend")
	}
	for _, want := range []string{"--swdio-dir", "linuxgpiod"} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("the error should mention %q: %v", want, err)
		}
	}
}

// And the other way: the character-device backend needs a chip on every pin,
// with no default for a bare number to land on.
func TestChardevBackendRefusesABarePin(t *testing.T) {
	opt := &options{driver: driverLinuxGPIOD,
		swclk: "16", swdioIn: "gpiochip0:12", swdioOut: "gpiochip0:20", swdioDir: "gpiochip0:21"}
	_, err := opt.pins()
	if err == nil {
		t.Fatal("a bare line number was accepted by the character-device backend")
	}
	for _, want := range []string{"--swclk", "chip:line"} {
		if !strings.Contains(err.Error(), want) {
			t.Errorf("the error should mention %q: %v", want, err)
		}
	}
}

func TestEveryPinKeepsItsOwnChip(t *testing.T) {
	opt := &options{
		driver: driverLinuxGPIOD,
		swclk:  "gpiochip0:16", swdioIn: "gpiochip0:12",
		swdioOut: "pinctrl-rp1:20", swdioDir: "gpiochip2:5",
	}
	pins, err := opt.pins()
	if err != nil {
		t.Fatal(err)
	}
	for _, c := range []struct {
		name string
		got  string
		want string
	}{
		{"swclk", pins.swclk.Chip, "gpiochip0"},
		{"swdio-in", pins.swdioIn.Chip, "gpiochip0"},
		{"swdio-out", pins.swdioOut.Chip, "pinctrl-rp1"},
		{"swdio-dir", pins.swdioDir.Chip, "gpiochip2"},
	} {
		if c.got != c.want {
			t.Errorf("%s landed on chip %q, want %q", c.name, c.got, c.want)
		}
	}
	if pins.swclk.Line != 16 || pins.swdioDir.Line != 5 {
		t.Errorf("lines came out as %+v", pins)
	}
}

// A flag the wiring does not read must not fail the run or pick a backend. The
// one-pin wirings never look at --swdio-in, which still holds its default.
func TestUnusedPinFlagsAreIgnored(t *testing.T) {
	opt := &options{
		driver: driverBCM2835,
		bidir:  true,
		swclk:  "16", swdio: "20",
		// Left over from the split wiring's defaults, and nonsense besides.
		swdioIn: "not-a-pin", swdioOut: "gpiochip9:3", swdioDir: "21",
	}
	n, err := opt.pinNumbers()
	if err != nil {
		t.Fatalf("parsing the pins this wiring uses: %v", err)
	}
	if n.swclk != 16 || n.swdio != 20 {
		t.Errorf("got swclk %d swdio %d, want 16 and 20", n.swclk, n.swdio)
	}
}

// The one-pin wiring reads --swdio-dir only when it was given one.
func TestBidirReadsTheDirectionPinOnlyWhenGiven(t *testing.T) {
	bare := &options{driver: driverBCM2835, bidir: true, swclk: "16", swdio: "20", swdioDir: "nonsense"}
	if _, err := bare.pinNumbers(); err != nil {
		t.Errorf("an unread --swdio-dir failed the run: %v", err)
	}

	buffered := &options{driver: driverBCM2835, bidir: true, gaveSWDIODir: true,
		swclk: "16", swdio: "20", swdioDir: "21"}
	n, err := buffered.pinNumbers()
	if err != nil {
		t.Fatal(err)
	}
	if n.swdioDir != 21 {
		t.Errorf("swdio-dir = %d, want 21", n.swdioDir)
	}
}

func TestBadPinFlagsNameTheFlag(t *testing.T) {
	for _, c := range []struct {
		name string
		opt  options
	}{
		{"not a number", options{driver: driverBCM2835,
			swclk: "sixteen", swdioIn: "12", swdioOut: "20", swdioDir: "21"}},
		{"empty chip", options{driver: driverBCM2835,
			swclk: ":16", swdioIn: "12", swdioOut: "20", swdioDir: "21"}},
		{"no line", options{driver: driverBCM2835,
			swclk: "gpiochip0:", swdioIn: "12", swdioOut: "20", swdioDir: "21"}},
	} {
		_, err := c.opt.pinNumbers()
		if err == nil {
			t.Errorf("%s: accepted, want an error", c.name)
			continue
		}
		if !strings.Contains(err.Error(), "--swclk") {
			t.Errorf("%s: the error does not name the flag: %v", c.name, err)
		}
	}
}
