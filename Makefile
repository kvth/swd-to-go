# swd-to-go
#
#   make            build everything for this machine, into the repo root
#   make pi         cross-build for a 64-bit Pi, into dist/linux-arm64/
#   make pi32       cross-build for a 32-bit Pi, into dist/linux-armv7/
#   make test       run the tests
#   make check      vet, gofmt and test
#   make docs       serve package docs locally at http://localhost:6060
#
# Pure Go, so cross-building needs no toolchain -- except for the USB HID
# transport, which is cgo. `make pi`/`make pi32` leave it out (see NOUSB
# below); everything else, including the gateway, builds anywhere.

GO      ?= go
GOFLAGS ?=

# Trim paths and drop the symbol table: these run on a Pi, and nothing here
# wants a 40% larger binary for stack traces it will never read.
LDFLAGS ?= -s -w
BUILD    = $(GO) build $(GOFLAGS) -trimpath -ldflags "$(LDFLAGS)"

# The USB HID transport needs libudev through cgo, which a cross-build has no
# toolchain for. The tag leaves the package's contents out; nothing else in the
# module depends on it.
NOUSB = -tags no_libudev

.PHONY: all
all: swd-gateway rtt-local rtt-tcp probe-info flash-rp2040

.PHONY: swd-gateway
swd-gateway:
	$(BUILD) -o swd-gateway ./cmd/swd-gateway

.PHONY: rtt-local
rtt-local:
	$(BUILD) -o rtt-local ./examples/rtt-local

.PHONY: rtt-tcp
rtt-tcp:
	$(BUILD) -o rtt-tcp ./examples/rtt-tcp

.PHONY: probe-info
probe-info:
	$(BUILD) -o probe-info ./examples/probe-info

.PHONY: flash-rp2040
flash-rp2040:
	$(BUILD) -o flash-rp2040 ./examples/flash-rp2040

# --- cross-building ---------------------------------------------------------

# A 64-bit Pi (Pi 3 and later, CM3/CM4 on a 64-bit OS).
.PHONY: pi
pi:
	@mkdir -p dist/linux-arm64
	GOOS=linux GOARCH=arm64 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-arm64/swd-gateway ./cmd/swd-gateway
	GOOS=linux GOARCH=arm64 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-arm64/rtt-local ./examples/rtt-local
	GOOS=linux GOARCH=arm64 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-arm64/rtt-tcp ./examples/rtt-tcp
	GOOS=linux GOARCH=arm64 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-arm64/probe-info ./examples/probe-info
	GOOS=linux GOARCH=arm64 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-arm64/flash-rp2040 ./examples/flash-rp2040

# A 32-bit Pi. Pi 2 and later are armv7; a Pi 1 or Zero needs GOARM=6.
.PHONY: pi32
pi32:
	@mkdir -p dist/linux-armv7
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-armv7/swd-gateway ./cmd/swd-gateway
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-armv7/rtt-local ./examples/rtt-local
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-armv7/rtt-tcp ./examples/rtt-tcp
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-armv7/probe-info ./examples/probe-info
	GOOS=linux GOARCH=arm GOARM=7 CGO_ENABLED=0 $(BUILD) $(NOUSB) -o dist/linux-armv7/flash-rp2040 ./examples/flash-rp2040

# --- checks -----------------------------------------------------------------

.PHONY: test
test:
	$(GO) test ./...

.PHONY: race
race:
	$(GO) test -race ./...

.PHONY: vet
vet:
	$(GO) vet ./...

.PHONY: fmt
fmt:
	$(GO) fmt ./...

# gofmt -l prints what is misformatted and exits zero either way, so turn a
# non-empty list into a failure rather than a line of output nobody reads.
.PHONY: fmt-check
fmt-check:
	@files=$$(gofmt -l . ); \
	if [ -n "$$files" ]; then \
		echo "not gofmt'ed:"; echo "$$files"; exit 1; \
	fi

.PHONY: check
check: fmt-check vet test

.PHONY: cover
cover:
	$(GO) test -coverprofile=cover.out ./...
	$(GO) tool cover -func=cover.out | tail -1
	@echo "html: go tool cover -html=cover.out"

.PHONY: tidy
tidy:
	$(GO) mod tidy

# --- docs --------------------------------------------------------------

# Once this module lives at a real GitHub import path (see CLAUDE.md), pushing
# a tag is enough for pkg.go.dev to pick it up on its own -- nothing to build
# or publish by hand. This target is only for reading the docs locally first.
.PHONY: docs
docs:
	go run golang.org/x/tools/cmd/godoc@latest -http=:6060

.PHONY: clean
clean:
	rm -f swd-gateway rtt-local rtt-tcp probe-info flash-rp2040 cover.out
	rm -rf dist
