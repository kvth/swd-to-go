package swd

import (
	"errors"
	"fmt"
)

// TransferError is what a [Probe] returns when a transfer list did not run to
// completion.
//
// Most callers only care that it failed and can treat it as an ordinary error.
// It carries the count because cmsis/server has to put it in the DAP_Transfer
// response, where a debugger uses it to work out which of the transfers it
// asked for actually happened.
type TransferError struct {
	// Status is the result of the transfer that stopped the list.
	Status Status
	// Completed is how many of the requests ran; Total is how many there were.
	Completed int
	Total     int
}

func (e *TransferError) Error() string {
	return fmt.Sprintf("swd: transfer %d/%d failed: %s", e.Completed, e.Total, e.Status)
}

// ErrNotImplemented is what a [Probe] method returns when the probe cannot do
// the thing at all -- as opposed to trying and failing.
//
// [Probe] is the set of calls every probe answers, but not every probe can
// answer all of them meaningfully: a probe at the far end of a link that does
// not carry pin access has no pins to report. Returning this is how it says so
// without a separate interface to type-assert for.
var ErrNotImplemented = errors.New("swd: not implemented")
