package swd

// Calibrator is implemented by bitbang drivers that pace clock edges with a
// counted delay loop rather than by polling a wall clock.
//
// The parameterisation is the one OpenOCD's "bcm2835gpio speed_coeffs" uses,
// because the same two numbers have to serve both: one SWCLK half period is
//
//	iterations = ceil(coeff / kHz) - offset
//
// loop iterations, so coeff is the loop's iteration count per kHz and offset is
// the fixed cost of the GPIO register write that brackets it, in the same
// units. One iteration therefore costs 500000/coeff nanoseconds, and the
// highest clock the driver can reach at all is coeff/offset kHz.
//
// The numbers are specific to the compiled loop, not just to the CPU: values
// measured for a C gateway's loop do not transfer to this one, which is why
// [Calibrator.Calibrate] exists rather than a table of defaults.
type Calibrator interface {
	// Calibrate measures the delay loop and a bare clock edge on this machine,
	// applies the result at the current clock, and reports what it measured.
	//
	// It drives real SWCLK edges — SWDIO is left alone — so it is safe with a
	// target attached, but a session mid-transfer will see spurious clocking
	// while it runs.
	Calibrate() (coeff, offset uint32, err error)

	// SetSpeedCoeffs applies values measured earlier, skipping the
	// measurement. offset must be at least 1 and below coeff: zero would claim
	// an unbounded maximum clock.
	SetSpeedCoeffs(coeff, offset uint32) error

	// SpeedCoeffs reports the values currently in effect.
	SpeedCoeffs() (coeff, offset uint32)
}
