# Control filters

Implement filter plugins for control purposes as https://index.ros.org/r/filters/github-ros-filters/

## Available filters

* Low Pass: implements a low-pass filter based on a time-invariant [Infinite Impulse Response (IIR) filter](https://en.wikipedia.org/wiki/Infinite_impulse_response), for different data types (doubles or wrench).
* Exponential Filter: Exponential filter for double data type.

## Low Pass filter

This filter implements a low-pass filter in the form of an [IIR filter](https://en.wikipedia.org/wiki/Infinite_impulse_response), applied to a `data_in` (double or wrench).
The feedforward and feedback coefficients of the IIR filter are computed from the low-pass filter parameters.

### Required parameters

* sampling frequency as `sf`
* damping frequency as `df`
* damping intensity as `di`

### Algorithm

Given
* above-required parameters,  `sf`, `df`, `di`
* `data_in`, a double or wrench `x`

Compute `data_out`, the filtered output `y(n)` with equation:

y(n) = b x(n-1) + a y(n-1)

with

* a the feedbackward coefficient such that a = exp( -1/sf (2 pi df) / (10^(di/-10)) )
* b the feedforward coefficient such that b = 1 - a


## Exponential filter

### Required parameters
* `alpha`: the exponential decay factor

### Algorithm

  smoothed_value  = alpha * current_value + (1 - alpha) * last_smoothed_value;
