# Control filters

Implement filter plugins for control purposes as https://index.ros.org/r/filters/github-ros-filters/

## Available filters

* Gravity Compensation: implements a gravity compensation algorithm, removing the gravity component from the incoming data (Wrench).
* Low Pass: implements a low-pass filter based on a time-invariant [Infinite Impulse Response (IIR) filter](https://en.wikipedia.org/wiki/Infinite_impulse_response), for different data types (doubles or wrench).
* Exponential Filter: Exponential filter for double data type.


## Gravity compensation filter

This filter implements an algorithm compensating for the gravity forces acting at the center of gravity (CoG) of a known mass, computed at a `sensor_frame` and applied to a `data_in` wrench.

 The filter relies on tf2, and might fail if transforms are missing.

 Note that, for convenience, the filter can perform additional frame changes if data_out frame id is given.

### Required parameters

* `world_frame` (&Rscr;<sub>w</sub>): frame in which the `CoG.force` is represented.
* `sensor_frame` (&Rscr;<sub>s</sub>): frame in which the `CoG.pos` is defined
* `CoG.pos` (p<sub>s</sub>): position of the CoG of the mass the filter should compensate for
* `CoG.force` (g<sub>w</sub>): constant (but updatable) force of gravity at the Cog (typically m.G), defined along axes of the `world_frame`

### Algorithm

Given

* above-required parameters,  &Rscr;<sub>w</sub>, &Rscr;<sub>s</sub>, p<sub>s</sub>, g<sub>w</sub>
* `data_in`, a wrench &Fscr;<sub>i</sub> = {f<sub>i</sub>, &tau;<sub>i</sub>} represented in the `data_in` frame &Rscr;<sub>i</sub>
* access to tf2 homogeneous transforms:
  * T<sub>si</sub> from &Rscr;<sub>i</sub> to &Rscr;<sub>s</sub>
  * T<sub>sw</sub> from &Rscr;<sub>w</sub> to &Rscr;<sub>s</sub>
  * T<sub>os</sub> from &Rscr;<sub>s</sub> to &Rscr;<sub>o</sub>

Compute `data_out` compensated wrench &Fscr;c<sub>o</sub> = {fc<sub>o</sub>, &tau;c<sub>o</sub>} represented in the `data_out` frame &Rscr;<sub>o</sub> if given, or the `data_in` frame &Rscr;<sub>i</sub> otherwise, with equations:

&Fscr;c<sub>o</sub> = T<sub>os</sub>.&Fscr;c<sub>s</sub>,


with &Fscr;c<sub>s</sub> = {fc<sub>s</sub>, &tau;c<sub>s</sub>}  the compensated wrench in `sensor_frame` (common frame for computation)

and,

fc<sub>s</sub> = f<sub>s</sub> - T<sub>sw</sub>g<sub>w</sub>

its force  and,

&tau;c<sub>s</sub> = &tau;<sub>s</sub> - p<sub>s</sub> x (T<sub>sw</sub>g<sub>w</sub>)

its torque, and

&Fscr;<sub>s</sub>  = T<sub>si</sub>.&Fscr;<sub>i</sub> = {f<sub>s</sub>, &tau;<sub>s</sub>}

the full transform of the input wrench &Fscr;<sub>i</sub> to sensor frame &Rscr;<sub>s</sub>

Remarks :
* a full vector is used for gravity force, to not impose gravity to be only along z of `world_frame`.
* `data_in` frame is usually equal to `sensor_frame`, but could be different since measurement of wrench might occur in another frame. E.g.: measurements are at the **FT sensor flange** = `data_in` frame, but CoG is given in **FT sensor base** = `sensor_frame` (=frame to which it is mounted on the robot), introducing an offset (thickness of the sensor) to be accounted for.
* `data_out` frame is usually `data_in` frame, but for convenience, can be set to any other useful frame. E.g.: wrench expressed in a `control_frame` like the center of a gripper.
* T<sub>sw</sub> will only rotate the g<sub>w</sub> vector, because gravity is a field applied everywhere, and not a wrench (no torque should be induced by transforming from &Rscr;<sub>w</sub> to &Rscr;<sub>s</sub>).


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
