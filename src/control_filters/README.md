# Control Filters

## Available filters

* Gravity Compensation: implements a gravity compensation algorithm, removing the gravity component from the incoming data (Wrench).
*


## Gravity Compensation filter

 This filter implements a gravity compensation algorithm, applied to an `data_in wrench`, computed at a `sensor frame` in which the center of gravity (CoG) of the to-be-compensated mass is known.

 The filter relies on ROS TF, and might fail if transforms are missing.

 Note that, for convenience, the filter can perform additional frame changes if data_out frame id is given.

### required parameters

* `world_frame` (&Rscr;<sub>w</sub>): frame in which the `CoG.force` is represented.
* `sensor_frame` (&Rscr;<sub>s</sub>): frame in which the `CoG.pos` is defined
* `CoG.pos` (p<sub>s</sub>): position of the CoG of the mass the filter should compensate for
* `CoG.force` (g<sub>w</sub>): constant (but updatable) force of gravity at the Cog (typically m.G), defined along axes of the `world_frame`

### algorithm

Given
* above-required parameters,  &Rscr;<sub>w</sub>, &Rscr;<sub>s</sub>, p<sub>s</sub>, g<sub>w</sub>
* `data_in`, a wrench &Fscr;<sub>i</sub> = {f<sub>i</sub>, &tau;<sub>i</sub>} represented in `data_in.frame` &Rscr;<sub>i</sub>
* access to TF homogeneous transforms:
  * T<sub>si</sub> from &Rscr;<sub>i</sub> to &Rscr;<sub>s</sub>
  * T<sub>sw</sub> from &Rscr;<sub>w</sub> to &Rscr;<sub>s</sub>
  * T<sub>os</sub> from &Rscr;<sub>s</sub> to &Rscr;<sub>o</sub>

Compute `data_out` compensated wrench &Fscr;c<sub>o</sub> = {fc<sub>o</sub>, &tau;c<sub>o</sub>} represented in `data_out.frame` &Rscr;<sub>o</sub> if given, or `data_in.frame` &Rscr;<sub>i</sub> otherwise, with equations:

&Fscr;c<sub>o</sub> = T<sub>os</sub>.&Fscr;c<sub>s</sub>,


with &Fscr;c<sub>s</sub> = {fc<sub>s</sub>, &tau;c<sub>s</sub>}  the compensated wrench in `sensor_frame` (common frame for computation)

and,

fc<sub>s</sub> = f<sub>s</sub> - T<sub>sw</sub>.g<sub>w</sub>

its force  and,

&tau;c<sub>s</sub> = &tau;<sub>s</sub> - p<sub>s</sub> x (T<sub>sw</sub>.g<sub>w</sub>)

its torque, and

&Fscr;<sub>s</sub>  = T<sub>si</sub>.&Fscr;<sub>i</sub>

the full transform of the input wrench &Fscr;<sub>i</sub> to sensor frame &Rscr;<sub>s</sub>

Remarks :
* a full vector is used for gravity force, to not impose gravity to be only along z of `world_frame`.
* `data_in.frame` is usually equal to `sensor_frame`, but could be different since measurement of wrech might occur in another frame. Ex: measurements are at the **FT sensor flange** = `data_in.frame`, but CoG is given in **FT sensor base** = `sensor_frame` (=frame to which it is mounted on the robot), introducing an offset (thichkess of the sensor) to be accounted for.
* `data_out.frame` is usually `data_in.frame`, but for convenience, can be set to any other useful frame. Ex: Wrench expressed in a `control_frame` for instance center of a gripper.
* T<sub>sw</sub> will only rotate the g<sub>w</sub> vector, because gravity is a field applied everywhere, and not a wrench (no torque should be induced by transforming from &Rscr;<sub>w</sub> to &Rscr;<sub>s</sub>).
