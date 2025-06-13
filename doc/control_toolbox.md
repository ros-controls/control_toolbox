# Base classes


## PID


## PID Controller

The PID (Proportional-Integral-Derivative) controller is a widely used feedback control loop mechanism that calculates an "error" value as the difference between a measured process variable and a desired setpoint. The controller attempts to minimize the error by adjusting the process control inputs. This class implements a generic structure that can be used to create a wide range of PID controllers. It can function independently or be subclassed to provide more specific controls based on a particular control loop.

### PID Equation

The standard PID equation is given by:

command = p<sub>term</sub> + i<sub>term</sub> + d<sub>term</sub>

where:
* p<sub>term</sub> = p<sub>gain</sub> * error
* i<sub>term</sub> = i<sub>term</sub> + i<sub>gain</sub> * error * dt
* d<sub>term</sub> = d<sub>gain</sub> * d<sub>error</sub>

and:
* error = desired_state - actual_state
* d<sub>error</sub> = (error - error<sub>last</sub>) / dt

### Parameters

*   `p` (Proportional gain): This gain determines the reaction to the current error. A larger proportional gain results in a larger change in the controller output for a given change in the error.
*   `i` (Integral gain): This gain determines the reaction based on the sum of recent errors. The integral term accounts for past values of the error and integrates them over time to produce the `i_term`. This helps in eliminating steady-state errors.
*   `d` (Derivative gain): This gain determines the reaction based on the rate at which the error has been changing. The derivative term predicts future errors based on the rate of change of the current error. This helps in reducing overshoot, settling time, and other transient performance variables.
*   `u_clamp` (Minimum and maximum bounds for the controller output): These bounds are applied to the final command output of the controller, ensuring the output stays within acceptable physical limits.
*   `trk_tc` (Tracking time constant): This parameter is specific to the 'back_calculation' anti-windup strategy. If set to 0.0 when this strategy is selected, a recommended default value will be applied.
*   `antiwindup_strat` (Anti-windup strategy): This parameter selects how the integrator is prevented from winding up when the controller output saturates. Available options are:
    *   `NONE`: no anti-windup technique; the integral term accumulates without correction.
    *   `BACK_CALCULATION`: adjusts the integral term based on the difference between the unsaturated and saturated outputs using the tracking time constant `trk_tc`. Faster correction for smaller `trk_tc`.
    *   `CONDITIONAL_INTEGRATION`: only updates the integral term when the controller is not in saturation or when the error drives the output away from saturation, freezing integration otherwise.

### Anti-Windup Strategies

Anti-windup functionality is crucial for PID controllers, especially when the control output is subject to saturation (clamping). Without anti-windup, the integral term can accumulate excessively when the controller output is saturated, leading to large overshoots and sluggish response once the error changes direction. The `control_toolbox::Pid` class offers two anti-windup strategies:

*   **`BACK_CALCULATION`**: This strategy adjusts the integral term based on the difference between the saturated and unsaturated controller output. When the controller output `command` exceeds the output limits (`u_max` or `u_min`), the integral term `i_term` is adjusted by subtracting a value proportional to the difference between the saturated output `command_sat` and the unsaturated output `command`. This prevents the integral term from accumulating beyond what is necessary to maintain the output at its saturation limit. The `tracking_time_constant` (`trk_tc`) parameter is used to tune the speed of this adjustment. A smaller `trk_tc` results in faster anti-windup action.

    The update rule for the integral term with back-calculation is:

    i<sub>term</sub> += dt * (i<sub>gain</sub> * error + (1 / trk<sub>tc</sub>) * (command<sub>sat</sub> - command))

    If `trk_tc` is set to 0.0, a default value is calculated based on the proportional and derivative gains:
    *   If `d_gain` is not zero: trk<sub>tc</sub> = &radic;(d<sub>gain</sub> / i<sub>gain</sub>)
    *   If `d_gain` is zero: trk<sub>tc</sub> = p<sub>gain</sub> / i<sub>gain</sub>

*   **`CONDITIONAL_INTEGRATION`**: In this strategy, the integral term is only updated when the controller is not in saturation or when the error has a sign that would lead the controller out of saturation. Specifically, the integral term is frozen (not updated) if the controller output is saturated and the error has the same sign as the saturated output. This prevents further accumulation of the integral term in the direction of saturation.

    The integral term is updated only if the following condition is met:

    (command - command<sub>sat</sub> = 0) &or; (error * command &le; 0)

    This means the integral term `i_term` is updated as `i_term += dt * i_gain * error` only when the controller is not saturated, or when it is saturated but the error is driving the output away from the saturation limit.

### Usage Example

To use the `Pid` class, you should first call some version of `initialize()` and then call `compute_command()` at every update step. For example:

```cpp
control_toolbox::Pid pid;
pid.initialize(6.0, 1.0, 2.0, 5, -5,2,control_toolbox::AntiwindupStrategy::BACK_CALCULATION);
double position_desired = 0.5;
...
rclcpp::Time last_time = get_clock()->now();
while (true) {
  rclcpp::Time time = get_clock()->now();
  double effort = pid.compute_command(position_desired - currentPosition(), time - last_time);
  last_time = time;
}
```
