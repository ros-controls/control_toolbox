:github_url: https://github.com/ros-controls/control_toolbox/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Jazzy to Lyrical
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Jazzy (previous) and Lyrical (current) releases.

Pid/PidROS
***********************************************************
* Added a saturation feature to PID output and two anti-windup techniques (back calculation and conditional integration) (`#298 <https://github.com/ros-controls/control_toolbox/pull/298>`_).
* Added a constructor argument to ``PidROS`` to control if the PID state publisher is initially active or not. Can be changed during runtime by using  ``activate_state_publisher`` parameter. (`#431 <https://github.com/ros-controls/control_toolbox/pull/431>`_).

Filters
***********************************************************
* Added a templated ``control_toolbox::ExponentialFilter`` class, which is now used by ``control_filters::ExponentialFilter``. ``filters::exponentialSmoothing()`` is now a template, too. (`#493 <https://github.com/ros-controls/control_toolbox/pull/493>`_).
* Added a ``LowPassFilter::configure(initial_state)`` overload to start the filter from a given state instead of from the first input. (`#657 <https://github.com/ros-controls/control_toolbox/pull/657>`_).
