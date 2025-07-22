:github_url: https://github.com/ros-controls/control_toolbox/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Jazzy (previous) and Kilted (current) releases.

Pid/PidROS
***********************************************************
* Added a saturation feature to PID output and two anti-windup techniques (back calculation and conditional integration) (`#298 <https://github.com/ros-controls/control_toolbox/pull/298>`_).
* Added a constructor argument to ``PidROS`` to control if the PID state publisher is initially active or not. Can be changed during runtime by using  ``activate_state_publisher`` parameter. (`#431 <https://github.com/ros-controls/control_toolbox/pull/431>`_).
