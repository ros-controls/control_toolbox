:github_url: https://github.com/ros-controls/control_toolbox/blob/{REPOS_FILE_BRANCH}/doc/release_notes.rst

Release Notes: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes the changes between Humble (previous) and Jazzy (current) releases.

.. note::

  This list was created in June 2025 (tag 4.4.0), earlier changes may not be included.

Pid/PidROS
***********************************************************
* Added a saturation feature to PID output and two anti-windup techniques (back calculation and conditional integration) (`#298 <https://github.com/ros-controls/control_toolbox/pull/298>`_).
* Added a constructor argument to ``PidROS`` to control if the PID state publisher is initially active or not. Can be changed during runtime by using  ``activate_state_publisher`` parameter. (`#431 <https://github.com/ros-controls/control_toolbox/pull/431>`_).
