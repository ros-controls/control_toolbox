:github_url: https://github.com/ros-controls/control_toolbox/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Jazzy to Kilted
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Jazzy (previous) and Kilted (current) releases, where changes to user code might be necessary.

Pid/PidROS
***********************************************************
* The parameters ``antiwindup``, ``i_clamp_max``, and ``i_clamp_min`` have been removed. The anti-windup behavior is now configured via the ``AntiWindupStrategy`` enum. (`#298 <https://github.com/ros-controls/control_toolbox/pull/298>`_).
* The constructors of ``PidROS`` have changed and ``prefix_is_for_params`` argument has been deprecated. Use explicit parameter and topic prefix instead (`#431 <https://github.com/ros-controls/control_toolbox/pull/431>`_).
