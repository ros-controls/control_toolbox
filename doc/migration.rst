:github_url: https://github.com/ros-controls/control_toolbox/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Humble (previous) and Jazzy (current) releases, where changes to user code might be necessary.

.. note::

  This list was created in June 2025 (tag 4.4.0), earlier changes may not be included.

Pid/PidROS
***********************************************************
* The parameters ``antiwindup``, ``i_clamp_max``, and ``i_clamp_min`` will be removed. The anti-windup behavior is now configured via the ``AntiWindupStrategy`` enum. (`#298 <https://github.com/ros-controls/control_toolbox/pull/298>`_).
* The constructors of ``PidROS`` have changed and ``prefix_is_for_params`` argument has been deprecated. Use explicit parameter and topic prefix instead (`#431 <https://github.com/ros-controls/control_toolbox/pull/431>`_).
