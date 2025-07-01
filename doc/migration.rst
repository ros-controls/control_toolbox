:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
<<<<<<< HEAD
This list summarizes important changes between Humble (previous) and Jazzy (current) releases, where changes to user code might be necessary.

.. note::

  This list was created in June 2025 (tag 4.4.0), earlier changes may not be included.
=======
This list summarizes important changes between Jazzy (previous) and Kilted (current) releases, where changes to user code might be necessary.

Pid/PidRos
***********************************************************
* The parameters :paramref:`antiwindup`, :paramref:`i_clamp_max`, and :paramref:`i_clamp_min` have been removed. The anti-windup behavior is now configured via the :paramref:`AntiWindupStrategy` enum. (`#298 <https://github.com/ros-controls/control_toolbox/pull/298>`_).
>>>>>>> ff1f2d6 (Update documentation of PID class (#388))
