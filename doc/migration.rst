:github_url: https://github.com/ros-controls/realtime_tools/blob/{REPOS_FILE_BRANCH}/doc/migration.rst

Migration Guides: Humble to Jazzy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This list summarizes important changes between Humble (previous) and Jazzy (current) releases, where changes to user code might be necessary.

.. note::

  This list was created in June 2025 (tag 4.4.0), earlier changes may not be included.

Pid/PidRos
***********************************************************
* The parameters :paramref:`antiwindup`, :paramref:`i_clamp_max`, and :paramref:`i_clamp_min` will be removed. The anti-windup behavior is now configured via the :paramref:`AntiWindupStrategy` enum. (`#298 <https://github.com/ros-controls/control_toolbox/pull/298>`_).
