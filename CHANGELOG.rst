^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package control_toolbox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.18.2 (2020-06-04)
-------------------
* Python 3 compat part II
* Contributors: Tobias Fischer

1.18.1 (2020-06-01)
-------------------
* Python 3 compat
* Migrate to new industrial_ci
* Contributors: Matt Reynolds, Tobias Fischer

1.18.0 (2020-03-27)
-------------------
* Bump CMake version to avoid CMP0048 warning
* add static method to generate non-deterministic seed
* migrate to STL random library
* Contributors: James Xu, Shane Loretz, ahcorde

1.17.0 (2019-01-31)
-------------------
* update anti windup clamping
* update negativeIntegrationAntiwindupTest
* Address catkin_lint issues
* Add executable flag
* convert to package xml format 2
* Remove doc header
* Contributors: Bence Magyar, Cong, Gennaro Raiola

1.16.0 (2017-11-30)
-------------------
* switched to industrial_ci
* Add control_msgs to CATKIN_DEPENDS.
* Contributors: Bence Magyar, Mathias Luedtke, Mike Purvis

1.15.0 (2016-06-28)
-------------------
* avoid ABI breaks in PID class
* fix add_dependencies call
* rollback API changes in PID class
* cfg: removed rosbuild support related error handling
* Contributors: Bence Magyar, Igor Napolskikh, ipa-mig

1.14.0 (2016-05-03)
-------------------
* Fix negative gains issue and add tests; update gains setting through DynamicReconfig
* Add antiwindup and tests to PID controller; rename old behaviour 'clamping'
* Move message to control_toolbox
* Add optional state publishing to PID controller, for logging/debugging/etc
* Fix some typos in comments
* changed the range of dynamic reconfigure to allow negative ones
* Address -Wunused-parameter warnings
* Factor out updatePid as negative calls to computeCommand
* Increasing covergae of PID class test suite.
* Chain calls of computeCommand and updatePid for code reuse
* Contributors: Adolfo Rodriguez Tsouroukdissian, Bence Magyar, Carlos Rosales, Guillaume Walck, Paul Bovbel, VahidAminZ

1.13.2 (2015-05-22)
-------------------
* CRITICAL BUGFIX: Fix broken PID command computation.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Paul Bovbel

1.13.1 (2015-04-30)
-------------------
* Improvement in integral contribution implementation. Resolve `#32 <https://github.com/ros-controls/control_toolbox/issues/32>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian, Carlos Rosales

1.13.0 (2014-10-31)
-------------------
* Harmonize pid gain names between rosparam and dynamic_reconfigure
* Read i_clamp_min and i_clamp_max form parameter server - if available
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, ipa-fxm

1.12.1 (2014-06-12)
-------------------
* Remove broken test code. Hotfix for `#18 <https://github.com/ros-controls/control_toolbox/issues/18>`_.
* Contributors: Adolfo Rodriguez Tsouroukdissian

1.12.0 (2014-06-12)
-------------------
* pid: Adding quiet flag to suppress error message
* Contributors: Jonathan Bohren

1.11.0 (2014-05-12)
-------------------
* Remove rosbuild artifacts
* Cleaned up CMake and removed unnecessary dependencies
* Made default value negative to match valid range
* Fix for i_clamp_min to be negative in dynamic reconfigure
* Fix abs/fabs problem with Clang and libc++
* Contributors: Adolfo Rodriguez Tsouroukdissian, Dave Coleman, Marco Esposito

1.10.4 (2014-02-05)
-------------------
* Added Travis support
* Renamed manifest.xml so it doesn't break rosdep
* Expanded range of PID and windup gains for certain applications.
* Expanded range of PID and windup gains for certain applications. Lowered default integral and derivative gain
* check for CATKIN_ENABLE_TESTING
* Add some comments to Parameters.cfg
* Add support for dynamic_reconfigure for rosbuild
* Contributors: Austin Hendrix, Dave Coleman, Lukas Bulwahn, Paul Dinh

1.10.3 (2013-08-02)
-------------------
* Fix bug in copy constructor.
* Contributors: Austin Hendrix

1.10.2 (2013-07-29)
-------------------
* Fix copy constructor.
* Merge pull request `#1 <https://github.com/ros-controls/control_toolbox/issues/1>`_ from davetcoleman/hydro-devel
  Added dynamic reconfigure for PID gains
* Removed const getGains function
* Small fixes
* Compatibility changes for realtime_tools, tweaked getests
* Made realtime_buffer copiable
* Added test for getting/settings gains, copying/assigning pid class
* Removed const read, added copy constructor and print values function
* Added new function getGainsConst that allows one to get the PID gains from a const PID class
* Added realtime_tools as a dependency in package.xml and CMakeLists
* Added realtime buffer to PID, re-ordered functions to more logical order and to match header file
* Fixes per Austin review
* Updated CMakeLists.txt and made fixes per Adolfo
* Merged hydro-devel
* Added dynamic reconfigure for PID gains
* Tests build.
* Contributors: Austin Hendrix, Dave Coleman

1.10.1 (2013-06-26)
-------------------
* Add dependency on tinyxml.
* Contributors: Austin Hendrix

1.10.0 (2013-06-25)
-------------------
* Version 1.10.0
* comment format consistentcy
* Fixing comment in pid source code
* Install tune_pid.py under catkin.
* adding install targets
* adding missing manifests
* merging CMakeLists.txt files from rosbuild and catkin
* adding hybrid-buildsystem makefiles
* Merging from master, re-adding manifest.xml files
* using more standard way of depending on gencpp
* Add .gitignore file.
* Fixing library export
* catkinizing, could still be cleaned up
* Fixing doc errors in PID
* Changing @ commands to \ commands
* Enforcing i_min_ <= 0 and i_max_ >= 0 in integral bound parameters, reducing duplicated code
* Merge pull request `#14 <https://github.com/ros-controls/control_toolbox/issues/14>`_ from bobholmberg/fix-PID-unbounded-i_error
  Using zero i_gain_ to turn off integral control did unsavory things.
* Adding alternative name for new pid command computation API
* Fixing merge error
* Merge branch 'fix-pid-backwards-compatibility' into fix-PID-unbounded-i_error
* Removing lie from documentation
* Adding Bob's fixes to the backwards-compatibility API
* Merge branch 'fix-pid-backwards-compatibility' into fix-PID-unbounded-i_error
* bringing back old updatePid function contents
* adding documentation warning
* This makes the internal computations of updatePid() keep the same sign that they did before the API change
* Merge typo
* Resolving conflict from new Pid API
* Merge branch 'master' into test-bad-integral-bounds
* Merge branch 'test-bad-integral-bounds' into fix-PID-unbounded-i_error
* Specifying div-by-zero test, adding other integral term tests
* Merge branch 'test-bad-integral-bounds' into fix-PID-unbounded-i_error
* Adding test to expose Pid class zero-division vulnerability
* If the user did not want integral control and set i_gain_ to zero,
  then dividing by i_gain_ would set i_error_ to NaN.  This is not
  desired.  Instead, replace the use of division to create i_term
  with direct integration of i_term_.
  Replace private member i_error_ with i_term_.
  In getCurrentPIDErrors() create & return i_error_ with the same old meaning and units.
  NOTE: i_error_ is not needed internally anywhere else.
* Cleaning up documentation, making argument names in function declaration match those in the implementation
* adding doxygen deprecation flags
* Fixing documentation
* Merging changes from other branch
* Adding conventional PID computation
* Fixing inconsistent formatting, and reducing some duplicated code
* remove .svn folder
* move control_toolbox into ros_control
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Bob Holmberg, Jonathan Bohren, Wim Meeussen, wmeeusse
