^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package control_toolbox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

6.0.0 (2025-10-16)
------------------

5.9.0 (2025-10-16)
------------------
* Add test for trc if i-gain is zero (`#526 <https://github.com/ros-controls/control_toolbox/issues/526>`_)
* Fix calculation of tracking time constant (`#511 <https://github.com/ros-controls/control_toolbox/issues/511>`_)
* Remove duplicate storage of limits (`#512 <https://github.com/ros-controls/control_toolbox/issues/512>`_)
* Improve PID parameter validation (`#510 <https://github.com/ros-controls/control_toolbox/issues/510>`_)
* [PidROS] Change args to const reference (`#513 <https://github.com/ros-controls/control_toolbox/issues/513>`_)
* Fix -Wunused-result (`#506 <https://github.com/ros-controls/control_toolbox/issues/506>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota, bijoua29

5.8.1 (2025-10-05)
------------------
* Fix ambiguous constructor overload (`#499 <https://github.com/ros-controls/control_toolbox/issues/499>`_)
* Contributors: Christoph Fröhlich

5.8.0 (2025-10-02)
------------------
* Cleanup `prefix_is_for_params` (`#494 <https://github.com/ros-controls/control_toolbox/issues/494>`_)
* Increase PID ROS wrapper test coverage  (`#484 <https://github.com/ros-controls/control_toolbox/issues/484>`_)
* Fix -Wuninitialized-const-reference (`#485 <https://github.com/ros-controls/control_toolbox/issues/485>`_)
* Contributors: Abdullah, Christoph Fröhlich

5.7.0 (2025-09-22)
------------------
* Add backward_ros dependency (`#475 <https://github.com/ros-controls/control_toolbox/issues/475>`_)
* Remove legacy and deprecated PID parameters (`#436 <https://github.com/ros-controls/control_toolbox/issues/436>`_)
* Fix rst errors (`#447 <https://github.com/ros-controls/control_toolbox/issues/447>`_)
* Use the FilterTest fixture instead (`#439 <https://github.com/ros-controls/control_toolbox/issues/439>`_)
* Fix deprecated TF headers (`#444 <https://github.com/ros-controls/control_toolbox/issues/444>`_)
* Declare missing parameters for PID (`#443 <https://github.com/ros-controls/control_toolbox/issues/443>`_)
* Contributors: Christoph Fröhlich, Victor Coutinho Vieira Santos

5.6.0 (2025-07-22)
------------------
* Deprecate `prefix_is_for_params` of PidROS (`#431 <https://github.com/ros-controls/control_toolbox/issues/431>`_)
* Fix integral action for `AntiWindupStrategy::NONE` (`#432 <https://github.com/ros-controls/control_toolbox/issues/432>`_)
* Update description of limit() function in rate_limiter (`#425 <https://github.com/ros-controls/control_toolbox/issues/425>`_)
* Update documentation of PID class (`#388 <https://github.com/ros-controls/control_toolbox/issues/388>`_)
* Contributors: Aarav Gupta, Christoph Fröhlich, Victor Coutinho Vieira Santos

5.5.0 (2025-06-18)
------------------
* Use RealtimeThreadSafeBox for PID class (`#387 <https://github.com/ros-controls/control_toolbox/issues/387>`_)
* Redefine the antiwindup strategy and their parameter interfacing (`#400 <https://github.com/ros-controls/control_toolbox/issues/400>`_)
* Skip callback if saturation parameter is not declared (`#397 <https://github.com/ros-controls/control_toolbox/issues/397>`_)
* Use new RT publisher API in PID class (`#394 <https://github.com/ros-controls/control_toolbox/issues/394>`_)
* [PID] Cleanup saturation parameter in the methods and constructors (`#390 <https://github.com/ros-controls/control_toolbox/issues/390>`_)
* Don't validate data_out if it is empty (`#391 <https://github.com/ros-controls/control_toolbox/issues/391>`_)
* Update anti-windup techniques (`#298 <https://github.com/ros-controls/control_toolbox/issues/298>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota, Victor Coutinho Vieira Santos

5.4.0 (2025-05-23)
------------------
* Update deprecated call to ament_target_dependencies (`#364 <https://github.com/ros-controls/control_toolbox/issues/364>`_)
* Make member variables of LowPassFilter class generic (`#351 <https://github.com/ros-controls/control_toolbox/issues/351>`_)
* Contributors: David V. Lu!!, Pedro de Azeredo

5.3.2 (2025-05-05)
------------------
* fix deprecated tf2 header (`#361 <https://github.com/ros-controls/control_toolbox/issues/361>`_)
* Contributors: Bence Magyar

5.3.1 (2025-04-25)
------------------
* Minor filter fixes + clang-format rules update (`#347 <https://github.com/ros-controls/control_toolbox/issues/347>`_)
* Cleanup removed msg fields (`#346 <https://github.com/ros-controls/control_toolbox/issues/346>`_)
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

5.3.0 (2025-04-24)
------------------
* Add std::vector<double> option to the low pass filter (`#340 <https://github.com/ros-controls/control_toolbox/issues/340>`_)
* Contributors: Marq Rasmussen

5.2.0 (2025-04-19)
------------------
* Fix clang-format (`#327 <https://github.com/ros-controls/control_toolbox/issues/327>`_)
* Add gravity compensation filter (`#153 <https://github.com/ros-controls/control_toolbox/issues/153>`_)
* Contributors: Christoph Fröhlich, GuiHome, Daniel Zumkeller, Denis Štogl

5.1.0 (2025-04-06)
------------------
* Remove unused class variables (`#328 <https://github.com/ros-controls/control_toolbox/issues/328>`_)
* Don't update internal states if called with dt=0 or garbage (`#326 <https://github.com/ros-controls/control_toolbox/issues/326>`_)
* Move the package to a subfolder (`#318 <https://github.com/ros-controls/control_toolbox/issues/318>`_)
* Contributors: Christoph Fröhlich

5.0.0 (2025-03-13)
------------------
* Replaced gMock instead of gTest (`#300 <https://github.com/ros-controls/control_toolbox/issues/300>`_)
* Update downstream repository branches for humble (`#312 <https://github.com/ros-controls/control_toolbox/issues/312>`_)
* Update upstream/downstream repository branches (`#309 <https://github.com/ros-controls/control_toolbox/issues/309>`_)
* Make downstream job a semi-binary build (`#301 <https://github.com/ros-controls/control_toolbox/issues/301>`_)
* Cleanup duplicate entries in the msg definition (`#299 <https://github.com/ros-controls/control_toolbox/issues/299>`_)
* [Pid] Save `i_term` instead of error integral (`#294 <https://github.com/ros-controls/control_toolbox/issues/294>`_)
* Fix mergify config (`#296 <https://github.com/ros-controls/control_toolbox/issues/296>`_)
* [Pid] Remove deprecated variables and methods (`#293 <https://github.com/ros-controls/control_toolbox/issues/293>`_)
* Change workflows and readme for jazzy branch (`#292 <https://github.com/ros-controls/control_toolbox/issues/292>`_)
* Bump version of pre-commit hooks (`#288 <https://github.com/ros-controls/control_toolbox/issues/288>`_)
* Contributors: Aditya Pawar, Christoph Fröhlich, github-actions[bot]

4.0.1 (2025-02-13)
------------------
* Rename save_iterm to save_i_term (`#285 <https://github.com/ros-controls/control_toolbox/issues/285>`_)
* Bump version of pre-commit hooks (`#282 <https://github.com/ros-controls/control_toolbox/issues/282>`_)
* Use ABI workflow from ros2_control_ci (`#278 <https://github.com/ros-controls/control_toolbox/issues/278>`_)
* Use jazzy branch for realtime_tools (`#279 <https://github.com/ros-controls/control_toolbox/issues/279>`_)
* Contributors: Christoph Fröhlich, github-actions[bot]

4.0.0 (2025-01-29)
------------------
* Rework PID class API (`#246 <https://github.com/ros-controls/control_toolbox/issues/246>`_)
* [PID] Add support for saving i-term when PID is reset (`#180 <https://github.com/ros-controls/control_toolbox/issues/180>`_)
* Update codecov badge in README.md (`#273 <https://github.com/ros-controls/control_toolbox/issues/273>`_)
* Update mergify.yml (`#270 <https://github.com/ros-controls/control_toolbox/issues/270>`_)
* Branch for humble (`#265 <https://github.com/ros-controls/control_toolbox/issues/265>`_)
* Update include paths of GPL (`#264 <https://github.com/ros-controls/control_toolbox/issues/264>`_)
* Contributors: Christoph Fröhlich, Dr. Denis

3.5.0 (2025-01-15)
------------------
* Update plugin lib exception handling (`#263 <https://github.com/ros-controls/control_toolbox/issues/263>`_)
* Fix control_filters tests (`#261 <https://github.com/ros-controls/control_toolbox/issues/261>`_)
* Fix lifecycle warning in test (`#262 <https://github.com/ros-controls/control_toolbox/issues/262>`_)
* Add missing exponential filter tests and export (`#260 <https://github.com/ros-controls/control_toolbox/issues/260>`_)
* Remove visibility boilerplate code (`#258 <https://github.com/ros-controls/control_toolbox/issues/258>`_)
* Add filter plugin for exponential filter (`#231 <https://github.com/ros-controls/control_toolbox/issues/231>`_)
* Bump version of pre-commit hooks (`#255 <https://github.com/ros-controls/control_toolbox/issues/255>`_)
* change the realtime_tools header extensions (`#247 <https://github.com/ros-controls/control_toolbox/issues/247>`_)
* Contributors: Christoph Fröhlich, Julia Jia, Sai Kishor Kothakota, github-actions[bot]

3.4.0 (2024-12-08)
------------------
* Add job for clang build (`#239 <https://github.com/ros-controls/control_toolbox/issues/239>`_)
* Fix bug in rate_limiter filter and add more tests (`#237 <https://github.com/ros-controls/control_toolbox/issues/237>`_)
* Fix jerk limiter in rate_limiter (`#240 <https://github.com/ros-controls/control_toolbox/issues/240>`_)
* Add downstream build job (`#243 <https://github.com/ros-controls/control_toolbox/issues/243>`_)
* Bump version of pre-commit hooks (`#242 <https://github.com/ros-controls/control_toolbox/issues/242>`_)
* Fix mergify rules (`#241 <https://github.com/ros-controls/control_toolbox/issues/241>`_)
* Remove iron workflows and update readme (`#217 <https://github.com/ros-controls/control_toolbox/issues/217>`_)
* Minor include cleanup (`#230 <https://github.com/ros-controls/control_toolbox/issues/230>`_)
* Minor CI updates (`#236 <https://github.com/ros-controls/control_toolbox/issues/236>`_)
* Move speed limiter from ros2_control repo (`#212 <https://github.com/ros-controls/control_toolbox/issues/212>`_)
* Add semi-binary build (`#228 <https://github.com/ros-controls/control_toolbox/issues/228>`_)
* Add the same compile flags as with ros2_controllers and fix errors (`#219 <https://github.com/ros-controls/control_toolbox/issues/219>`_)
* LPF: Throw if calling `udpate` unconfigured (`#229 <https://github.com/ros-controls/control_toolbox/issues/229>`_)
* Add standalone version of LPF (`#222 <https://github.com/ros-controls/control_toolbox/issues/222>`_)
* Pid class does not depend on rclcpp (`#221 <https://github.com/ros-controls/control_toolbox/issues/221>`_)
* Change license to Apache-2 (`#220 <https://github.com/ros-controls/control_toolbox/issues/220>`_)
* Update README.md (`#215 <https://github.com/ros-controls/control_toolbox/issues/215>`_)
* Update README.md (`#214 <https://github.com/ros-controls/control_toolbox/issues/214>`_)
* Bump version of pre-commit hooks (`#213 <https://github.com/ros-controls/control_toolbox/issues/213>`_)
* Contributors: Christoph Fröhlich, Thibault Poignonec, github-actions[bot]

3.3.0 (2024-10-28)
------------------
* PID: Improve the API docs and change default value of antiwindup (`#202 <https://github.com/ros-controls/control_toolbox/issues/202>`_)
* [CI] Specify runner/container images and add Jazzy jobs (`#200 <https://github.com/ros-controls/control_toolbox/issues/200>`_)
* Add custom rosdoc2 config (`#199 <https://github.com/ros-controls/control_toolbox/issues/199>`_)
* [CI] Update pre-commit and remove ros-lint (`#187 <https://github.com/ros-controls/control_toolbox/issues/187>`_)
* Use Eigen CMake target (`#190 <https://github.com/ros-controls/control_toolbox/issues/190>`_)
* [CI] Use wf from ros2_control_ci for coverage build (`#188 <https://github.com/ros-controls/control_toolbox/issues/188>`_)
* Contributors: Christoph Fröhlich, dependabot[bot], github-actions[bot]

3.2.0 (2023-12-12)
------------------
* [CI] fix source build (`#168 <https://github.com/ros-controls/control_toolbox/issues/168>`_)
* Bump actions/setup-python from 4 to 5 (`#167 <https://github.com/ros-controls/control_toolbox/issues/167>`_)
* [CI] Touchups (`#166 <https://github.com/ros-controls/control_toolbox/issues/166>`_)
* [PID] Update documentation to reflect ROS 2 usage of time (`#165 <https://github.com/ros-controls/control_toolbox/issues/165>`_)
* Bump actions/checkout from 3 to 4 (`#163 <https://github.com/ros-controls/control_toolbox/issues/163>`_)
* Bump ros-tooling/setup-ros from 0.6 to 0.7 (`#161 <https://github.com/ros-controls/control_toolbox/issues/161>`_)
* Add filters structure and lowpass filter (`#152 <https://github.com/ros-controls/control_toolbox/issues/152>`_)
* Bump codecov/codecov-action from 3.1.2 to 3.1.4 (`#160 <https://github.com/ros-controls/control_toolbox/issues/160>`_)
* Contributors: Christoph Fröhlich, GuiHome, Patrick Roncagliolo

3.1.0 (2023-04-29)
------------------
* Check for i_min <= i_max at initialization (`#139 <https://github.com/ros-controls/control_toolbox/issues/139>`_)
* Contributors: Christoph Fröhlich

3.0.0 (2023-04-05)
------------------
* [PidROS] Enable interpreting prefix as param prefix. (`#129 <https://github.com/ros-controls/control_toolbox/issues/129>`_)
* Use std::clamp (`#140 <https://github.com/ros-controls/control_toolbox/issues/140>`_)
* [CI] Fixes and update for branch out (`#155 <https://github.com/ros-controls/control_toolbox/issues/155>`_)
* Enable subclassing of PID implementation. (`#148 <https://github.com/ros-controls/control_toolbox/issues/148>`_)
* [CI] Add Humble job (`#147 <https://github.com/ros-controls/control_toolbox/issues/147>`_)
* Finally update formatting to other repositories convention. (`#131 <https://github.com/ros-controls/control_toolbox/issues/131>`_)
* [CI] 🔧 Update pre-commit hooks and sync actions to other repositories. (`#130 <https://github.com/ros-controls/control_toolbox/issues/130>`_)
* Contributors: Bence Magyar, Christoph Fröhlich, Dr. Denis, dependabot[bot]

2.2.0 (2023-02-20)
------------------
* Fix overriding of package (`#145 <https://github.com/ros-controls/control_toolbox/issues/145>`_)
* Various dependabot version bumps
* [CI] Add dependabot configuration to automatically update actions.
* Contributors: Christoph Fröhlich, Dr. Denis, Tyler Weaver, dependabot[bot]

2.1.2 (2022-11-15)
------------------
* export missing dependency (`#128 <https://github.com/ros-controls/control_toolbox/issues/128>`_)
* Contributors: Noel Jiménez García

2.1.1 (2022-11-05)
------------------
* Add declaration of parameters in ROSPid.
* Fix namespace collision and parameter_callback problems in PidROS
* Contributors: Aris Synodinos, Denis Štogl

2.1.0 (2022-06-30)
------------------
* Fix parameter loading log levels
* Support pass in a precomputed derivative error
* Add getParametersCallbackHandle function
* Add topic_prefix\_ to declareParam & setParameter
* Update include/control_toolbox/dither.hpp
* Correct contributing and license files for ament_copyright.
* Added license text file and contributing guidelines, corrected license short identifier.
* Remove build of downstream workspace.
* Update CI config and add pre-commit-config.
* Contributors: Bence Magyar, ChenJun, Denis Štogl, Timon Engelke

2.0.2 (2021-05-25)
------------------
* remove unused variables
* Update visibility_control.hpp
* Windows bringup.
* Contributors: Karsten Knese, Sean Yen, Bence Magyar

2.0.1 (2020-08-01)
------------------
* Fix dependencies
* Export ament_cmake build type
* Contributors: ahcorde

2.0.0 (2020-07-28)
------------------
* Refactor the Pid class to be completely ROS agnostic and added a ROS 2 wrapper
* Avoid crash when the type of the parameter doesn't match
* Added topic_prefix to publisher topic name (`#95 <https://github.com/ros-controls/control_toolbox/issues/95>`_)
* Created a shared library (`#93 <https://github.com/ros-controls/control_toolbox/issues/93>`_)
* Aliases not part of the public API are now private
* Removing pid_gains_setter
* Removed unnecessary dependencies
* Cleared empty non virtual destructors
* Removed unused limited proxy variables
* Added pid state real-time publisher
* Removed all references to tinyxml
* Removed tune_pid.py
* Adding missing copyright licenses
* Adapted dither, sine_sweep and sinusoid to ROS2
* Removed dynamic reconfigure completely
* Removed deprecated functions
* Contributors: Alejandro Hernández Cordero, Bence Magyar, James Xu, Jordan Palacios, Shane Loretz, ahcorde

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
* Enforcing ``i_min_`` <= 0 and ``i_max_`` >= 0 in integral bound parameters, reducing duplicated code
* Merge pull request `#14 <https://github.com/ros-controls/control_toolbox/issues/14>`_ from bobholmberg/fix-PID-unbounded-i_error
  Using zero ``i_gain_`` to turn off integral control did unsavory things.
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
* If the user did not want integral control and set ``i_gain_`` to zero,
  then dividing by ``i_gain_`` would set ``i_error_`` to NaN.  This is not
  desired.  Instead, replace the use of division to create ``i_term``
  with direct integration of ``i_term_``.
  Replace private member ``i_error_`` with ``i_term_``.
  In ``getCurrentPIDErrors()`` create & return ``i_error_`` with the same old meaning and units.
  NOTE: ``i_error_`` is not needed internally anywhere else.
* Cleaning up documentation, making argument names in function declaration match those in the implementation
* adding doxygen deprecation flags
* Fixing documentation
* Merging changes from other branch
* Adding conventional PID computation
* Fixing inconsistent formatting, and reducing some duplicated code
* remove .svn folder
* move control_toolbox into ros_control
* Contributors: Adolfo Rodriguez Tsouroukdissian, Austin Hendrix, Bob Holmberg, Jonathan Bohren, Wim Meeussen, wmeeusse
