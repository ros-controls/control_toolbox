control_toolbox
===========
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codecov](https://codecov.io/gh/ros-controls/control_toolbox/graph/badge.svg?token=0o4dFzADHj)](https://codecov.io/gh/ros-controls/control_toolbox)

This package contains several C++ classes useful in writing controllers.

See the documentation of [ros2_control](http://control.ros.org) and release infos on [index.ros.org](http://index.ros.org/p/control_toolbox).

## Build status
ROS2 Distro | Branch | Build status | Documentation | Package build
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`ros2-master`](https://github.com/ros-controls/control_toolbox/tree/ros2-master) | [![Binary Build](https://github.com/ros-controls/control_toolbox/actions/workflows/rolling-build-binary.yml/badge.svg?branch=ros2-master)](https://github.com/ros-controls/control_toolbox/actions/workflows/rolling-build-binary.yml?branch=ros2-master) <br> [![Source Build](https://github.com/ros-controls/control_toolbox/actions/workflows/rolling-build-source.yml/badge.svg?branch=ros2-master)](https://github.com/ros-controls/control_toolbox/actions/workflows/rolling-build-source.yml?branch=ros2-master) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Rdev__control_toolbox__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Rdev__control_toolbox__ubuntu_noble_amd64/) | [API](http://docs.ros.org/en/rolling/p/control_toolbox/)  | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__realtime_tools__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__realtime_tools__ubuntu_noble_amd64__binary/)
**Jazzy** | [`jazzy`](https://github.com/ros-controls/control_toolbox/tree/jazzy) | [![Jazzy Binary Build](https://github.com/ros-controls/control_toolbox/actions/workflows/jazzy-build-binary.yml/badge.svg?branch=ros2-master)](https://github.com/ros-controls/control_toolbox/actions/workflows/jazzy-build-binary.yml?branch=ros2-master) <br> [![Jazzy Source Build](https://github.com/ros-controls/control_toolbox/actions/workflows/jazzy-build-source.yml/badge.svg?branch=ros2-master)](https://github.com/ros-controls/control_toolbox/actions/workflows/jazzy-build-source.yml?branch=ros2-master) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Jdev__control_toolbox__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Jdev__control_toolbox__ubuntu_noble_amd64/) | [API](http://docs.ros.org/en/jazzy/p/control_toolbox/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__realtime_tools__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__realtime_tools__ubuntu_noble_amd64__binary/)
**Humble** | [`humble`](https://github.com/ros-controls/control_toolbox/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/control_toolbox/actions/workflows/humble-build-binary.yml/badge.svg?branch=ros2-master)](https://github.com/ros-controls/control_toolbox/actions/workflows/humble-build-binary.yml?branch=ros2-master) <br> [![Humble Source Build](https://github.com/ros-controls/control_toolbox/actions/workflows/humble-build-source.yml/badge.svg?branch=ros2-master)](https://github.com/ros-controls/control_toolbox/actions/workflows/humble-build-source.yml?branch=ros2-master) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Hdev__control_toolbox__ubuntu_jammy_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Hdev__control_toolbox__ubuntu_jammy_amd64/) | [API](http://docs.ros.org/en/humble/p/control_toolbox/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__realtime_tools__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__realtime_tools__ubuntu_jammy_amd64__binary/)

## Publication

If you find this work useful please give credits to the authors by citing:

* S. Chitta, E. Marder-Eppstein, W. Meeussen, V. Pradeep, A. Rodríguez Tsouroukdissian, J. Bohren, D. Coleman, B. Magyar, G. Raiola, M. Lüdtke and E. Fernandez Perdomo
**"ros_control: A generic and simple control framework for ROS"**,
The Journal of Open Source Software, 2017. ([PDF](http://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf))

```
@article{ros_control,
author = {Chitta, Sachin and Marder-Eppstein, Eitan and Meeussen, Wim and Pradeep, Vijay and Rodr{\'i}guez Tsouroukdissian, Adolfo  and Bohren, Jonathan and Coleman, David and Magyar, Bence and Raiola, Gennaro and L{\"u}dtke, Mathias and Fern{\'a}ndez Perdomo, Enrique},
title = {ros\_control: A generic and simple control framework for ROS},
journal = {The Journal of Open Source Software},
year = {2017},
doi = {10.21105/joss.00456},
URL = {http://www.theoj.org/joss-papers/joss.00456/10.21105.joss.00456.pdf}
}
```
