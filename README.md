control_toolbox
===========

See [ros_control](http://wiki.ros.org/ros_control) and [control_toolbox](http://wiki.ros.org/control_toolbox) documentation on ros.org



### Build Status

[![Build Status](https://travis-ci.org/ros-controls/control_toolbox.png?branch=kinetic-devel)](https://travis-ci.org/ros-controls/control_toolbox)

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


## Code Formatting

This repository uses `pre-commit` tool for code formatting.
The tool checks formatting each time you commit to a repository.
To install it locally use:
  ```
  pip3 install pre-commit  # (prepend `sudo` if you want to install it system wide)
  ```

To run it initially over the whole repo you can use:
  ```
  pre-commit run -a
  ```

If you get error that something is missing on your computer, do the following for:

  - `clang-format-10`
     ```
     sudo apt install clang-format-10
     ```
