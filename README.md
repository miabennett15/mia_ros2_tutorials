# HDI Backpack Robot Operating System 2 (ROS2) Workspace for Indoor Mapping

<!-- HDI Way of Working: Badge Holder Start -->
[![HDI Way of Working](https://img.shields.io/badge/HDI-Way%20of%20Working-8169e3?labelColor=000)](https://healthdatainsight.github.io/way_of_working/)
<!-- HDI Way of Working: Badge Holder End -->


https://www.youtube.com/watch?v=Gg25GfA456o
https://www.youtube.com/c/RoboticsBackEnd

## Setup

### Clone the repository

  ```bash
  cd ~
  git clone https://github.com/HealthDataInsight/hdi_backpack_ros2_ws ros2_ws
  ```

## Build the project

Build the project with

  ```bash
  cd ~/ros2_ws
  make build_all
  ```

If you see the error:

  ```bash
  /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  ```

You will need to downgrade setuptools, with:

  ```bash
  pip3 install setuptools==58.2.0
  ```

and re-run `make build_all`.

### Setup environment

Source the workspace in your `.bashrc` file.

  ```bash
  # Replace ".bash" with your shell if you're not using bash
  # Possible values are: setup.bash, setup.sh, setup.zsh
  echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```
