# HDI Backpack Robot Operating System 2 (ROS2) Workspace for Indoor Mapping

<!-- HDI Way of Working: Badge Holder Start -->
[![HDI Way of Working](https://img.shields.io/badge/HDI-Way%20of%20Working-8169e3?labelColor=000)](https://healthdatainsight.github.io/way_of_working/)
[![Inclusive Language](https://github.com/HealthDataInsight/hdi_backpack_ros2_ws/actions/workflows/inclusive-language.yml/badge.svg)](https://github.com/HealthDataInsight/hdi_backpack_ros2_ws/actions/workflows/inclusive-language.yml)
<!-- HDI Way of Working: Badge Holder End -->

## Documentation

* [Installing ROS2 (Iron) on Raspberry Pi OS (Bookworm)](docs/install-ros2-iron-on-rpi-os-bookworm.md)
* [Raspberry Pi interface configuration](docs/rpi-interface-configuration.md)
* [Project Decision Records](https://github.com/HealthDataInsight/indoor-mapping/blob/main/docs/decisions)
* [Useful links](https://github.com/HealthDataInsight/indoor-mapping/blob/main/docs/useful-links.md)

## Setup

### Clone the repository

Clone the repository and initialise submodules with

  ```bash
  cd ~
  git clone https://github.com/HealthDataInsight/hdi_backpack_ros2_ws ros2_ws
  cd ~/ros2_ws
  ```

## Build the project

Build the project dependencies with:

  ```bash
  cd ~/ros2_ws
  make dependencies
  echo "source $HOME/ros2_ws_dependencies/install/setup.bash" >> ~/.bashrc
  ```

Build the project with:

  ```bash
  cd ~/ros2_ws
  source ~/.bashrc
  make update_and_build
  ```

If you see the error:

  ```bash
  /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  ```

You will need to downgrade setuptools, with:

  ```bash
  pip3 install setuptools==58.2.0
  ```

and re-run `make update_and_build`.

### Setup environment

Source the workspace in your `.bashrc` file.

  ```bash
  # Replace ".bash" with your shell if you're not using bash
  # Possible values are: setup.bash, setup.sh, setup.zsh
  echo "source $HOME/ros2_ws/install/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

### Running the backpack robot

To run the robot on the backpack Raspberry Pi (with sensor driver nodes running), use:

  ```bash
  ros2 launch backpack_bringup backpack_app.launch.py
  ```

To run the robot on any machine replaying sensor data bags, use:

  ```bash
  ros2 launch backpack_bringup backpack_app.launch.py run_sensors:=False
  ```

### Recording sensor data bags

To record all the topics published by backpack sensors and save it in a bag called `test_calibration_bag`, for example, use:

  ```bash
  make record_sensor_bag name=test_calibration_bag
  ```

It will be saved in the `bags` folder. The folder exists for convenience, but please do not force commit any bags into this repository.

If recorded remotely, use SCP to copy it to your current local folder with:

  ```bash
  scp -r <username>@<server_name>:/home/<username>/ros2_ws/bags/test_calibration_bag .
  ```

If you add a new sensor to the robot, please add the sensor topics to the `record_sensor_bag` task in the `Makefile`.

### Testing with bags

Setup the imu-to-transform converter (make sure you source the install)

  ```bash
  ros2 run tf_imu transform
  ```

Record on `/tf`:

  ```bash
  ros2 bag record tf
  ```

You can play back a bag and remap topics with

  ```bash
  ros2 bag play data/test_moving_imu --remap bno055/imu:=imu/data
  ```

### Running automated tests and linter

To run all the colcon tests and flake8 linter, run:

  ```bash
  make test
  ```
