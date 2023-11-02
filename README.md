# HDI Backpack Robot Operating System 2 (ROS2) Workspace for Indoor Mapping

https://www.youtube.com/watch?v=Gg25GfA456o
https://www.youtube.com/c/RoboticsBackEnd

## Setup

    $ cd ~
    $ git clone https://github.com/HealthDataInsight/hdi_backpack_ros2_ws ros2_ws

Build the project with

    $ colcon build

If you see the error:

    /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.

You will need to downgrade setuptools, with:

    $ pip3 install setuptools==58.2.0
