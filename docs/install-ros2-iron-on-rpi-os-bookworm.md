# Raspberry Pi OS (source)

## System requirements

Follow these instructions to install Iron Irwini on:

- Raspberry Pi OS Debian Linux - Bookworm (12) 64-bit

They are based on:

- [Iron Irwini Ubuntu (source) installation instructions](https://docs.ros.org/en/iron/Installation/Alternatives/Ubuntu-Development-Setup.html)
- [Raspberry Pi + ROS 2 + Camera](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304)

with further amendments.

## System setup

### Set locale

Make sure you have a locale which supports `UTF-8`.
If you are in a minimal environment (such as a docker container), the locale may be something minimal like `POSIX`.
We test with the following settings. However, it should be fine if you're using a different UTF-8 supported locale.

  ```bash
  locale  # check for UTF-8

  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  locale  # verify settings
  ```

### Enable required repositories

You will need to add the ROS 2 apt repository to your system.

~First ensure that the `Ubuntu Universe repository <https://help.ubuntu.com/community/Repositories/Ubuntu>`_ is enabled.~

  ```bash
  sudo apt install software-properties-common
  ```

Now add the ROS 2 GPG key with apt.

  ```bash
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  ```

Then add the repository to your sources list.

  ```bash
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```

### Install development tools

  ```bash
  sudo apt update && sudo apt install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-rerunfailures \
    ros-dev-tools
  ```

## Build ROS 2

### Get ROS 2 code

Create a workspace and clone all repos:

  ```bash
  mkdir -p ~/ros2_iron/src
  cd ~/ros2_iron
  vcs import --input https://raw.githubusercontent.com/ros2/ros2/iron/ros2.repos src
  ```

### Install dependencies using rosdep

ROS 2 packages are built on frequently updated Ubuntu systems.
It is always recommended that you ensure your system is up to date before installing new packages.

  ```bash
  sudo apt upgrade

  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
  ```

~**Note**: If you're using a distribution that is based on Ubuntu (like Linux Mint) but does not identify itself as such, you'll get an error message like `Unsupported OS [mint]`. In this case append `--os=ubuntu:jammy` to the above command.~

### Ignore some subtrees

  ```bash
  cd ~/ros2_iron/
  touch src/ros2/rviz/AMENT_IGNORE
  touch src/ros-visualization/AMENT_IGNORE
  touch src/ros2/system_tests/AMENT_IGNORE
  ```

### Use Clang compilers and build the code in the workspace

Configure CMake to detect and use Clang:

  ```bash
  sudo apt install clang
  export CC=clang
  export CXX=clang++
  ```

~If you have already installed ROS 2 another way (either via Debians or the binary distribution), make sure that you run the below commands in a fresh environment that does not have those other installations sourced.
Also ensure that you do not have `source /opt/ros/${ROS_DISTRO}/setup.bash` in your `.bashrc`.
You can make sure that ROS 2 is not sourced with the command `printenv | grep -i ROS`.
The output should be empty.~

More info on working with a ROS workspace can be found in [this tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html).

  ```bash
  cd ~/ros2_iron/
  colcon build --cmake-force-configure --symlink-install
  ```

{: .note }
> If you are having trouble compiling all examples and this is preventing you from completing a successful build, you can use the `--packages-skip` colcon flag to ignore the package that is causing problems.
>
> For instance, if you don't want to install the large OpenCV library, you could skip building the packages that depend on it using the command:
>
> `colcon build --symlink-install --packages-skip image_tools intra_process_demo`

## Setup environment

Set up your environment by sourcing the ROS setup in your `.bashrc` file.

  ```bash
  # Replace ".bash" with your shell if you're not using bash
  # Possible values are: setup.bash, setup.sh, setup.zsh
  echo "source $HOME/ros2_iron/install/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

## Try some examples

In one terminal, source the setup file and then run a C++ `talker`\ :

  ```bash
  source ~/.bashrc
  ros2 run demo_nodes_cpp talker
  ```

In another terminal source the setup file and then run a Python `listener`\ :

  ```bash
  source ~/.bashrc
  ros2 run demo_nodes_py listener
  ```

You should see the `talker` saying that it's `Publishing` messages and the `listener` saying `I heard` those messages.
This verifies both the C++ and Python APIs are working properly.
Hooray!

## Next steps

Continue with the [tutorials and demos](https://docs.ros.org/en/iron/Tutorials.html) to configure your environment, create your own workspace and packages, and learn ROS 2 core concepts.

## Use the ROS 1 bridge (optional)

The ROS 1 bridge can connect topics from ROS 1 to ROS 2 and vice-versa.
See the dedicated [document](https://docs.ros.org/en/iron/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html) on how to build and use the ROS 1 bridge.

## Stay up to date

See [Maintain source checkout](https://docs.ros.org/en/iron/Installation/Maintaining-a-Source-Checkout.html) to periodically refresh your source installation.

## Troubleshoot

Troubleshooting techniques can be found [here](https://docs.ros.org/en/iron/How-To-Guides/Installation-Troubleshooting.html#linux-troubleshooting).

## Uninstall

1. If you installed your workspace with colcon as instructed above, "uninstalling" could be just a matter of opening a new terminal and not sourcing the workspace's `setup` file.
   This way, your environment will behave as though there is no Iron install on your system.

2. If you're also trying to free up space, you can delete the entire workspace directory with:

  ```bash
  rm -rf ~/ros2_iron
  ```
