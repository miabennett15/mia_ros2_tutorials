ROBOT_NAME="hdi_backpack"
# Usage: make create_node name=controller
create_node:
	cd src/
	ros2 pkg create $(ROBOT_NAME)_$(name) --build-type ament_python --dependencies rclpy
	cd ..

build_bringup:
	rosdep install --from-paths src/backpack_bringup -y --ignore-src
	colcon build --packages-select backpack_bringup --symlink-install

build_bno055:
	rosdep install --from-paths src/bno055 -y --ignore-src
	colcon build --packages-select bno055

build_imu_tf:
	rosdep install --from-paths src/imu_tf -y --ignore-src
	colcon build --packages-select imu_tf --symlink-install

build_ldlidar_stl_ros2:
	rosdep install --from-paths src/ldlidar_stl_ros2 -y --ignore-src
	colcon build --packages-select ldlidar_stl_ros2

build_ros_foxglove_bridge:
	rosdep install --from-paths src/ros-foxglove-bridge -y --ignore-src --os=debian:bullseye
	colcon build --packages-select foxglove_bridge

build_all: build_bringup build_bno055 build_imu_tf build_ldlidar_stl_ros2 build_ros_foxglove_bridge
	@echo "\nNow run: source ./install/setup.bash"

build_update:
	sudo apt update
	git submodule update --init --recursive --remote --force
	rosdep update

update_and_build: build_update build_all

launch:
	ros2 launch backpack_bringup backpack_app.launch.py
