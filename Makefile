ROBOT_NAME="hdi_backpack"
# Usage: make create_node name=controller
create_node:
	cd src/
	ros2 pkg create $(ROBOT_NAME)_$(name) --build-type ament_python --dependencies rclpy
	cd ..

build_bringup:
	colcon build --packages-select backpack_bringup --symlink-install

build_all:
	rosdep install --from-paths src -y --ignore-src
	colcon build --symlink-install
	@echo "\nNow run: source ./install/setup.bash"

build_update:
	sudo apt update
	rm -r src/imu_tf
	git submodule update --init --recursive --remote --force
	rosdep update
	mv ${HOME}/ros2_ws/src/ros_imu_tools/imu_tf ${HOME}/ros2_ws/src/imu_tf
	rm -rf ${HOME}/ros2_ws/src/ros_imu_tools

update_and_build: build_update build_all

launch:
	ros2 launch backpack_bringup backpack_app.launch.py
