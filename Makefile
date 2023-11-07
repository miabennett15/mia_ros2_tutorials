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
	git submodule update --init --recursive --remote
	rosdep update
	ln -sf $(pwd)/src/ros_imu_tools/imu_tf $(pwd)/src/imu_tf

update_and_build: build_update build_all

launch:
	ros2 launch backpack_bringup backpack_app.launch.py
