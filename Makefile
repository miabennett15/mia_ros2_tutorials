ROBOT_NAME="hdi_backpack"
# Usage: make create_node name=controller
create_node:
	cd src/
	ros2 pkg create $(ROBOT_NAME)_$(name) --build-type ament_python --dependencies rclpy
	cd ..

build_bringup:
	colcon build --packages-select backpack_bringup --symlink-install

build_all:
	colcon build --symlink-install
	. install/setup.bash

launch:
	ros2 launch backpack_bringup backpack_app.launch.py
