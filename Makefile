ROBOT_NAME="hdi_backpack"
# Usage: make create_node name=controller
create_node:
	cd src/
	ros2 pkg create $(ROBOT_NAME)_$(name) --build-type ament_python --dependencies rclpy
	cd ..

build_bringup:
	colcon build --packages-select backpack_bringup --symlink-install

build_all: build_bringup

launch:
	ros2 launch backpack_bringup backpack_app.launch.py
