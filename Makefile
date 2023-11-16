SHELL := /bin/bash

ROBOT_NAME="hdi_backpack"
# Usage: make create_node name=controller
create_node:
	cd src/
	ros2 pkg create $(ROBOT_NAME)_$(name) --build-type ament_python --dependencies rclpy
	cd ..

# Install dependencies, not available through rosdep
setup:
	sudo apt install -y python3-tf2-ros python3-transforms3d

build_bringup:
	@echo -e "\nBuilding backpack_bringup...\n"
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	rosdep install --from-paths src/backpack_bringup -y --ignore-src
	colcon build --packages-select backpack_bringup --symlink-install

build_bno055:
	@echo -e "\nBuilding bno055...\n"
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	rosdep install --from-paths src/bno055 -y --ignore-src
	colcon build --packages-select bno055

build_imu_tf:
	@echo -e "\nBuilding imu_tf...\n"
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	rosdep install --from-paths src/imu_tf -y --ignore-src
	colcon build --packages-select imu_tf --symlink-install

build_ldlidar_stl_ros2:
	@echo -e "\nBuilding ldlidar_stl_ros2...\n"
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	rosdep install --from-paths src/ldlidar_stl_ros2 -y --ignore-src
	colcon build --packages-select ldlidar_stl_ros2

# nmea_navsat_driver and two dependencies are not available as packages and need to be included and built locally
build_nmea_navsat_driver:
	@echo -e "\nBuilding nmea_navsat_driver and local dependencies...\n"
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	rosdep install --from-paths src/nmea_msgs src/nmea_navsat_driver src/tf_transformations -y --ignore-src
	colcon build --packages-select nmea_msgs nmea_navsat_driver tf_transformations --symlink-install

build_ros_foxglove_bridge:
	@echo -e "\nBuilding foxglove_bridge...\n"
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	rosdep install --from-paths src/ros-foxglove-bridge -y --ignore-src --os=debian:bullseye
	colcon build --packages-select foxglove_bridge

build_system_stats_pkg:
	@echo -e "\nBuilding system_stats_pkg...\n"
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	rosdep install --from-paths src/system_stats_pkg -y --ignore-src
	colcon build --packages-select system_stats_pkg --symlink-install

build_all: build_bno055 build_imu_tf build_ldlidar_stl_ros2 build_nmea_navsat_driver build_ros_foxglove_bridge build_system_stats_pkg build_bringup
	@echo -e "\nNow run: source ./install/setup.bash\n"

build_update: setup
	rm -rf rm build/ install/ log/
	sudo apt update
	git submodule update --init --recursive --remote --force
	rosdep update

update_and_build: build_update build_all

launch:
	ros2 launch backpack_bringup backpack_app.launch.py

launch_without_sensors:
	ros2 launch backpack_bringup backpack_app.launch.py run_sensors:=False

test:
	colcon test-result --all --delete-yes
	colcon test --ctest-args tests --packages-select backpack_bringup system_stats_pkg
	colcon test-result --all --verbose
