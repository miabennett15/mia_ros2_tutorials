SHELL := /bin/bash

ROBOT_NAME="hdi_backpack"
# Usage: make create_node name=controller
create_node:
	cd src/
	ros2 pkg create $(ROBOT_NAME)_$(name) --build-type ament_python --dependencies rclpy
	cd ..

dependencies:
	@echo -e "\nBuilding dependencies...\n"
	# Install dependencies, not available through rosdep
	sudo apt install -y python3-tf2-ros python3-transforms3d
	mkdir -p ~/ros2_ws_dependencies/src
	cd ~/ros2_ws_dependencies && \
	  vcs import --input ~/ros2_ws/dependency.repos src && \
	  sudo apt upgrade && \
	  rosdep update && \
 	  rosdep install --from-paths src/ros-foxglove-bridge -y --ignore-src --os=debian:bullseye && \
	  rosdep install --from-paths src --ignore-src -y --skip-keys "nlohmann-json-dev" && \
	  colcon build --symlink-install
	@echo -e "\nNow run: source ~/ros2_ws_dependencies/install/setup.bash\n"

build_bringup:
	@echo -e "\nBuilding backpack_bringup...\n"
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	rosdep install --from-paths src/backpack_bringup -y --ignore-src
	colcon build --packages-select backpack_bringup --symlink-install

build_system_stats_pkg:
	@echo -e "\nBuilding system_stats_pkg...\n"
	if [ -f install/setup.bash ]; then source install/setup.bash; fi
	rosdep install --from-paths src/system_stats_pkg -y --ignore-src
	colcon build --packages-select system_stats_pkg --symlink-install

build_all:
	rm -rf rm build/ install/ log/
	rosdep install --from-paths src -y --ignore-src
	colcon build --symlink-install
	@echo -e "\nNow run: source ./install/setup.bash\n"

update_and_build: dependencies build_all

launch:
	ros2 launch backpack_bringup backpack_app.launch.py

launch_without_sensors:
	ros2 launch backpack_bringup backpack_app.launch.py run_sensors:=False

# Usage: make record_sensor_bag name=test_calibration_bag
record_sensor_bag:
	ros2 bag record /fix /heading /time_reference /vel \
	                /bno055/calib_status /bno055/mag /bno055/temp /imu/data /imu/data_raw \
	                /scan \
	                /system_stats/cpu_usage /system_stats/disk_usage /system_stats/swap_mem_usage /system_stats/virtual_mem_usage \
			-o bags/$(name)

test:
	colcon test-result --all --delete-yes
	colcon test --ctest-args tests
	colcon test-result --all --verbose
