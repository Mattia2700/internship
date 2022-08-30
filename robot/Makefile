SHELL := /bin/bash

home = ${HOME}
current_dir = ${PWD}

clean:
	@echo "Cleaning build directories"
	@rm -rf $(current_dir)/build $(current_dir)/devel

build:
	@echo "Building ROS1 packages"
	@source /opt/ros/noetic/setup.bash && catkin_make

run: 
	@echo "Running navigation"
	@source $(current_dir)/devel/setup.bash && roslaunch ros_interface real_robot.launch

bridge:
	@echo "Running bridge"
	@source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics

.PHONY: clean build run

