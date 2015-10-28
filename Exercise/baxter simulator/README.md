

	mkdir -p ~/catkin_ws/src

	(cd ~/catkin_ws/src && catkin_init_workspace)

	(cd ~/catkin_ws/src && git clone https://github.com/cwru-robotics/cwru_baxter.git)

	(cd ~/catkin_ws/src && git clone https://github.com/cwru-robotics/external_packages.git)
	
	(cd ~/catkin_ws/src && git clone https://github.com/cwru-robotics/cwru_msgs.git)

	(cd ~/catkin_ws/src && git clone https://github.com/cwru-robotics/eecs-397-f15.git)

	mkdir ~/catkin_ws/src/rethink

	(cd ~/catkin_ws/src/rethink && git clone https://github.com/RethinkRobotics/baxter_interface.git)
	(cd ~/catkin_ws/src/rethink && git clone https://github.com/RethinkRobotics/baxter_common.git)
	(cd ~/catkin_ws/src/rethink && git clone https://github.com/RethinkRobotics/baxter_tools.git)
	
	sudo apt-get install ros-indigo-controller-interface ros-indigo-gazebo-ros-control ros-indigo-joint-state-controller ros-indigo-effort-controllers ros-indigo-moveit-msgs

	(cd ~/catkin_ws && catkin_make clean)
	(cd ~/catkin_ws && catkin_make --pkg cwru_srv)
	
	(cd ~/catkin_ws && catkin_make --pkg baxter_core_msgs)
	(cd ~/catkin_ws && catkin_make --pkg baxter_traj_streamer)
	(cd ~/catkin_ws && catkin_make --pkg cartesian_moves)

	(cd ~/catkin_ws && catkin_make)
	(cd ~/catkin_ws && catkin_make install)

