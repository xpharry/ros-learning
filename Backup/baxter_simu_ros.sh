
mkdir -p ~/ros_ws/src

(cd ~/ros_ws/src && catkin_init_workspace)

(cd ~/ros_ws/src && git clone https://github.com/cwru-robotics/cwru_baxter.git)

(cd ~/ros_ws/src && git clone https://github.com/cwru-robotics/external_packages.git)

(cd ~/ros_ws/src && git clone https://github.com/cwru-robotics/cwru_msgs.git)

(cd ~/ros_ws/src && git clone https://github.com/cwru-robotics/eecs-397-f15.git)

mkdir ~/ros_ws/src/rethink

(cd ~/ros_ws/src/rethink && git clone https://github.com/RethinkRobotics/baxter_interface.git)
(cd ~/ros_ws/src/rethink && git clone https://github.com/RethinkRobotics/baxter_common.git)
(cd ~/ros_ws/src/rethink && git clone https://github.com/RethinkRobotics/baxter_tools.git)

sudo apt-get install ros-indigo-controller-interface ros-indigo-gazebo-ros-control ros-indigo-joint-state-controller ros-indigo-effort-controllers ros-indigo-moveit-msgs

(cd ~/ros_ws && catkin_make clean)
(cd ~/ros_ws && catkin_make --pkg cwru_srv)

(cd ~/ros_ws && catkin_make --pkg baxter_core_msgs)
(cd ~/ros_ws && catkin_make --pkg baxter_traj_streamer)
(cd ~/ros_ws && catkin_make --pkg cartesian_moves)

(cd ~/ros_ws && catkin_make)
(cd ~/ros_ws && catkin_make install)

