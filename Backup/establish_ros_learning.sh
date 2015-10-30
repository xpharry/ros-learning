
mkdir -p ~/ROS-learning/Reference
cd ~/ROS-learning/Reference
git init
git clone https://github.com/cwru-robotics/eecs-397-f15.git
git clone https://github.com/cwru-robotics/cwru_baxter.git
git clone https://github.com/cwru-robotics/cwru_msgs.git
git clone https://github.com/cwru-robotics/external_packages.git
git clone https://github.com/cwru-robotics/cwru-ros-pkg.git
git clone https://github.com/fairlight1337/ros_service_examples.git
git clone https://github.com/ros/ros_tutorials.git

mkdir -p ~/ROS-learning/Reference/rethink
cd ~/ROS-learning/Reference/rethink
(cd ~/ROS-learning/Reference/rethink && git clone https://github.com/RethinkRobotics/baxter_interface.git)
(cd ~/ROS-learning/Reference/rethink && git clone https://github.com/RethinkRobotics/baxter_common.git)
(cd ~/ROS-learning/Reference/rethink && git clone https://github.com/RethinkRobotics/baxter_tools.git)
