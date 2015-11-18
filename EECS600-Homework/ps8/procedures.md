1 start baxter simulator

	roslaunch cwru_baxter_sim baxter_world.launch

2 enable robot

	rosrun baxter_tools enable_robot.py -e

3 start kinect simulator

	roslaunch cwru_baxter_sim kinect_xform.launch

4 start rviz

	rosrun rviz rviz
