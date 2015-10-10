often use:

	. ~/catkin_ws/devel/setup.bash

operations:

	1. roscore

	2. rosrun gazebo_ros gazebo

		check in gazebo

	3. navigate to the folder “robot_description” first:

		cd ~/catkin_ws/src/robot_description

	4. rosrun gazebo_ros spawn_model -file two_dof_robot.urdf -urdf -model two_DOF_robot

		check in gazebo

	5. roslaunch joints_controller joints_controller.launch

	6. rqt_plot

		check jnt1_pos_cmd & jnt1_pos_cmd ...
	

