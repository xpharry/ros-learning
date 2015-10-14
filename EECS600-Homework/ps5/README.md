procedures:

	1. roscore

	2. roslaunch robot_description two_dof_robot.launch

		equivalent to following #2.1 -#2.4

	#2.1 rosrun gazebo_ros gazebo

		check in gazebo

	#2.2 navigate to the folder “robot_description” first:

		cd ~/catkin_ws/src/robot_description

	#2.3 rosrun gazebo_ros spawn_model -file two_dof_robot.urdf -urdf -model two_DOF_robot

		check in gazebo

	#2.4 roslaunch joints_controller joints_controller.launch

	3. rosrun robot_trajectory robot_trajectory_action_server

	4. rosrun robot_trajectory robot_trajectory_action_client

	5. rqt_plot

		check jnt1_pos_cmd & jnt1_pos_cmd ...
	


