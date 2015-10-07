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

	5. rosrun joint1_controller joint1_controller

	*5.5. rostopic pub jnt1_pos_cmd std_msgs/Float64 0

	6. rosrun joint2_commander joint2_commander

	7. rosrun joint1_controller joint1_controller

	*7.5. rostopic pub jnt1_pos_cmd std_msgs/Float64 0

	8. rosrun joint2_commander joint2_commander

	9. rqt_plot

		check jnt1_pos_cmd & jnt1_pos_cmd ...
	


