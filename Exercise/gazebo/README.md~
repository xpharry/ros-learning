Realization of PDF file:

intro_to_gazebo.pdf

	Adding Dynamic Information to the URDF

	The Gazebo simulator

	A minimal joint controller

	Displaying the robot in Rviz

operations:

	1. roscore

	2. rosrun gazebo_ros gazebo

		check in gazebo

	3. navigate to the folder “minimal_robot_description” first, then command

		cd ~/catkin_ws/src/minimal_robot_description

	   	rosrun gazebo_ros spawn_model -file minimal_robot_description.urdf -urdf -model one_DOF_robo

			check in gazebo

	*4. rosservice call /gazebo/get_joint_properties "joint1" # check the joint1 properties

	5. rostopic pub pos_cmd std_msgs/Float64 1.0 

		check in gazebo & rqtplot

	   rqt_plot

	6 Dislay in rviz:

		1) roslaunch minimal_robot_description minimal_robot_description.launch

		2) rosrun robot_state_publisher robot_state_publisher

			check in rviz


