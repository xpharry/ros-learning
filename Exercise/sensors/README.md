Realization of PDF file:

simulating_sensors_in_gazebo.pdf

	Adding a Kinect sensor to the robot model URDF

procedures:

	1. roscore

	2. rosrun gazebo_ros gazebo

	3. navigate to the directory: cd ~/catkin_ws/src/minimal_robot_description/

	4. rosrun gazebo_ros spawn_model -file minimal_robot_w_sensor.urdf -urdf -model one_DOF_robot

	5. rosrun minimal_joint_controller minimal_joint_controller

	6. roslaunch minimal_robot_description minimal_robot_w_sensor.launch

	7. rosrun robot_state_publisher robot_state_publisher

	8. rosrun rviz rviz

	or just use the following command to substitude the above operations:

	* roslaunch minimal_robot_description minimal_robot.launch

	9 rostopic pub pos_cmd std_msgs/Float64 0.5

there is some problem in rviz showing!!

	PointCloud2: transform

		Solved: <frameName>kinect_pc_frame</frameName> in minimal_robot_w_sensor.urdf need to be modified as
			<frameName>kinect_sensor_frame</frameName>

	
