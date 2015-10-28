run the script baxter_simu.sh as preparation

		chmod 775 baxter_simu.sh 

		./baxter_simu.sh 

run baxter simulator

	1. run and wait for the message: "Gravity compensation was tuned off"

		roslaunch cwru_baxter_sim baxter_world.launch

	2. in another window, enable the robot with the command: 
  
		rosrun baxter_tools enable_robot.py -e

	   This command will run to completion.

	3. Start the trajectory interpolator action server:

		rosrun baxter_traj_streamer traj_interpolator_as

	   Leave this node running.

	4. In another terminal, start your node.  For test purposes, run the example client node:
  
		rosrun baxter_traj_streamer traj_action_client_pre_pose
