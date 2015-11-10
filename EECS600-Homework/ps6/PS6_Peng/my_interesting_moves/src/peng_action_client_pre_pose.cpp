// traj_action_client_pre_pose: 
// wsn, June, 2015
// uses traj_interpolator_as to send robot to pre pose--right arm lifted/retracted for grasp from above

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_interesting_moves/my_interesting_moves.h>
//this #include refers to the new "action" message defined for this package
// the action message can be found in: .../my_interesting_moves/action/traj.action
// automated header generation creates multiple headers for message I/O
// these are referred to by the root name (traj) and appended name (Action)
// If you write a new client of the server in this package, you will need to include my_interesting_moves in your package.xml,
// and include the header file below
#include <my_interesting_moves/trajAction.h>
#include <iostream>
using namespace std;
#define VECTOR_DIM 7 // e.g., a 7-dof vector

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const my_interesting_moves::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
}

void sendCommand(trajectory_msgs::JointTrajectory &des_trajectory) {
    int g_count = 0;
     // here is a "goal" object compatible with the server, as defined in example_action_server/action
    my_interesting_moves::trajGoal goal; 
    // does this work?  copy traj to goal:
    goal.trajectory = des_trajectory;
    //cout<<"ready to connect to action server; enter 1: ";
    //cin>>ans;
    // use the name of our server, which is: trajActionServer (named in traj_interpolator_as.cpp)
    actionlib::SimpleActionClient<my_interesting_moves::trajAction> action_client("trajActionServer", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running


    if (!server_exists) {
        ROS_WARN("could not connect to server; will wait forever");
        return; // bail out; optionally, could print a warning message and retry
    }
    server_exists = action_client.waitForServer(); //wait forever 
    
   
    ROS_INFO("connected to action server");  // if here, then we connected to the server;

    //while(true) {
    // stuff a goal message:
    g_count++;
    goal.traj_id = g_count; // this merely sequentially numbers the goals sent
    ROS_INFO("sending traj_id %d",g_count);
    //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
    action_client.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
    //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
    
    bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
    //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on result for goal number %d",g_count);
        return;
    }
    else {
        ROS_INFO("finished before timeout");
    }   
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_action_client_node"); // name this node 
    ros::NodeHandle nh; //standard ros node handle        
    int ans;

    //cin>>ans;
    My_interesting_moves my_interesting_moves(&nh); //instantiate a my_interesting_moves object and pass in pointer to nodehandle for constructor to use  
    // warm up the joint-state callbacks;
    cout<<"warming up callbacks..."<<endl;
    for (int i=0;i<100;i++) {
        ros::spinOnce();
        //cout<<"spin "<<i<<endl;
        ros::Duration(0.01).sleep();
    }
        
    trajectory_msgs::JointTrajectory des_trajectory; // an empty trajectory 
    

    int i;
    while(1) {
        cout<<"Choose what you want Baxter do: 1: salute, 2: wave, 3: sayhi 0:quit"<<endl;
        cout<<"Only type the according number"<<endl;
        cin>>i;
        switch(i) {
            case 1:
                my_interesting_moves.set_goal_salute(des_trajectory);
                sendCommand(des_trajectory);
                break;
            case 2:
                my_interesting_moves.set_goal_wave(des_trajectory);
                sendCommand(des_trajectory);
                break;
            case 3:
                my_interesting_moves.set_goal_sayhi(des_trajectory);
                sendCommand(des_trajectory);
                break;
            case 0:
                return 0; 
            default:
                cout<<"wrong input!"<<endl;
                break;         
        }
        ros::Duration(0.5).sleep(); // sleep for half a second
    }
    return 0;
}

