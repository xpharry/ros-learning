#include <iostream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <peng_nodes_action/monitorAction.h>

using namespace std;

peng_nodes_action::monitorResult g_result;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const peng_nodes_action::monitorResultConstPtr& result) {
    //ROS_INFO("doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("Succefully got result amplitude = %f; frequency = %f; ncycles = %ld", result->amplitude, 
        result->frequency, result->ncycles);
    g_result = *result;
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "monitor_action_client_node"); // name this node 

        double desired_amplitude;
        double desired_frequency;
        int desired_ncycles;

        std::cout<<"Enter your desired amplitude, frequency and number of cycles:"<<std::endl;
        std::cin>>desired_amplitude>>desired_frequency>>desired_ncycles;
        ROS_INFO("%f %f %d", desired_amplitude, desired_frequency, desired_ncycles);

        // here is a "goal" object compatible with the server, as defined in peng_nodes_action/action
        peng_nodes_action::monitorGoal goal; 
        
        // use the name of our server, which is: example_action (named in peng_nodes_action.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<peng_nodes_action::monitorAction> ac("monitor_action", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server: ");
        bool server_exists = ac.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        goal.amplitude = desired_amplitude;
        goal.frequency = desired_frequency;
        goal.ncycles = desired_ncycles;

        ROS_INFO("In client, goal.amplitude is: %f", goal.amplitude);
        ROS_INFO("In client, goal.frequency is: %f", goal.frequency);
        ROS_INFO("In client, goal.ncycles is: %ld", goal.ncycles);

        while(true) {
            if (g_result.ncycles >= goal.ncycles)
            {
                ROS_INFO("goal succeeded!");
                goal.amplitude = 0;
                goal.frequency = 0;
                goal.ncycles = 0;
            }
            //action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
            ac.sendGoal(goal,&doneCb); // we could also name additional callback functions here, if desired
            //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this
            
            //bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));
            bool finished_before_timeout = ac.waitForResult(); // wait forever...
            if (!finished_before_timeout) {
                ROS_WARN("giving up waiting on result for timeout");
                return 0;
            }
            else {
                //if here, then server returned a result to us
                ROS_INFO("finished before timeout");
            }
        }
        
    return 0;
}

