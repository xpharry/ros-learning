#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <sstream>
#include <peng_node_service/GetVelocity.h>

double PI = 3.1415926;
int i = 0;
double vel;

double amp;
double fre;

bool get_vel(peng_node_service::GetVelocityRequest  &req,
         peng_node_service::GetVelocityResponse &res)
{
  ROS_INFO("callback activated");
  //res.velocity = req.amplitude*sin(req.frequency*PI/180*i);
  ROS_INFO("request: amplitude=%f, frequency=%f", req.amplitude, req.frequency);
  res.amplitude = req.amplitude;
  res.frequency = req.frequency;

  amp = req.amplitude;
  fre = req.frequency;
  return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "minimal_commander"); // name of this node will be "minimal_"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher pub = n.advertise<std_msgs::Float64>("vel_cmd", 1000); 
    //"vel_cmd" is the name of the topic to which we will publish
    // the "1000" argument says to use a buffer size of 1000; could make larger, if expect network backups
   	amp = 0.0;
	fre = 0.0;
    std_msgs::Float64 velocity; //create a variable of type "Float64",
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission

    ros::ServiceServer service = n.advertiseService("get_velocity", get_vel);
   
   ros::Rate naptime(10.0); //create a ros object from the ros “Rate” class; 
   //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

    peng_node_service::GetVelocityResponse res;
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
        velocity.data = amp*sin(fre*PI/180*i); //phase increment by frequency*PI/180 radius each iteration
        i++;
        ROS_INFO("Sending data %f", velocity.data);
        pub.publish(velocity); // publish the value--of type Float64-- 
        //to the topic "topic1"
	//the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency
	ros::spinOnce(); 
	naptime.sleep(); 
    }
}

