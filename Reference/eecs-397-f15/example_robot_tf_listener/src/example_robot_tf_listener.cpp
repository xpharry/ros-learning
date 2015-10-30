//example_tf_listener.cpp:
//wsn, March 2015
//illustrative node to show use of tf listener
// this example written to be used with minimal_robot, one_DOF_robot
// start up the one-DOF robot with:  roslaunch minimal_robot_description minimal_robot.launch 

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
// tf-specific headers:
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <tf/LinearMath/Matrix3x3.h>






int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "demoTfListener"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    tf::StampedTransform tf_sensor_frame_to_world_frame; //need objects of this type to hold tf's
    tf::TransformListener tfListener; //create a TransformListener to listen for tf's and assemble them

    //the tf listener needs to "warm up", in that it needs to collect a complete set of transforms
    // to deduce arbitrary connectivities; use try/catch to let it fail until success
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and world...");
    while (tferr) {
        tferr = false;
        try {

            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("world", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_world_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll

    ros::Rate sleep_timer(1.0); //a timer for desired rate, e.g. 1Hz
   
    ROS_INFO:("starting main loop");
    tf::Vector3 origin,row;
    tf::Matrix3x3 R_orientation;



    while (ros::ok()) {
	tfListener.lookupTransform("world", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_world_frame);
	origin= tf_sensor_frame_to_world_frame.getOrigin();
    	ROS_INFO("origin: %f, %f, %f",origin[0],origin[1],origin[2]);
	R_orientation = tf_sensor_frame_to_world_frame.getBasis();
	ROS_INFO("orientation matrix: ");
	row = R_orientation[0];
	ROS_INFO(" %f    %f   %f  ",row[0],row[1],row[2]);
	row = R_orientation[1];
	ROS_INFO(" %f    %f   %f  ",row[0],row[1],row[2]);
	row = R_orientation[02];
	ROS_INFO(" %f    %f   %f  ",row[0],row[1],row[2]);
        ROS_INFO(" ");
        ROS_INFO(" ");

        //ros::spinOnce(); // tf_listener is its own thread--it does not require a spins from main
        sleep_timer.sleep();
    }
    return 0;
} 

