// illustrative program showing use of Eigen::Affine3d for transforms
// run this program with minimal robot:  
// roslaunch minimal_robot_description minimal_robot.launch
#include<ros/ros.h>
#include <tf/transform_listener.h> // for transform listener
#include <fstream> // useful I/O functions
#include <iostream>
#include <sstream>
#include <string>  //strings and vectors are often useful
#include <vector>

#include <Eigen/Eigen> // these to use some of the features of Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

//here are a couple of utility fncs useful for converting transform data types
Eigen::Affine3d transformTFToEigen(const tf::Transform &t) {
    Eigen::Affine3d e;
    // treat the Eigen::Affine as a 4x4 matrix:
    for (int i = 0; i < 3; i++) {
        e.matrix()(i, 3) = t.getOrigin()[i]; //copy the origin from tf to Eigen
        for (int j = 0; j < 3; j++) {
            e.matrix()(i, j) = t.getBasis()[i][j]; //and copy 3x3 rotation matrix
        }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
        e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
}

//if you have a geometry_msgs/Pose, can convert to equiv Affine this way
// also, can populate parts of affine with this alternative approach
Eigen::Affine3d transformPoseToEigenAffine3d(geometry_msgs::Pose pose) {
  Eigen::Affine3d affine;
    Eigen::Vector3d Oe;

    Oe(0)= pose.position.x;
    Oe(1)= pose.position.y;
    Oe(2)= pose.position.z;
    affine.translation() = Oe; // the "translation" part of affine is the vector between origins

    Eigen::Quaterniond q;
    q.x() = pose.orientation.x;
    q.y() = pose.orientation.y;
    q.z() = pose.orientation.z;
    q.w() = pose.orientation.w;
    Eigen::Matrix3d Re(q); //convenient conversion...initialize a 3x3 orientation matrix
                           // using a quaternion, q

    affine.linear() = Re; // the rotational part of affine is the "linear" part

 return affine;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_eigen"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    tf::StampedTransform tfTransform; //need objects of this type to hold tf's
    tf::TransformListener tfListener; //create a TransformListener to listen for tf's and assemble them

    // keep trying tf listener until successful...
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and world...");
    while (tferr) {
        tferr = false;
        try {
            //The direction of the transform returned will be from the target_frame to the source_frame. 
            //Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tfListener.lookupTransform("world", "kinect_pc_frame", ros::Time(0), tfTransform);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //tf-listener found a complete chain from sensor to world; ready to roll

    // let's collect individual transforms:
    tfListener.lookupTransform("world", "kinect_pc_frame", ros::Time(0), tfTransform);
    // instantiate a bunch of affine's to hold sequential transforms
    Eigen::Affine3d affine_kinect_pc_wrt_kinect_link;
    Eigen::Affine3d affine_kinect_link_wrt_link2;
    Eigen::Affine3d affine_link2_wrt_link1;
    Eigen::Affine3d affine_link1_wrt_world;
    Eigen::Affine3d affine_kinect_pc_wrt_world;
    Eigen::Affine3d affine_kinect_pc_wrt_world_from_tf;    

    // we'll get the individual tf's from the transform listener
    // this one has already been set--complete transform from sensor frame to world
    affine_kinect_pc_wrt_world_from_tf= transformTFToEigen(tfTransform);
    
    // lookup the transform from sensor frame to sensor body:
    tfListener.lookupTransform("kinect_link", "kinect_pc_frame", ros::Time(0), tfTransform);
    affine_kinect_pc_wrt_kinect_link= transformTFToEigen(tfTransform); 
    
    // next incremental transform
    tfListener.lookupTransform("link2", "kinect_link", ros::Time(0), tfTransform);
    affine_kinect_link_wrt_link2= transformTFToEigen(tfTransform);
    
    tfListener.lookupTransform("link1", "link2", ros::Time(0), tfTransform);
    affine_link2_wrt_link1= transformTFToEigen(tfTransform);
    
    tfListener.lookupTransform("world", "link1", ros::Time(0), tfTransform);
    affine_link1_wrt_world= transformTFToEigen(tfTransform);

    //now, simply multiply all of these together to get complete transform
    // from sensor frame to world:
    affine_kinect_pc_wrt_world= 
            affine_link1_wrt_world*affine_link2_wrt_link1*affine_kinect_link_wrt_link2*affine_kinect_pc_wrt_kinect_link;
  
    // here's what we get from transform listener in single request from sensor frame to world
    ROS_INFO("affine of kinect_pc_frame w/rt world, per tf: ");
    ROS_INFO("orientation: ");
    cout<<affine_kinect_pc_wrt_world_from_tf.linear()<<endl;
    cout<<"origin: "<<affine_kinect_pc_wrt_world_from_tf.translation().transpose()<<endl;

    //and here is the result using multiplies of separate transforms:
    ROS_INFO("affine of kinect_pc_frame w/rt world, per affine multiplies: ");
    ROS_INFO("orientation: ");
    cout<<affine_kinect_pc_wrt_world.linear()<<endl;
    cout<<"origin: "<<affine_kinect_pc_wrt_world.translation().transpose()<<endl;
    
    //let's say we have a point, "p", as detected in the sensor frame;
    // arbitrarily, initialize this to [1;2;3]
    Eigen::Vector3d p_wrt_sensor(1,2,3);
    cout<<"point w/rt sensor: "<<p_wrt_sensor.transpose()<<endl;
    // here's how to convert this point to the world frame:
    Eigen::Vector3d p_wrt_world;
    p_wrt_world = affine_kinect_pc_wrt_world*p_wrt_sensor;
    cout<<"point w/rt world: "<<p_wrt_world.transpose()<<endl;
    
    //we can transform in the opposite direction with the transform inverse:
    Eigen::Vector3d p_back_in_sensor_frame;
    p_back_in_sensor_frame = affine_kinect_pc_wrt_world.inverse()*p_wrt_world;
    cout<<"p back in sensor frame: "<<p_back_in_sensor_frame.transpose()<<endl;
    
    //maybe do something useful here...e.g., keep computing transforms while robot moves
    //ros::Rate sleep_timer(1.0); //a timer for desired rate, e.g. 1Hz
    //while (ros::ok()) {
    //    sleep_timer.sleep();
    //}
}

