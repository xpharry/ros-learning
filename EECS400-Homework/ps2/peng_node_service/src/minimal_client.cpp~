#include "ros/ros.h"
#include "peng_node_service/GetVelocity.h"
#include <cstdlib>
#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "minimal_client");
/*  if (argc != 3)
  {
    ROS_INFO("usage: minimal_client amplitude frequency");
    return 1;
  }
*/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<peng_node_service::GetVelocity>("get_velocity");
  peng_node_service::GetVelocity srv;
  ROS_INFO("COME");

  //srv.request.amplitude = atof(argv[1]);
  //srv.request.frequency = atof(argv[2]);
  //srv.request.amplitude = 1;
  //srv.request.frequency = 5;
  ROS_INFO("COME again");
std::cout<<"Enter values of the amplitude and frequency:"<<std::endl;
    std::cin>>srv.request.amplitude;
    std::cin>>srv.request.frequency;
  //bool success = client.call(srv.request, srv.response);
  //ROS_INFO("is success ture or false (0 or 1)= %d", success);
  if (client.call(srv))
  {
    
    ROS_INFO("The amplitude got back from response: %f", srv.response.amplitude);
    ROS_INFO("The frequency got back from response: %f", srv.response.frequency);
  }
  else
  {
    ROS_ERROR("Failed to call service minimal_commander");
    return 1;
  }

  return 0;
}
