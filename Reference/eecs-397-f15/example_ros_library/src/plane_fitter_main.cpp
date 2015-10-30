//plane_fitter_main.cpp
#include<ros/ros.h>
#include <Eigen/Eigen>
#include <example_ros_library/plane_fitter.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_eigen_plane_fit"); //node name
    ros::NodeHandle nh; // don't really need this in this example

    Eigen::Vector3d normal_vec(1,2,3); // here is an arbitrary normal vector
    cout<<"normal: "<<normal_vec.transpose()<<endl; //.transpose() is so I can display the components on the same line, instead of as a column
    normal_vec/=normal_vec.norm(); // make this vector unit length
    cout<<"unit length normal: "<<normal_vec.transpose()<<endl;
    double dist = 1.23;  // define the plane to have this distance from the origin.  Note that if we care about positive/negative sides of a plane,
                         // then this "distance" could be negative, as measured from the origin to the plane along the positive plane normal
    cout<<"plane distance from origin: "<<dist<<endl;

double noise_gain = 0.1;
double npts = 10;
Eigen::MatrixXd points_array;

  PlaneFitter planeFitter;
  planeFitter.generate_planar_points(normal_vec,dist,npts,noise_gain,points_array);
  cout<<"points: "<<endl;
  cout<<points_array<<endl;

  cout<<"try to identify plane: "<<endl;
  Eigen::Vector3d est_normal_vec;
  double est_dist;
  planeFitter.fit_points_to_plane(points_array,est_normal_vec, est_dist);
  cout<<"identified norma: "<<est_normal_vec.transpose()<<endl;
  cout<<"estimated distance: "<<est_dist<<endl;



}

