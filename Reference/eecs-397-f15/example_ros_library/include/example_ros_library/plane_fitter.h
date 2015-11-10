// plane_fitter.h header file //
/// wsn; Oct, 2015.  
/// Include this file in "plane_fitter.cpp", and in any main that uses this library.
///This class provides a function to fit a plane to given set of (presumably) nearly-coplanar points
///

#ifndef PLANE_FITTER_H_
#define PLANE_FITTER_H_

#include<ros/ros.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

// define a class, including a constructor, member variables and member functions
class PlaneFitter
{
public:
    PlaneFitter(); 

     /**provide an array of 3-D points (in columns), and this function will use and eigen-vector approach to find the best-fit plane
     * It returns the plane's normal vector and the plane's (signed) distance from the origin.
     * @param points_array input: points_array is a matrix of 3-D points to be plane-fitted; coordinates are in columns
     * @param plane_normal output: this function will compute components of the plane normal here
     * @param plane_dist output: scalar (signed) distance of the plane from the origin
     */
    void fit_points_to_plane(Eigen::MatrixXd points_array, 
        Eigen::Vector3d &plane_normal, 
        double &plane_dist); 
        /**
     * Function to generate a set of random points that are (nearly) co-planar.  Useful for testing fit_point_to_plane() fnc.
     * Provide plane params, number of points desired, and scale of noise to add.  Resulting points are returned in points_array
     * @param normal_vec  [in] input: specify plane normal
     * @param dist input: specify plane's (signed) distance from the origin
     * @param npts input: specify number of desired points
     * @param noise_gain input: specify magnitude of noise (multiplier times noise in range [-1,1])
     * @param points_array output: this array gets populated with the generated points
     */
    void generate_planar_points(Eigen::Vector3d normal_vec,double dist,int npts, double noise_gain,Eigen::MatrixXd &points_array);

private:
    // put private member data here;  "private" data will only be available to member functions of this class;

}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
