#ifndef MY_PCL_UTILS_H_
#define MY_PCL_UTILS_H_

// define a class, including a constructor, member variables and member functions
class MyPclUtils : public CwruPclUtils
{
public:
    /* the following are added by Peng Xu */
    void extract_coplanar_pcl_operation();
    double distance_between(Eigen::Vector3f pt1, Eigen::Vector3f pt2);
    Eigen::Vector3f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr points_mat);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  
