// cwru_pcl_utils: a  ROS library to illustrate use of PCL, including some handy utility functions
//

#include <cloud_neighbor_finder/my_pcl_utils.h>
//uses initializer list for member vars

/**
    These function is what I wrote to extract the plane that colanar with the selected patch.
    Wrote by Peng Xu.
*/
void CwruPclUtils::extract_coplanar_pcl_operation() {
    int npts = pclTransformed_ptr_->points.size(); //number of points in kinect point cloud
    //pclGenPurposeCloud_ptr_->points.resize(npts);
    Eigen::Vector3f centroid = compute_centroid(pclTransformedSelectedPoints_ptr_);
    cout<< "coplanar ... " << endl;

    for (int i = 0; i < npts; ++i) {
        if( distance_between( centroid, pclTransformed_ptr_->points[i].getVector3fMap() ) < 0.8
                && ( centroid[2] - pclTransformed_ptr_->points[i].getVector3fMap()[2] ) < 0.000001
                && ( centroid[2] - pclTransformed_ptr_->points[i].getVector3fMap()[2] ) > -0.000001 ) {

            cout << "height of centroid = " << centroid[2] << endl;
            cout << "the height of point " << i << "= " << pclTransformed_ptr_->points[i].getVector3fMap()[2] << endl;
            pclGenPurposeCloud_ptr_->points.push_back(pclTransformed_ptr_->points[i]);
        }  
    }

    pclGenPurposeCloud_ptr_->header = pclTransformedSelectedPoints_ptr_->header;
    pclGenPurposeCloud_ptr_->is_dense = pclTransformedSelectedPoints_ptr_->is_dense;
    pclGenPurposeCloud_ptr_->width = npts;
    pclGenPurposeCloud_ptr_->height = 1; 

} 

double CwruPclUtils::distance_between(Eigen::Vector3f pt1, Eigen::Vector3f pt2) {
    Eigen::Vector3f pt = pt1 - pt2;
    double distance = pt(0)*pt(0) + pt(1)*pt(1) + pt(2)*pt(2);
    return distance;
}

Eigen::Vector3f CwruPclUtils::compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    Eigen::Vector3f centroid;
    // here's a handy way to initialize data to all zeros; more variants exist
    centroid = Eigen::MatrixXf::Zero(3, 1); // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
    //add all the points together:
    int npts = cloud->points.size(); // number of points = number of columns in matrix; check the size
    //cout<<"matrix has ncols = "<<npts<<endl;
    for (int ipt = 0; ipt < npts; ipt++) {
        centroid += cloud->points[ipt].getVector3fMap(); //add all the column vectors together
    }
    centroid /= npts; //divide by the number of points to get the centroid

    return centroid;
}