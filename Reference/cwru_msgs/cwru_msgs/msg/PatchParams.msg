#patch params: parameters describing a planar patch
#a geometric planar fit to the patch is described in terms of a normal vector and 
#distance of the plane from the origin of the reference frame
#also provide centroid of the plane
float64[] centroid # 3-D x,y,z centroid of the patch
float64[] normal_vec # nx, ny, nz normal vector of the planar patch
float64 offset # distance of the geometric plane from the reference-frame origin
string frame_id #desirably, specify the reference frame for the data contained herein
