/**

\author Michael Ferguson

@b geometry helpers for chess board localization.

**/

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
    btMatrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
               trans(1,0),trans(1,1),trans(1,2),
               trans(2,0),trans(2,1),trans(2,2));
    btTransform ret;
    ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
}

cv::Point findIntersection( cv::Vec4i a, cv::Vec4i b )
{
    double ma = (a[3]-a[1])/(double)(a[2]-a[0]);
    double mb = (b[3]-b[1])/(double)(b[2]-b[0]);
    double ba = a[1] - ma*a[0];
    double bb = b[1] - mb*b[0];
    
    double x = (bb-ba)/(ma-mb);
    double y = ma*x + ba;

    if( (x>=0) && (x<640) && (y>=0) && (y<480) ){
        return cv::Point((int)x,(int)y);
    }else{
        return cv::Point(-1,-1);
    }
}
