/**

\author Michael Ferguson

@b Publish a transform between the checkerboard and the camera link. 

**/

#include <iostream>
#include <algorithm>
#include <limits>
#include <math.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

#include <pcl_ros/point_cloud.h>

#include "geometry.h"

using namespace std;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> CameraSyncPolicy;
typedef pcl::PointXYZ point;
typedef pcl::PointXYZRGB color_point;

class ChessBoardLocator
{
  public:
    ChessBoardLocator(ros::NodeHandle & n): nh_ (n),
        image_sub_ (nh_, "/camera/rgb/image_color", 3),
        cloud_sub_(nh_, "/camera/rgb/points", 3),
        sync_(CameraSyncPolicy(10), image_sub_, cloud_sub_),
        msgs_(0),   
        debug_(false)
    {
        ros::NodeHandle nh ("~");
        // load parameters for hough transform
        if (!nh.getParam ("h_rho", h_rho_))
            h_rho_ = 1;
        ROS_INFO ("Hough Rho: %d", h_rho_);  
        if (!nh.getParam ("h_threshold", h_threshold_))
            h_threshold_ = 50;
        ROS_INFO ("Hough Threshold: %d", h_threshold_); 
        if (!nh.getParam ("h_min_length", h_min_length_))
            h_min_length_ = 100;
        ROS_INFO ("Hough Min Length: %d", h_min_length_);    

        // create a window to display results in
        if (debug_) cv::namedWindow("chess_board_locator");
        pub_ = nh.advertise< pcl::PointCloud<point> >("points", 10);

        // create a cloud for ideal board
        for(int i = 1; i<8; i++)
        {
            for(int j = 1; j<8; j++)
            {
                chess_board.push_back(point( 0.05715*i, 0.05715*j, 0));
                //chess_board.push_back(point( 0.05715*i, 0.05715 + 0.01143*j, 0));
                //chess_board.push_back(point( 0.05715 + 0.01143*j, 0.05715*i, 0));
            }   
        }
        // prepare matcher
        matcher.setMaximumIterations(250);
        //matcher.setRANSACOutlierRejectionThreshold(0.025);
        matcher.setTransformationEpsilon(1e-9);
        matcher.setInputCloud(chess_board.makeShared());

        sync_.registerCallback(boost::bind(&ChessBoardLocator::cameraCallback, this, _1, _2));
    }

    /* 
     * Determine transform for chess board
     */
    void cameraCallback ( const sensor_msgs::ImageConstPtr& image,
                          const sensor_msgs::PointCloud2ConstPtr& depth)
    {
        // convert image
        try
        {
            bridge_ = cv_bridge::toCvCopy(image, "bgr8");
            ROS_INFO("New image/cloud.");
        }
        catch(cv_bridge::Exception& e)
        {
           ROS_ERROR("Conversion failed");
        }
        // convert cloud
        pcl::PointCloud<color_point> cloud;
        pcl::fromROSMsg(*depth, cloud);

        // start with edge detection
        cv::Mat /*src,*/ dst, cdst;
        //cv::cvtColor(bridge_->image, src, CV_BGR2GRAY);
        // alternatively, we can segment based on a particular channel
        cv::Mat src(bridge_->image.rows, bridge_->image.cols, CV_8UC1);
        for(int i = 0; i < bridge_->image.rows; i++)
        {
            char* Di = bridge_->image.ptr<char>(i);
            char* Ii = src.ptr<char>(i);
            for(int j = 0; j < bridge_->image.cols; j++)
            {   
                Ii[j] = Di[j*3];
            }   
        }
 
        cv::threshold(src, src, 100, 255, cv::THRESH_BINARY);
        cv::erode(src, src, cv::Mat());
        cv::dilate(src, src, cv::Mat());
        // edge detection, dilation before hough transform 
        cv::Canny(src, dst, 30, 200, 3); 
        cv::dilate(dst, dst, cv::Mat());

        // do a hough transformation to find lines
        vector<cv::Vec4i> lines;
        cv::HoughLinesP(dst, lines, h_rho_, CV_PI/180, h_threshold_, h_min_length_, 10 );
        ROS_INFO("Found %d lines", (int) lines.size());

        // split into vertical/horizontal lines
        vector<int> h_indexes, v_indexes;
        for( size_t i = 0; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
            int dx = l[2]-l[0]; int dy = l[3]-l[1];
            if(abs(dx) > abs(dy)){
                h_indexes.push_back(i);
            }else{
                v_indexes.push_back(i);
            }
        }

        // output lines to screen
        if(debug_)
        {
            // convert back to color
            cv::cvtColor(dst, cdst, CV_GRAY2BGR);
            
            // then red/green for horizontal/vertical
            ROS_INFO("horizontal lines: %d", (int) h_indexes.size());
            for( size_t i = 0; i < h_indexes.size(); i++ )
            {
                cv::Vec4i l = lines[h_indexes[i]];
                cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
            }
            ROS_INFO("vertical lines: %d", (int) v_indexes.size());
            for( size_t i = 0; i < v_indexes.size(); i++ )
            {
                cv::Vec4i l = lines[v_indexes[i]];
                cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 3, CV_AA);
            }
        }

        // get all endpoints
        /*pcl::PointCloud<point> endpoints;
        for( size_t i = 0; i < lines.size(); i++ ){
            cv::Vec4i l = lines[i];
            color_point s = cloud(l[0],l[1]);
            color_point e = cloud(l[2],l[3]);
            endpoints.push_back(point(s.x,s.y,s.z));
            endpoints.push_back(point(e.x,e.y,e.z));
        }*/

        // get all intersections;
        pcl::PointCloud<point> endpoints;
        endpoints.header.frame_id  = depth->header.frame_id;
        endpoints.header.stamp  = depth->header.stamp;
        for( size_t i = 0; i < h_indexes.size(); i++ )
        {
            cv::Vec4i hl = lines[h_indexes[i]];
            for( size_t j = 0; j < v_indexes.size(); j++ )
            {
                cv::Vec4i vl = lines[v_indexes[j]];
                cv::Point p = findIntersection(hl,vl);
                if(p.x > 0 && p.y > 0){
                    color_point cp = cloud(p.x,p.y);
                    endpoints.push_back( point(cp.x,cp.y,cp.z) );
                    if(debug_)
                    {
                        cv::circle( cdst, p, 5, cv::Scalar(255,0,0), -1 );
                    }
                }
            }
        }
        pub_.publish(endpoints);
        //chess_board.header.frame_id = depth->header.frame_id;
        //chess_board.header.stamp = depth->header.stamp;     
        //pub_.publish(chess_board);

        // find best fit transformation
        Eigen::Matrix4f trans;
        pcl::PointCloud<point> output;
        matcher.setInputTarget(endpoints.makeShared());
        matcher.align(output);
        cout << matcher.getFinalTransformation() << endl;
        cout << matcher.getFitnessScore() << endl;

        // publish transform
        tf::Transform transform = tfFromEigen(matcher.getFinalTransformation());
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), depth->header.frame_id, "chess_board"));
        ROS_INFO("published %d", msgs_++);

        // show the image
        if(debug_){
            cv::imshow("chess_board_locator", cdst);
            cv::imwrite("image.png", cdst);
            cv::waitKey(3);
        }
    }

  private: 
    /* node handles, subscribers, publishers, etc */
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_; 
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Synchronizer<CameraSyncPolicy> sync_;
    ros::Publisher pub_;
    tf::TransformBroadcaster br_;
    cv_bridge::CvImagePtr bridge_;
    
    /* cloud and matcher for computing transformation */
    pcl::PointCloud<point> chess_board;
    pcl::IterativeClosestPoint<point,point> matcher;

    /* parameters for hough line detection */
    int h_rho_;
    int h_threshold_;
    int h_min_length_;
    int msgs_;
    bool debug_;
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "chess_board_locator");
  ros::NodeHandle n;
  ChessBoardLocator locator(n);
  ros::spin();
  return 0;
}

