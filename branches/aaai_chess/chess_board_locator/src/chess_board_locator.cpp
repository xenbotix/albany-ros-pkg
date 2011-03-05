/**

\author Michael Ferguson

@b Publish a transform between the checkerboard and the camera link. 

**/

#include <iostream>
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

using namespace std;

//The policy merges kinect messages with approximately equal timestamp into one callback 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> CameraSyncPolicy;

class ChessBoardLocator
{
  public:
    ChessBoardLocator(ros::NodeHandle & n): nh_ (n),
        image_sub_ (nh_, "/camera/rgb/image_color", 3),
        info_sub_(nh_, "/camera/rgb/camera_info", 3),
        cloud_sub_(nh_, "/camera/depth/points2", 3),
        sync_(CameraSyncPolicy(10), image_sub_, info_sub_, cloud_sub_)
    {
        // temporary code: create a window to display results in
        cv::namedWindow("chess_board_locator");

        // ApproximateTime takes a queue size as its constructor argument, hence CameraSyncPolicy(10)
        sync_.registerCallback(boost::bind(&ChessBoardLocator::cameraCallback, this, _1, _2, _3));
    }

    /* 
     * Determine transform for chess board
     */
    void cameraCallback ( const sensor_msgs::ImageConstPtr& image,
                          const sensor_msgs::CameraInfoConstPtr& info,
                          const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        try
        {
            bridge_ = cv_bridge::toCvCopy(image, "bgr8");
        }
        catch(cv_bridge::Exception& e)
        {
           ROS_ERROR("Conversion failed");
        }

        cv::Mat src, dst, cdst;
        cv::cvtColor(bridge_->image, src, CV_BGR2GRAY);
        cv::Canny(src, dst, 40, 200,3); 
        cv::cvtColor(dst, cdst, CV_GRAY2BGR);

        vector<cv::Vec4i> lines;
        cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
        for( size_t i = 0; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
            cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
        }

        // temporary code: show the image
        cv::imshow("chess_board_locator", cdst);
        cv::waitKey(3);

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0) );
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "openni_camera", "chess_board"));

        cv::imwrite("image.png", cdst);

        ROS_INFO("published");
    }

  private: 
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_; 
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Synchronizer<CameraSyncPolicy> sync_;
    tf::TransformBroadcaster    br_;
    cv_bridge::CvImagePtr bridge_;
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "chess_board_locator");
  ros::NodeHandle n;
  ChessBoardLocator locator(n);
  ros::spin ();
  return 0;
}
