/**

\author Michael Ferguson

@b Publish a transform between the checkerboard and the camera link. 

**/

#include <iostream>
#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/cv_bridge.h"
#include "tf/transform_broadcaster.h"

#include "pcl/io/io.h"

using namespace std;

class ChessBoardLocator
{
  public:
    ChessBoardLocator(ros::NodeHandle & n):n_ (n)
    {
        // create a window to display results in
        cv::namedWindow("chess_board_locator");

        // tf transforms
        

        // register callbacks        
        image_sub_ = n.subscribe("/camera/rgb/image_color", 1, &ChessBoardLocator::image_cb, this);
        depth_sub_ = n.subscribe("/camera/depth/points2", 1, &ChessBoardLocator::depth_cb, this);
    }

    /* 
     * Store the latest point cloud
     */
    void depth_cb ( const sensor_msgs::PointCloud2ConstPtr& cloud )
    {
        depth_ = cloud;
    }

    /* 
     * Determine transform
     */
    void image_cb ( const sensor_msgs::Image& image )
    {
        //rgb_ = image;

        try
        {
            // bgr8 is the pixel encoding -- 8 bits per color, organized as blue/green/red
            bridge = cv_bridge::toCvCopy(image, "bgr8"); //"8UC1"); //"bgr8");
        }
        catch(cv_bridge::Exception& e)
        {
            // all print statements should use a ROS_ form, don't cout!
            ROS_ERROR("Conversion failed");
        }

        cv::Mat src, dst, cdst;
        //cv::medianBlur(bridge->image, bridge->image, 5);
        cv::cvtColor(bridge->image, src, CV_BGR2GRAY);
        cv::Canny(src, dst, 40, 200,3); 
        cv::cvtColor(dst, cdst, CV_GRAY2BGR);

        vector<cv::Vec4i> lines;
        cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
        for( size_t i = 0; i < lines.size(); i++ )
        {
            cv::Vec4i l = lines[i];
            cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
        }


        // show the image
        cv::imshow("chess_board_locator", cdst); //bridge->image);
        cv::waitKey(3);

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0) );
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "openni_camera", "chess_board"));
        ROS_INFO("published");
    }

  private: 
    ros::Subscriber             image_sub_; 
    sensor_msgs::PointCloud2ConstPtr    depth_;
    ros::Subscriber             depth_sub_; 
    ros::NodeHandle             n_;
    tf::TransformBroadcaster    br;
    cv_bridge::CvImagePtr bridge;
};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "chess_board_locator");
  ros::NodeHandle n;
  ChessBoardLocator locator(n);
  ros::spin ();
  return 0;
}
