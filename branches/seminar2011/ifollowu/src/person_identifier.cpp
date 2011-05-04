/**
 * Co-Authors: Stu Odell and Ramon Rahman, with oversight by Mike Ferguson.
 * Person Recognition Utility.
 * Will take a depth image and analyze it by segmenting it and then analyzing
 * the segment to see which segment most resembles a human being.
 */

#include <stdio.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <math.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Point.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <limits>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> CameraSyncPolicy;

static const char WINDOW[] = "Image Window";
using namespace std;

ros::Publisher person_pub;
int skip;

//Struct needed to start the segment being and end information in the vector.
struct PointInfo
{
  int row;
  int col;
  float distance;
  PointInfo() {}
  PointInfo( int y, int x, float d )
  {
    row = y;
    col = x;
    distance = d;
  }
};


/**
 * This function takes a depth image, walks across the center row and breaks
 * the image into distinct segments using a jump distance metric. 
 * 
 * Returns a vector that stores endpoints of each segment
 *  segment 0: start index = 0, end index = 1
 *  segment 1: start index = 2, end index = 3 
 *  ...
 *
 * Author: Ramon Rahman
 * Date Created: April 3rd, 2011
 */
void doSegmentation(cv_bridge::CvImagePtr depth, vector<PointInfo> &segments)
{
    // get the halfway point in the rows up the picture. So should be chest or higher 
    // but we can adjust this as we go.
    int row = depth->image.rows/2;
    float previousPointDistance = depth->image.at<float>( row, 1 );

    //we are storing the initial vector info.
    PointInfo initalPointInfo;
    initalPointInfo.row = row;
    initalPointInfo.col = 1;
    initalPointInfo.distance = previousPointDistance;
    segments.push_back(initalPointInfo);

    for ( int col = 2; col < depth->image.cols; col++ )
	{
	    float currentPointDistance = depth->image.at<float>(row,col);

        if(!std::isnan(currentPointDistance) && !std::isinf(currentPointDistance))
		{
		    if(fabs(previousPointDistance - currentPointDistance) > .04)
			{
                // TODO: deal with nan's better (they are probably jump conditions) 
				PointInfo previousPointInfo( row, col -1, previousPointDistance );
				//store the last point in the next slot, so we can
				//say where the last object similar distance away ends.
				segments.push_back(previousPointInfo);

				PointInfo currentPointInfo( row, col, currentPointDistance );
				//store the current beginning of a new segment
				segments.push_back(currentPointInfo);
			}
			previousPointDistance = currentPointDistance;
		}
    }

    //we are storing the final vector info, where the last segment is stored.
    PointInfo finalPointInfo( row, depth->image.cols-2, depth->image.at<float>( row, depth->image.cols - 2 ) );
    //finalPointInfo.row = row;
    //finalPointInfo.col = depth->image.cols-2;
    //finalPointInfo.distance = depth->image.at<float>(row,depth->image.cols-1);
    segments.push_back(finalPointInfo);
}


/**
 * Will analyze the rgb image info between where the segment we are analyzing currently starts
 * and ends. Will return true if it is a person by examing the arch and the histogram method.
 * Need to be confined only in this segment, and analyze only this segment to find a human
 * or something else.
 *
 * TODO: For STU.
 * Walk up the segment, and then do a search accross once at the top. Ask Mike about this,
 * he can explain it to you where you have to start 5 rows down once at the top, don't have
 * to start at the bottom for all the columns accross.
 *
 */
bool heightCheck(pcl::PointCloud<pcl::PointXYZ> &cloud, PointInfo segmentStart, PointInfo segmentEnd)
{
  float width = fabs( cloud( segmentEnd.col, segmentEnd.row ).x - cloud( segmentStart.col, segmentStart.row ).x );
  if ( width > .3 && width < .7 )
  {
    float highestPoint = 0; // will be negative becouse y is inverted
    float depth_buffer = .03; // about 3 centimeters
    pcl::PointXYZ prev_point, next_point;
    for ( int row = segmentStart.row, col = segmentStart.col; col <= segmentEnd.col; ++col ) // itorate through every column
    {
      row = segmentStart.row; // set row to the row of the segment
      while( fabs( cloud( col, row - 1 ).z - cloud( col, row ).z ) < depth_buffer && row > 1 )
        --row;
      if ( highestPoint > cloud( col, row ).y )
        highestPoint = cloud( col, row ).y;
      //ROS_INFO( "y = %.2f", cloud( col, row ).y );
    }
    //ROS_INFO( "highestPoint = %d" , highestPoint);
    return ( highestPoint > -.8 && highestPoint < -.3 );
  }
  else
    return false;
}


/**
 * This is to find the 2 closest segments. This is for the implementation that assumes
 * that the person will be closest thing.
 * 
 * Author: Ramon Rahman
 */
PointInfo * findClosestSegment(vector<PointInfo> segments)
{ 
  PointInfo closestPoints[2];
  float closestDistanceSoFar = 9999.99;
  for(unsigned int i = 0; i < segments.size(); i += 2)
  {
     //printf("i: %d\n", i);
     PointInfo segmentStart = segments[i];
     PointInfo segmentEnd = segments[i+1];
     if(std::isnan(segmentStart.distance))
	continue;
  ROS_ERROR("In closest point function segmentLeft %f, segmentRight %f", segmentStart.distance, segmentEnd.distance);	
     if(segmentStart.distance > 0 && segmentStart.distance < closestDistanceSoFar)
     {
	closestPoints[0] = segmentStart;
	closestPoints[1] = segmentEnd;
	closestDistanceSoFar = segmentStart.distance;
	ROS_ERROR("closestSoFar: %f", closestDistanceSoFar);
     } 
  ROS_ERROR("bluebalha segmentLeft %f, segmentRight %f", closestPoints[0].distance, closestPoints[1].distance);
  }

  return closestPoints;
}

/**
 * This is to find the 2 closest segments. This is for the implementation that assumes
 * that the person will be closest thing.
 * 
 * Author: Ramon Rahman
 */
bool findCentroidClosestSegment(const sensor_msgs::PointCloud2ConstPtr & points, vector<PointInfo> segments, pcl::PointXYZ & centroid)
{
  // convert cloud from sensor message
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*points, cloud);

  float dist = 9999.999;
  //ROS_INFO("%d", (int) cloud.size());

  for(unsigned int i = 0; i < segments.size(); i += 2)
  {
    PointInfo start = segments[i];
    PointInfo end = segments[i+1];

    if ( heightCheck( cloud, start, end ) )///////////////////////
    {
      ROS_INFO( "HUMAN DETECTED!!!!111!!1!!11!!1111111212321" );
      pcl::PointXYZ p =  pcl::PointXYZ( 0, 0, 0);
      int n = 0;

      //ROS_INFO("(%d, %d) to (%d, %d)", start.row, start.col, end.row, end.col);
      for( int col = start.col; col < end.col; col++ )
      {
          pcl::PointXYZ k = cloud(col, start.row);
          if ( !std::isnan(k.x) && !std::isnan(k.y) && !std::isnan(k.z))
          {
              p.x += k.x;
              p.y += k.y;
              p.z += k.z;
              n++;
          }
      }
  
      if( n )
      {
          p.x /= n;
          p.y /= n;
          p.z /= n;
          float d = pcl::squaredEuclideanDistance<pcl::PointXYZ,pcl::PointXYZ>( p, pcl::PointXYZ(0,0,0) );
          ROS_INFO("d = %f", d);
          if( d < dist )
          {
              dist = d;
              centroid.x = p.x;
              centroid.y = p.y;
              centroid.z = p.z;
          }
      }
    } // end if
  }
  return true;
}


/**
 * This callback processes a depth image to find people, using some additional
 * data from the associated point cloud.
 */
void depthCb( const sensor_msgs::ImageConstPtr& image,
              const sensor_msgs::PointCloud2ConstPtr& points )
{
  //if( (skip++%5) != 0 )
    //return;

  //ROS_INFO("Callback triggered on depth image");

  cv_bridge::CvImagePtr depth;
  try
  {
    depth = cv_bridge::toCvCopy( image, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Unable to convert depth image");
  }

  // get the segment information and see where they are stored.
  // NOTE: changed 4/30, segments is now passed as parameter (MEF)
  vector<PointInfo> segments;
  doSegmentation(depth, segments);
  
  ROS_INFO("Found %d segments", (int) segments.size()/2);
  // create visual representation of segments
  // added 4/30 (MEF)
  cv::Mat img(depth->image.rows, depth->image.cols, CV_8UC3);
  for(int i = 0; i < depth->image.rows; i++)
  {
      float* Di = depth->image.ptr<float>(i);
      char* Ii = img.ptr<char>(i);
      for(int j = 0; j < depth->image.cols; j++)
      {   
          Ii[j*3] = (char) (255*((Di[j]-0.5)/5.0));
          Ii[j*3+1] = (char) (255*((Di[j]-0.5)/5.0));
          Ii[j*3+2] = (char) (255*((Di[j]-0.5)/5.0));
      }   
  }
  /* walk along segment
  for ( size_t i = 0; i < segments.size(); i+=2 )
  {
    PointInfo segmentStart = segments[i];
    PointInfo segmentEnd = segments[i+1];  

    float depth_buffer = .2;
    for ( int col = segmentStart.col; col <= segmentEnd.col; ++col ) // iterate through every column in segment
    {
        int row = segmentStart.row;
        float prev = depth->image.at<float>( row, col );
        while( fabs(depth->image.at<float>( row - 1, col ) - prev) < depth_buffer && row >= 0){
          cv::circle(img, cv::Point(col, row), 2, cvScalar(255,0,0));
          prev = depth->image.at<float>( --row, col );
        }
        if ( row >= 0 )
        {
          cv::circle(img, cv::Point(col, row), 2, cvScalar(255,0,0));
        }
    }	
    cv::circle(img, cv::Point(segmentStart.col, segmentStart.row), 2, cvScalar(0,255,0));
    cv::circle(img, cv::Point(segmentEnd.col, segmentEnd.row), 2, cvScalar(0,255,0));
  }
  //cv::imwrite( "depthImagewithpoints.jpg", img);
  */
  // find closest centroid
  pcl::PointXYZ centroid = pcl::PointXYZ( 0, 0, 0.8 );
  findCentroidClosestSegment( points, segments, centroid );
    
  // publish output
  geometry_msgs::Point msg = geometry_msgs::Point();
  msg.x = centroid.x;
  msg.y = centroid.y;
  msg.z = centroid.z;
  person_pub.publish(msg);
}


int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "person_identifier" );
  ros::NodeHandle n;

  skip = 0;

  // subscribe to the /camera/depth/image and /camera/depth/points
  message_filters::Subscriber<sensor_msgs::Image> depth_sub( n, "camera/depth/image", 3);
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub( n, "camera/depth/points", 3);
  message_filters::Synchronizer<CameraSyncPolicy> sync(CameraSyncPolicy(10), depth_sub, points_sub);

  // publisher
  person_pub = n.advertise<geometry_msgs::Point>("person_location", 10);

  // made cv named window here
  if(0){
    cv::namedWindow(WINDOW);
  }

  sync.registerCallback( boost::bind(&depthCb, _1, _2 ) );

  ros::spin();
}
