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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <limits>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> CameraSyncPolicy;

static const char WINDOW[] = "Image Window";
using namespace std;

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
		    if(fabs(previousPointDistance - currentPointDistance) > .15)
			{
                // TODO: deal with nan's better (they are probably jump conditions) 
				PointInfo previousPointInfo;
				previousPointInfo.row = row;
				previousPointInfo.col = col -1;
				previousPointInfo.distance = previousPointDistance;
				//store the last point in the next slot, so we can
				//say where the last object similar distance away ends.
				segments.push_back(previousPointInfo);
	
				PointInfo currentPointInfo;
				currentPointInfo.row = row;
				currentPointInfo.col = col;
				currentPointInfo.distance = currentPointDistance;
				//store the current beginning of a new segment
				segments.push_back(currentPointInfo);
			}
			previousPointDistance = currentPointDistance;
		}
    }

    //we are storing the final vector info, where the last segment is stored.
    PointInfo finalPointInfo;
    finalPointInfo.row = row;
    finalPointInfo.col = depth->image.cols;
    finalPointInfo.distance = depth->image.at<float>(row,depth->image.cols);
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
bool isHuman(cv_bridge::CvImagePtr depthImage, PointInfo segmentStart, PointInfo segmentEnd)
{
  vector<PointInfo> points;
  float depth_buffer = .03; // about 3 centimeters
  float prev;
  // REMEMBER!!! cvimages are in stupid row-major order because it sucks
  ROS_INFO( "checkpoint alpha" );
  for ( int row, col = segmentStart.col; col <= segmentEnd.col; ++col ) // itorate through every column in segment
  {
    row = segmentStart.row; // set row to the row of the segment
    prev = depthImage->image.at<float>( row, col ); // use the first pixel in column for depth reference
    while( depthImage->image.at<float>( row - 1, col ) - prev < depth_buffer && row >= 0)
      --row;
    if ( row >= 0 )
    {
      points.push_back( PointInfo( row, col, 0 ) );
      ROS_INFO( "point found at %d, %d", points.back().col, points.back().row );
    }
  }	
  ROS_INFO( "%d side points found!", (int) points.size() );
 
  cv::Mat img(depthImage->image.rows, depthImage->image.cols, CV_8UC1);
  ROS_INFO( "checkpoint beta" );
  for ( int i = 0; i < points.size(); i++ )
    img.at<float>( points.at( i ).row, points.at( i ).col ) = 200;
  //img.at<float>( segmentStart.row, segmentStart.col ) = 225;
  //img.at<float>( segmentEnd.row, segmentEnd.col ) = 225;
  // draws a white bar across the center for testing purposes
  //for ( int i = 0; i < img.cols; i++ )
    //img.at<float>( 100, i ) = 225;

  /*for(size_t j = 0; j< points.size(); j++){
    PointInfo i = points[j];
    img.at<char>( i.col, i.row ) = numeric_limits<float>::max();
  }*/
  cv::imwrite( "depthImagewithpoints.jpg", img); //depthImage->image );
  ROS_INFO( "checkpoint charlie" );
  //string name = sprintf( "depthImage%d.jpg", segmentStart.col);
  //cv::imwrite( name, img);
  //cv::waitKey(3);
  //ROS_INFO( "done" );
  return true;
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
 * This callback processes a depth image to find people, using some additional
 * data from the associated point cloud.
 */
void depthCb( const sensor_msgs::ImageConstPtr& image,
              const sensor_msgs::PointCloud2ConstPtr& points )
{
  ROS_INFO("Callback triggered on depth image");

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
  // walk along segment
  for ( int i = 0; i < segments.size(); i+=2 )
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
    cv::circle(img, cv::Point(segmentEnd.col, segmentEnd.row), 2, cvScalar(0,0,255));
  }
  cv::imwrite( "depthImagewithpoints.jpg", img); 
  return;





  int count = 1;
  for(unsigned int i = 0; i < segments.size(); i += 2)
  {
     //printf("i: %d\n", i);
     PointInfo start = segments[i];
     PointInfo end = segments[i+1];
     printf("segment %d\n----------\n start: (%d, %d) at distance %f(m)\n end: (%d, %d) at distance: %f(m)\n\n",
	      count, start.row, start.col, start.distance, end.row, end.col, end.distance);
     count++;

    //TODO: Hey Stu start here, if you wanna read what else I did just refer to comments here, but this is the
    //beginning of human shape detection per segment. Let me know if you have any questions. Thanks.
    //the top part is how to go about printing and seeing the segments. So now, we can send the two segments,
    //and the rgb image and analyze between those two segments
    //lets call the function we will call, isHuman, we'll see if our segment represents a human.
    //if(isHuman(depth, start, end))
	//printf("success, found a human!\n");
    //    printf(" ");
  }
}




int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "person_identifier" );
  ros::NodeHandle n;

  // subscribe to the /camera/depth/image and /camera/depth/points
  message_filters::Subscriber<sensor_msgs::Image> depth_sub( n, "camera/depth/image", 3);
  message_filters::Subscriber<sensor_msgs::PointCloud2> points_sub( n, "camera/depth/points", 3);
  message_filters::Synchronizer<CameraSyncPolicy> sync(CameraSyncPolicy(10), depth_sub, points_sub);

  //made cv named window here
  if(0){
    cv::namedWindow(WINDOW);
  }

  sync.registerCallback( boost::bind(&depthCb, _1, _2 ) );

  ros::spin();
}
