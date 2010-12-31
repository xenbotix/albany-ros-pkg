/*
 * slam_coreslam
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 * 
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey 
           Extended for CoreSLAM by Michael Ferguson */

#include "slam_coreslam.h"

#include <iostream>
#include <time.h>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamCoreSlam::SlamCoreSlam():
  map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 ))),
  laser_count_(0), transform_thread_(NULL)
{

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  got_first_scan_ = false;
  got_map_ = false;

  ros::NodeHandle private_nh_("~");

  // Be consistent with gmapping/karto
  if(!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  if(!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if(!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if(!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  double transform_publish_period;
  private_nh_.param("transform_publish_period", transform_publish_period, 0.05);

  double tmp;
  if(!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);
  
  // Parameters needed for CoreSLAM
  if(!private_nh_.getParam("sigma_xy", sigma_xy_))
    sigma_xy_ = 0.005;
  if(!private_nh_.getParam("sigma_theta", sigma_theta_))
    sigma_theta_ = 0.005;
  if(!private_nh_.getParam("hole_width", hole_width_))
    hole_width_ = 600;
  if(!private_nh_.getParam("span", span_))
    span_ = 3;
  if(!private_nh_.getParam("delta", delta_))
     delta_ = 0.05;
  ts_map_set_scale(MM_TO_METERS/delta_);

  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  ss_ = node_.advertiseService("dynamic_map", &SlamCoreSlam::mapCallback, this);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamCoreSlam::laserCallback, this, _1));

  transform_thread_ = new boost::thread(boost::bind(&SlamCoreSlam::publishLoop, this, transform_publish_period));
}

void SlamCoreSlam::publishLoop(double transform_publish_period){
  if(transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while(ros::ok()){
    publishTransform();
    r.sleep();
  }
}

SlamCoreSlam::~SlamCoreSlam()
{
  if(transform_thread_){
    transform_thread_->join();
    delete transform_thread_;
  }

  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;
}

bool
SlamCoreSlam::getOdomPose(ts_position_t& ts_pose, const ros::Time &t)
{
  // Get the base_link->odom
  tf::Stamped<tf::Pose> ident (btTransform(tf::createQuaternionFromRPY(0,0,0),
                                           btVector3(0,0,0)), t, base_frame_);
  tf::Stamped<btTransform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  ts_pose.x = odom_pose.getOrigin().x()*METERS_TO_MM + ((TS_MAP_SIZE/2)*delta_*METERS_TO_MM); // convert to mm
  ts_pose.y = odom_pose.getOrigin().y()*METERS_TO_MM + ((TS_MAP_SIZE/2)*delta_*METERS_TO_MM);
  ts_pose.theta = (yaw * 180/M_PI);

  ROS_DEBUG("ODOM POSE: %f, %f, %f", ts_pose.x, ts_pose.y, ts_pose.theta);

  return true;
}

bool
SlamCoreSlam::initMapper(const sensor_msgs::LaserScan& scan)
{
  // Transform laser to base link
  tf::Stamped<tf::Pose> ident (btTransform(tf::createQuaternionFromRPY(0,0,0),
                                           btVector3(0,0,0)), scan.header.stamp, scan.header.frame_id);
  tf::Stamped<btTransform> laser_pose;
  try
  {
    tf_.transformPose(base_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // configure previous_odom
  if(!getOdomPose(prev_odom_, scan.header.stamp))
     return false;
  position_ = prev_odom_;

  // configure laser parameters
  lparams_.offset = laser_pose.getOrigin().x();
  lparams_.scan_size = scan.ranges.size();
  lparams_.angle_min = scan.angle_min * 180/M_PI;
  lparams_.angle_max = scan.angle_max * 180/M_PI;
  lparams_.detection_margin = 0;
  lparams_.distance_no_detection = scan.range_max * METERS_TO_MM;

  // new coreslam instance
  ts_map_init(&ts_map_);
  ts_state_init(&state_, &ts_map_, &lparams_, &position_, sigma_xy_, sigma_theta_, hole_width_, 0);
  
  ROS_INFO("Initialized with sigma_xy=%f, sigma_theta=%f, hole_width=%d, delta=%f",sigma_xy_, sigma_theta_, hole_width_, delta_);
  ROS_INFO("Initialization complete");
  return true; 
}

bool
SlamCoreSlam::addScan(const sensor_msgs::LaserScan& scan, ts_position_t& odom_pose)
{
  // update odometry
  if(!getOdomPose(odom_pose, scan.header.stamp))
     return false;
  state_.position.x += odom_pose.x - prev_odom_.x;
  state_.position.y += odom_pose.y - prev_odom_.y;
  state_.position.theta += odom_pose.theta - prev_odom_.theta;
  prev_odom_ = odom_pose;

  ts_position_t prev = state_.position;

  // update params -- mainly for PML
  lparams_.scan_size = scan.ranges.size();
  lparams_.angle_min = scan.angle_min * 180/M_PI;
  lparams_.angle_max = scan.angle_max * 180/M_PI;

  if(laser_count_ < 10){
    // not much of a map, let's bootstrap for now
    ts_scan_t ranges;
    ranges.nb_points = 0;
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max){
        ranges.x[ranges.nb_points] = cos(scan.angle_min + i*scan.angle_increment) * (scan.ranges[i]*METERS_TO_MM);
        ranges.y[ranges.nb_points] = sin(scan.angle_min + i*scan.angle_increment) * (scan.ranges[i]*METERS_TO_MM);
        ranges.value[ranges.nb_points] = TS_OBSTACLE;
        ranges.nb_points++;
      }
    }
    ts_map_update(&ranges, &ts_map_, &state_.position, 50, hole_width_);  
    ROS_DEBUG("Update step, %d, now at (%f, %f, %f)",laser_count_, state_.position.x, state_.position.y, state_.position.theta);
  }else{
    ts_sensor_data_t data;
    data.position[0] = state_.position;
    if(lparams_.angle_max < lparams_.angle_min){
      // flip readings
      for(unsigned int i=0; i < scan.ranges.size(); i++)
        data.d[i] = (int) (scan.ranges[scan.ranges.size()-1-i]*METERS_TO_MM);
    }else{
      for(unsigned int i=0; i < scan.ranges.size(); i++)
        data.d[i] = (int) (scan.ranges[i]*METERS_TO_MM);
    } 
    ts_iterative_map_building(&data, &state_);  
    ROS_DEBUG("Iterative step, %d, now at (%f, %f, %f)",laser_count_, state_.position.x, state_.position.y, state_.position.theta);
    ROS_DEBUG("Correction: %f, %f, %f", state_.position.x - prev.x, state_.position.y - prev.y, state_.position.theta - prev.theta);
  }

  odom_pose = state_.position;

  return true;
}

void
SlamCoreSlam::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0,0);

  // We can't initialize CoreSLAM until we've got the first scan
  if(!got_first_scan_)
  {
    if(!initMapper(*scan))
      return;
    got_first_scan_ = true;
  }

  ts_position_t odom_pose;
  if(addScan(*scan, odom_pose))
  {
    ROS_DEBUG("scan processed");
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);

    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf_.transformPose(odom_frame_,tf::Stamped<tf::Pose> (btTransform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta*M_PI/180),
                                                                    btVector3((odom_pose.x-(TS_MAP_SIZE/2)*delta_*METERS_TO_MM)*MM_TO_METERS, 
                                                                              (odom_pose.y-(TS_MAP_SIZE/2)*delta_*METERS_TO_MM)*MM_TO_METERS, 0.0)).inverse(),
                                                                    scan->header.stamp, base_frame_),odom_to_map);
    }
    catch(tf::TransformException e){
      ROS_ERROR("Transform from base_link to odom failed\n");
      odom_to_map.setIdentity();
    }

    // This looks a bit crazy, but localization is currently done *inside* coreslam -- so odom->map never changes. Probably should fix that
    map_to_odom_mutex_.lock();
    map_to_odom_ = tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
                                 tf::Point(      odom_to_map.getOrigin() ) ).inverse();
    map_to_odom_mutex_.unlock();

    if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      updateMap();
      last_map_update = scan->header.stamp;
      ROS_DEBUG("Updated the map");
    }
  }
}

void
SlamCoreSlam::updateMap()
{
  boost::mutex::scoped_lock(map_mutex_);

  if(!got_map_) {
    map_.map.info.resolution = delta_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  if(map_.map.info.width != TS_MAP_SIZE || map_.map.info.height != TS_MAP_SIZE){
    map_.map.info.width = TS_MAP_SIZE;
    map_.map.info.height = TS_MAP_SIZE;
    map_.map.info.origin.position.x = -(TS_MAP_SIZE/2)*delta_;
    map_.map.info.origin.position.y = -(TS_MAP_SIZE/2)*delta_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);  
  }

  for(int x=0; x < TS_MAP_SIZE; x++)
  {
    for(int y=0; y < TS_MAP_SIZE; y++)
    {
      int occ= (int)(ts_map_.map[ y * TS_MAP_SIZE + x]);
      if(occ == (TS_OBSTACLE+TS_NO_OBSTACLE)/2 )
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ < (TS_OBSTACLE+TS_NO_OBSTACLE)/2 )
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
}

bool
SlamCoreSlam::mapCallback(nav_msgs::GetMap::Request  &req,
                          nav_msgs::GetMap::Response &res)
{
  boost::mutex::scoped_lock(map_mutex_);
  if(got_map_ && map_.map.info.width && map_.map.info.height)
  {
    res = map_;
    return true;
  }
  else
    return false;
}

void 
SlamCoreSlam::publishTransform()
{
  map_to_odom_mutex_.lock();
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
  tfB_->sendTransform( tf::StampedTransform (map_to_odom_, ros::Time::now(), map_frame_, odom_frame_));
  map_to_odom_mutex_.unlock();
}

