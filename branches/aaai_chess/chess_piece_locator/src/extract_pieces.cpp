/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: extract_clusters.hpp 32052 2010-08-27 02:19:30Z rusu $
 *
 */

#include <pluginlib/class_list_macros.h>
#include <pcl/io/io.h>
#include "extract_pieces.h"

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PieceExtraction::onInit ()
{
  // Call the super onInit ()
  pnh_.reset (new ros::NodeHandle (getMTPrivateNodeHandle ()));
        
  // Parameters that we care about only at startup
  pnh_->getParam ("max_queue_size", max_queue_size_);
        
  // ---[ Optional parameters
  pnh_->getParam ("use_indices", use_indices_);
  pnh_->getParam ("latched_indices", latched_indices_);
  pnh_->getParam ("approximate_sync", approximate_sync_);

  NODELET_DEBUG ("[%s::onInit] PCL Nodelet successfully created with the following parameters:\n"
            " - approximate_sync : %s\n"
            " - use_indices      : %s\n"
            " - latched_indices  : %s\n"
            " - max_queue_size   : %d",
            getName ().c_str (), 
            (approximate_sync_) ? "true" : "false",
            (use_indices_) ? "true" : "false", 
            (latched_indices_) ? "true" : "false", 
            max_queue_size_);

  // ---[ Mandatory parameters
  double cluster_tolerance;
  if (!pnh_->getParam ("cluster_tolerance", cluster_tolerance))
  {
    NODELET_ERROR ("[%s::onInit] Need a 'cluster_tolerance' parameter to be set before continuing!", getName ().c_str ()); 
    return;
  }
  int spatial_locator;
  if (!pnh_->getParam ("spatial_locator", spatial_locator))
  {
    NODELET_ERROR ("[%s::onInit] Need a 'spatial_locator' parameter to be set before continuing!", getName ().c_str ());
    return;
  }

  // output
  pub_output_ = pnh_->advertise<chess_msgs::ChessBoard>("output", max_queue_size_); 
  //pub_output_ = pnh_->advertise<PointCloud>("output", max_queue_size_);

  // Enable the dynamic reconfigure service
  srv_ = boost::make_shared <dynamic_reconfigure::Server<PieceExtractionConfig> > (*pnh_);
  dynamic_reconfigure::Server<PieceExtractionConfig>::CallbackType f =  boost::bind (&PieceExtraction::config_callback, this, _1, _2);
  srv_->setCallback (f);

  // If we're supposed to look for PointIndices (indices)
  if (use_indices_)
  {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe (*pnh_, "input", max_queue_size_);
    sub_indices_filter_.subscribe (*pnh_, "indices", max_queue_size_);

    if (approximate_sync_)
    {
      sync_input_indices_a_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud, PointIndices> > > (max_queue_size_);
      sync_input_indices_a_->connectInput (sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback (bind (&PieceExtraction::input_indices_callback, this, _1, _2));
    }
    else
    {
      sync_input_indices_e_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ExactTime<PointCloud, PointIndices> > > (max_queue_size_);
      sync_input_indices_e_->connectInput (sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback (bind (&PieceExtraction::input_indices_callback, this, _1, _2));
    }
  }
  else
    // Subscribe in an old fashion to input only (no filters)
    sub_input_ = pnh_->subscribe<PointCloud> ("input", max_queue_size_, bind (&PieceExtraction::input_indices_callback, this, _1, PointIndicesConstPtr ()));

  NODELET_DEBUG ("[%s::onInit] Nodelet successfully created with the following parameters:\n"
                 " - max_queue_size    : %d\n"
                 " - use_indices       : %s\n"
                 " - cluster_tolerance : %f\n"
                 " - spatial_locator   : %d",
                 getName ().c_str (),
                 max_queue_size_,
                 (use_indices_) ? "true" : "false", cluster_tolerance, spatial_locator);

  // Set given parameters here
  impl_.setSpatialLocator (spatial_locator);
  impl_.setClusterTolerance (cluster_tolerance);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PieceExtraction::config_callback (PieceExtractionConfig &config, uint32_t level)
{
  if (impl_.getClusterTolerance () != config.cluster_tolerance)
  {
    impl_.setClusterTolerance (config.cluster_tolerance);
    NODELET_DEBUG ("[%s::config_callback] Setting new clustering tolerance to: %f.", getName ().c_str (), config.cluster_tolerance);
  }
  if (impl_.getMinClusterSize () != config.cluster_min_size)
  {
    impl_.setMinClusterSize (config.cluster_min_size);
    NODELET_DEBUG ("[%s::config_callback] Setting the minimum cluster size to: %d.", getName ().c_str (), config.cluster_min_size);
  }
  if (impl_.getMaxClusterSize () != config.cluster_max_size)
  {
    impl_.setMaxClusterSize (config.cluster_max_size);
    NODELET_DEBUG ("[%s::config_callback] Setting the maximum cluster size to: %d.", getName ().c_str (), config.cluster_max_size);
  }
  if (max_clusters_ != config.max_clusters)
  {
    max_clusters_ = config.max_clusters;
    NODELET_DEBUG ("[%s::config_callback] Setting the maximum number of clusters to extract to: %d.", getName ().c_str (), config.max_clusters);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl_ros::PieceExtraction::input_indices_callback (
      const PointCloudConstPtr &cloud, const PointIndicesConstPtr &indices)
{
  // No subscribers, no work
  if (pub_output_.getNumSubscribers () <= 0)
    return;

  // If cloud is given, check if it's valid
  if (!isValid (cloud))
  {
    NODELET_ERROR ("[%s::input_indices_callback] Invalid input!", getName ().c_str ());
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid (indices))
  {
    NODELET_ERROR ("[%s::input_indices_callback] Invalid indices!", getName ().c_str ());
    return;
  }

  /// DEBUG
  if (indices)
    NODELET_DEBUG ("[%s::input_indices_callback]\n"
                   "                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
                   "                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
                   getName ().c_str (),
                   cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), cloud->header.stamp.toSec (), cloud->header.frame_id.c_str (), pnh_->resolveName ("input").c_str (),
                   indices->indices.size (), indices->header.stamp.toSec (), indices->header.frame_id.c_str (), pnh_->resolveName ("indices").c_str ());
  else
    NODELET_DEBUG ("[%s::input_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.", getName ().c_str (), cloud->width * cloud->height, cloud->header.stamp.toSec (), cloud->header.frame_id.c_str (), pnh_->resolveName ("input").c_str ());
  ///

  IndicesConstPtr indices_ptr;
  if (indices)
    indices_ptr = boost::make_shared <std::vector<int> > (indices->indices);

  PointCloud cloud_transformed;
  if (!pcl_ros::transformPointCloud (std::string("torso_link"), *cloud, cloud_transformed, tf_listener_))
  {
      //NODELET_ERROR ("[%s::computePublish] Error converting output dataset from %s to %s.", getName ().c_str (), output.header.frame_id.c_str (), tf_output_frame_.c_str ());
      //return;
  }
  //cloud.reset (new PointCloud2 (cloud_transformed));

  impl_.setInputCloud (cloud_transformed.makeShared());
  impl_.setIndices (indices_ptr);

  std::vector<PointIndices> clusters;
  impl_.extract (clusters);

  // output ChessBoard
  chess_msgs::ChessBoard cb;
  for (size_t c = 0; c < clusters.size (); ++c)
  {
    chess_msgs::ChessPiece p;
    p.header.frame_id = cloud_transformed.header.frame_id;
    
    // find cluster centroid/color
    float x = 0; float y = 0; float z = 0; int color = 0;
    for (size_t i = 0; i < clusters[c].indices.size(); i++)
    {
        int j = clusters[c].indices[i];
        x += cloud_transformed.points[j].x;
        y += cloud_transformed.points[j].y;
        z += cloud_transformed.points[j].z;
        unsigned char * rgb = (unsigned char *) &(cloud_transformed.points[j].rgb);
        color += (rgb[0] + rgb[1] + rgb[2])/3;
    }
    x = x/clusters[c].indices.size();
    y = y/clusters[c].indices.size();
    z = z/clusters[c].indices.size();
    // set centroid
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.position.z = z;
    // set color
    color = color/clusters[c].indices.size();
    if(color > 100){
        p.type = chess_msgs::ChessPiece::WHITE_UNKNOWN;   
    }else{
        p.type = chess_msgs::ChessPiece::BLACK_UNKNOWN;
    }

    bool new_ = true;
    // check if cluster overlaps any other
    for (size_t j = 0; j < cb.pieces.size(); j++)
    {
        if( (fabs(p.pose.position.x-cb.pieces[j].pose.position.x) < 0.012) &&
            (fabs(p.pose.position.y-cb.pieces[j].pose.position.y) < 0.012) )
        {
            cb.pieces[j].pose.position.x = (cb.pieces[j].pose.position.x + p.pose.position.x)/2;
            cb.pieces[j].pose.position.y = (cb.pieces[j].pose.position.y + p.pose.position.y)/2;
            cb.pieces[j].pose.position.z = (cb.pieces[j].pose.position.z + p.pose.position.z)/2;  
            cb.pieces[j].pts += p.pts;
            cb.pieces[j].color = (cb.pieces[j].color + p.color)/2;
            new_ = false;
            break;
        }
    }  
    
    // add cluster
    if (new_)
        cb.pieces.push_back(p);

    //NODELET_DEBUG ("[segmentAndPublish] Published cluster %zu (with %zu values and stamp %f) on topic %s",
    //               i, clusters[i].indices.size (), output.header.stamp.toSec (), pnh_->resolveName ("output").c_str ());

  }
  /*PointCloud cl;
  cl.header.frame_id = cb.pieces[0].header.frame_id;
  cl.header.stamp = cloud->header.stamp;
  for(size_t i = 0; i < cb.pieces.size(); i++){
    pcl::PointXYZRGB p;
    p.x = cb.pieces[i].pose.position.x;
    p.y = cb.pieces[i].pose.position.y;
    p.z = cb.pieces[i].pose.position.z;
    cl.push_back(p);
  }
  pub_output_.publish(cl);*/
  pub_output_.publish(cb);
}

typedef pcl_ros::PieceExtraction PieceExtraction;
PLUGINLIB_DECLARE_CLASS (chess_piece_locator, PieceExtraction, PieceExtraction, nodelet::Nodelet);

