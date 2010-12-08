/*
 * Magic Emotion Box - Nelson Emotional Control Program
 * Authors: Ken Stahl & Michael Ferguson
 */

//#include <unistd.h>
//#include <stdio.h>

#include "ros/ros.h"
#include "social_msgs/AffectiveState.h"
#include "sensor_msgs/JointState.h"

class MagicEmotionBox
{
public:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  MagicEmotionBox(ros::NodeHandle nh) : nh_(nh){
    sub_ = nh.subscribe("affective_state", 10, &MagicEmotionBox::affectCallback, this);
    pub_ = nh.advertise<sensor_msgs::JointState>("cmd_joints", 10);
  }

  void affectCallback(const social_msgs::AffectiveState::ConstPtr& msg){
    // process messages here
    
    // and publish a joint states message
  }

};

/* Magic Emotion Box Node */
int main(int argc, char **argv)
{
  ros::init(argc, argv,"magic_emotion_box");
  ros::NodeHandle nh;
  MagicEmotionBox em_box(nh);
  ros::spin();
  return 0;
}
