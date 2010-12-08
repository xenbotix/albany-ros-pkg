/*
 * Magic Emotion Box - Nelson Emotional Control Program
 * Authors: Ken Stahl & Michael Ferguson
 */

#include <iostream>
#include <stdlib.h>

#include "ros/ros.h"
#include "social_msgs/AffectiveState.h"
#include "sensor_msgs/JointState.h"
#include "emotion.h"

using namespace std;
  Emotion e;
  Pose *p;
  int SERVONUM = 19;
  const std::string servo[] = {"r_shoulder_lift_joint", "r_shoulder_pan_joint", "r_elbow_flex_joint",
		  "r_gripper_joint", "l_shoulder_lift_joint", "l_shoulder_pan_joint",
		  "l_elbow_flex_joint", "l_gripper_joint", "head_pan_joint",
	 	  "head_tilt_joint", "head_tilt2_joint", "eye_tilt",
		  "r_eye_pan", "l_eye_pan", "eyelids",
		  "top_lip", "bottom_lip", "r_eyebrow", "l_eyebrow"};

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
        int xx, yy;
		xx = (int) (msg->valence * 50 + 50);
		yy = (int) (msg->activation * 50 + 50);
		e.setpos(xx, yy);
		p = e.currentpose();

		sensor_msgs::JointState positions;

		for (int i = 0; i < SERVONUM; i++){
			int servoval = p->getservo(i);
			positions.name.resize(8);
			positions.name.push_back(servo[i]);
			positions.position.resize(8);
			positions.position.push_back( (servoval - 512) * (5.2359877/1024.0) );  //5.23 is rad(300)
		}

		pub_.publish(positions);
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
