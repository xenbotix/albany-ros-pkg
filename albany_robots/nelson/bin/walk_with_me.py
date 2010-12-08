#!/usr/bin/env python

""" Lead Nelson around by the hand
    Copyright 2010 Michael E. Ferguson """

import roslib; roslib.load_manifest('nelson')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from arbotix.srv import *

class walkWithMe():
    def __init__(self, name='WalkWithMe'):
        self.name = name
        self.pitch = 0
        self.angle = 0
        
        # startup and subscribe to joint_states
        rospy.init_node(name, anonymous=True)
        rospy.Subscriber('joint_states', JointState, self.callback)
        rospy.on_shutdown(self.cleanup)
        
        # publish to cmd_vel
        self.p = rospy.Publisher('cmd_vel', Twist)
       
        # need to relax the arm to pull him around by
        for servo in ["l_shoulder_lift_joint","r_shoulder_lift_joint","l_shoulder_pan_joint","r_shoulder_pan_joint","l_elbow_flex_joint","r_elbow_flex_joint","l_gripper_joint","r_gripper_joint"]:
            x = rospy.ServiceProxy(servo+"_relax",Relax)
            x()

        rospy.loginfo('walk_with_me.py initialized')

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = self.x_speed; twist.linear.y = 0; twist.linear.z = 0;            
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.r_speed;
            self.p.publish(twist)
            r.sleep()
            
    def callback(self, msg):
        armElev = float(msg.position[msg.name.index('l_shoulder_lift_joint')])
        armAngle = float(msg.position[msg.name.index('l_elbow_flex_joint')]) - float(msg.position[msg.name.index('l_shoulder_pan_joint')])
        if armElev > 0:
            # move forward
            self.x_speed = armElev
            self.r_speed = -armAngle;
        else:
            self.x_speed = 0.0
            self.r_speed = 0.0
        #print self.x_speed, self.r_speed

    def cleanup(self):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;            
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
        self.p.publish(twist)
        
if __name__ == "__main__":  
    w = walkWithMe()

