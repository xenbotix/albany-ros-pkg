#!/usr/bin/env python

""" Lead Nelson around by the hand
    Michael E. Ferguson, July 18, 2010. """

import roslib; roslib.load_manifest('nelson_apps')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from arbotix.srv import *
#from irobot_create_2_1.srv import *

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

        # services to run the create
        #rospy.wait_for_service('brake')
        #self.brake = rospy.ServiceProxy('brake', Brake)
        #rospy.wait_for_service('tank')
        #self.tank = rospy.ServiceProxy('tank', Tank)
       
        # need to relax the arm to pull him around by
        for servo in ["l_shoulder_pitch","r_shoulder_pitch","l_shoulder","r_shoulder","l_elbow","r_elbow","l_gripper","r_gripper"]:
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
        armElev = float(msg.position[msg.name.index('l_shoulder_pitch')])
        armAngle = float(msg.position[msg.name.index('l_elbow')]) - float(msg.position[msg.name.index('l_shoulder')])
        if armElev > 0:
            # move forward
            self.x_speed = armElev
            self.r_speed = -armAngle #* 2;
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

