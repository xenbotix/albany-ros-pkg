#!/usr/bin/env python

import roslib; roslib.load_manifest('albany_common')
import rospy

from geometry_msgs.msg import Twist

x_speed = 0.2  # 0.1 m/s
r_angle = 0.0  # 0.1 rad/s

def cleanup():
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;            
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = r_angle;
    p.publish(twist)

rospy.init_node('move')
rospy.on_shutdown(cleanup)
# publish to cmd_vel
p = rospy.Publisher('cmd_vel', Twist)

r = rospy.Rate(10)
c = 0
while not rospy.is_shutdown():
    twist = Twist()
    twist.linear.x = x_speed; twist.linear.y = 0; twist.linear.z = 0;            
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = r_angle;
    p.publish(twist)
    c = c + 1
    if c > 40:
        x_speed = -0.2
    if c > 80:
        c = 0
        x_speed = 0.2
    r.sleep()


