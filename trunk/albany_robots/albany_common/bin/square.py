#!/usr/bin/env python

# Draw a square - woohoo!
import roslib; roslib.load_manifest('albany_common')
import rospy

from geometry_msgs.msg import Twist

def cleanup():
    # stop the robot!
    twist = Twist()
    p.publish(twist)

rospy.init_node('square')
rospy.on_shutdown(cleanup)
# publish to cmd_vel
p = rospy.Publisher('cmd_vel', Twist)
rospy.sleep(1)

r = rospy.Rate(0.5)
for i in range(4):
    twist = Twist()
    twist.linear.x = 0.15;
    p.publish(twist)
    r.sleep()
    twist = Twist()
    twist.angular.z = 1.57/2;
    p.publish(twist)
    r.sleep()


