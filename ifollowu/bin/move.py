#!/usr/bin/env python

""" Example code of how to move a robot back and forth. """

# we always import these
import roslib; roslib.load_manifest('ex_move')
import rospy

# recall: robots generally take base movement commands on a topic 
#  called "cmd_vel" using a message type "geometry_msgs/Twist"
from geometry_msgs.msg import Twist, Point

# a nasty global variable
p = None


def cleanup():
    """ You should put comments like this inside each function. 
        This function is called when the node shuts down -- it stops the robot. """
    global p
    twist = Twist()
    p.publish(twist)

def callback(msg):
    """ You should put comments like this inside each function. 
        This function is called when the node shuts down -- it stops the robot. """
    global p
    twist = Twist()
    twist.linear.x = (msg.z - 1.0)*2.0;
    twist.angular.z = - msg.x * 7.0;
    print twist.linear.x, twist.angular.z
    p.publish(twist)


# this quick check means that the following code runs ONLY if this is the 
# main file -- if we "import move" in another file, this code will not execute.
if __name__=="__main__":

    # first thing, init a node!
    rospy.init_node('move')

    # setup to call this function on node exit
    rospy.on_shutdown(cleanup)

    # publish to cmd_vel
    p = rospy.Publisher('cmd_vel', Twist)
    rospy.Subscriber('person_location', Point, callback)

    rospy.spin()

