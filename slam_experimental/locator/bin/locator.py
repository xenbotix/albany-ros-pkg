#!/usr/bin/env python

import roslib; roslib.load_manifest('locator')
import rospy

import tf
from geometry_msgs.msg import Pose2D, PoseStamped, Pose
from ar_pose.msg import ARMarkers
from math import sin, cos, pi

class Locator:
    """ A class for localizing a robot using ar markers and odometry. """

    def __init__(self):
        self.pose = Pose2D()
        self.map_frame = rospy.get_param('map_frame', 'map')
        self.odom_frame = rospy.get_param('odom_frame', 'odom')
        self.base_frame = rospy.get_param('base_frame', 'base_link')
        # load map (TODO: this is temporary, remove when map_server is ready)
#       self.markers = { 0: ["ILS_OFFICE",0.4064,0.6858,1.22,0,0,-1.570796] }
        self.markers = { 0: ["ILS_OFFICE",-0.4064,-1.22,0.6858,-1.570796,0,0] }
#       self.markers = { 0: ["Office_Nick",4.97,0.71,1.22,0,0,3.14159],
#                        1: ["Office_Lynne",4.97,3.66,1.22,0,0,3.14159],
#                        2: ["Office_Tomek",4.97,5.08,1.22,0,0,3.14159],
#                        3: ["Office_Tomek_2",3.8,5.31,1.22,0,0,-1.570796],
#                        4: ["Office_Mail",0.71,-0.69,1.22,0,0,1.570796],
#                        5: ["Office_Exit",0,0.66,1.22,0,0,0] }
        self.markers = { 0: ["Office_Nick",0.71,-1.22,4.97,-1.570796,1.570796,0],
                         1: ["Office_Lynne",3.66,-1.22,4.97,-1.570796,1.570796,0],
                         2: ["Office_Tomek",5.08,-1.22,4.97,-1.570796,1.570796,0],
                         3: ["Office_Tomek_2",-3.8,-1.22,5.31,-1.570796,0,0],
                         4: ["Office_Mail",0.71,-1.22,0.69,-1.570796,3.14159,0],
                         5: ["Office_Exit",-0.66,-1.22,0,-1.570796,-1.570796,0] }
        # setup publishers and subscribers
        rospy.Subscriber('ar_pose_marker', ARMarkers, self.localize)
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()        
       
    def localize(self, data):
        for marker in data.markers:
            if marker.id in self.markers.keys():
                try:
                    (frame, xp, yp, zp, xr, yr, zr) = self.markers[marker.id]
                    # get odom->map transform (via ar->map->odom)
                    arpose=PoseStamped()
                    arpose.header.frame_id = frame
                    arpose.header.stamp = rospy.Time(0)
                    arpose.pose.position.x = xp
                    arpose.pose.position.y = yp
                    arpose.pose.position.z = zp
                    q = tf.transformations.quaternion_from_euler(xr, yr, zr)
                    arpose.pose.orientation.x = q[0]
                    arpose.pose.orientation.y = q[1]
                    arpose.pose.orientation.z = q[2]
                    arpose.pose.orientation.w = q[3]
                    p = self.listener.transformPose(self.odom_frame, arpose)
                    # update odom->map pose
                    q = p.pose.orientation
                    (phi, psi, theta) = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
                    self.pose.x = p.pose.position.x
                    self.pose.y = p.pose.position.y
                    self.pose.theta = theta
                    if self.pose.theta > pi:
                        self.pose.theta -= 2*pi
                    elif self.pose.theta < -pi:
                        self.pose.theta += 2*pi
                    print "Localized against marker:", frame, self.pose
                except (tf.LookupException, tf.ConnectivityException):
                    continue

    def update(self):
        self.broadcaster.sendTransform((self.pose.x, self.pose.y, 0),
                                       tf.transformations.quaternion_from_euler(0, 0, self.pose.theta),
                                       rospy.Time.now(),
                                       self.map_frame, self.odom_frame)

if __name__ == '__main__':
    rospy.init_node('locator')
    locator = Locator()
    r = rospy.get_param('rate', 10.0)
    rate = rospy.Rate(r)
    while not rospy.is_shutdown():
        locator.update()
        rate.sleep()

