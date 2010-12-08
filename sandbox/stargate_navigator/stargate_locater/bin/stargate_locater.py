#!/usr/bin/env python

""" Stargate Navigation System """

import roslib; roslib.load_manifest('stargate_locater')
import rospy

import tf

class stargate_locater:
    def __init__(self):
        self.br = tf.TransformBroadcaster()

    # Disparity between real marker and observed is used to correct odometry:  
    #   static tf does base_link -> usb_cam
    #   ar_pose publishes usb_cam -> marker
    #   map server publishes real marker -> map
    #   map server publishes map -> world???

    #   create driver publishes base_link -> odom
    #   this node will publish odom -> map

    def updateTF(self):
        """ currently does nothing! """
        self.br.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"map","odom")

if __name__ == "__main__":  
    rospy.init_node('stargate_locater')
    l = stargate_locater()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        l.updateTF()
        r.sleep()
