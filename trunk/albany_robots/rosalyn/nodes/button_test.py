#!/usr/bin/env python

# test button state
import roslib; roslib.load_manifest('rosalyn')
import rospy

from arbotix_msgs.msg import *

class ButtonTest:
    
    def buttonCb(self, msg):
        if msg.value == 0:
            print "Button pressed!"

    def __init__(self):
        rospy.init_node('button_test')
        rospy.Subscriber('/arbotix/head_button',Digital,self.buttonCb)
        rospy.spin()

if __name__ == "__main__":
    bt = ButtonTest()


