#!/usr/bin/env python

""" Put Nelson's head in the up position
    Michael E. Ferguson, 2010. """

import roslib; roslib.load_manifest('nelson_common')
import rospy

import time

from sensor_msgs.msg import JointState
from arbotix.srv import *

neck = {"head_pan":0.0, "head_tilt": 0.0, "head_tilt2":0.2 }

rospy.init_node('wakeup')
time.sleep(3.0)

while

for servo, value in neck.items():
    rospy.wait_for_service(servo+'_setangle')
    s = rospy.ServiceProxy(servo+'_setangle',SetAngle)
    s(value)

