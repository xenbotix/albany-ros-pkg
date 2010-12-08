#!/usr/bin/env python

""" Put Nelson's head in the up position
    Michael E. Ferguson, 2010. """

import roslib; roslib.load_manifest('nelson')
import rospy

import time

from sensor_msgs.msg import JointState
from arbotix.srv import *

neck = {"head_pan_joint":0.0, "head_tilt_joint": 0.0, "head_tilt2_joint":0.2 }

rospy.init_node('wakeup')
cmd_joints = rospy.Publisher('cmd_joints', JointState)

r = rospy.Rate(1)
for i in range(10):
    j = JointState()
    for servo in neck.keys():
        j.name.append(servo)
        j.position.append(neck[servo])
    cmd_joints.publish(j)
    r.sleep()

