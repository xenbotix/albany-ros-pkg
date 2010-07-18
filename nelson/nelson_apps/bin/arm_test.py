#!/usr/bin/env python

""" Simple Arm Test for Nelson
    Voodoo control of right arm, using left arm.
    Michael E. Ferguson, 2010. """

import roslib; roslib.load_manifest('nelson_apps')
import rospy

from sensor_msgs.msg import JointState
from arbotix.srv import *

mapping = { "l_shoulder_pitch":"r_shoulder_pitch", "l_shoulder":"r_shoulder", "l_elbow":"r_elbow", "l_gripper":"r_gripper" }
outputs = dict()
relax = dict()
values = dict()

def moveArms( msg ):
    """ We use this callback to update the arm output positions every time 
        the joint_states topic is published to. The actual output is done 
        within the main loop. """
    for read in mapping:
        write = mapping[read] 
        index = msg.name.index(read)
        values[write] = msg.position[index]

def cleanup():
    """ We use this callback to relax all servos when our node exits. """
    for servo in relax.keys():
        relax[servo]()

if __name__ == '__main__':
    # initialize and listen to joint_states topic
    rospy.init_node('arm_test')
    rospy.Subscriber('joint_states', JointState, moveArms)
    
    # this is so we can cleanup by relaxing arms when we exit
    for servo in mapping.values():
        outputs[servo] = rospy.ServiceProxy(servo+'_setangle',SetAngle) 
        relax[servo] = rospy.ServiceProxy(servo+'_relax',Relax) 
    for servo in mapping.keys():       
        relax[servo] = rospy.ServiceProxy(servo+'_relax',Relax) 
        relax[servo]()
    rospy.on_shutdown(cleanup)

    # logging information is a good idea
    rospy.loginfo('arm_test.py initialized')

    # Let this run at approximately 30hz
    # rospy.Rate handles all timing for us
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        for servo in values.keys():
            outputs[servo](values[servo], 50)
        r.sleep()

