#!/usr/bin/env python

import roslib; roslib.load_manifest('arbotix_controllers')
import rospy
import sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

servos = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 'arm_wrist_roll_joint']

forward = [-0.0409061543436171, 0.014913702104443736, 1.4817828305200882, 1.6515859816235403, -0.066472500808377785]
to_side = [1.4675082870772636, 0.024501082028728992, 1.4817828305200882, 1.6157930965728755, -0.066472500808377785]
tucked = [1.4675082870772636, -0.48576058283045309, 1.4817828305200882, 1.7180584824319183, -0.066472500808377785]

class tuck_arm:
    
    def __init__(self, pub=None):
        if pub != None:
            self.pub = pub
        else:
            self.pub = rospy.Publisher('arm_controller/command', JointTrajectory)

    def tuck(self):
        # prepare a joint trajectory
        msg = JointTrajectory()
        msg.joint_names = servos
        msg.points = list()
    
        point = JointTrajectoryPoint()
        point.positions = forward
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(5.0)
        msg.points.append(point)
        point = JointTrajectoryPoint()
        point.positions = to_side
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(8.0)
        msg.points.append(point)
        point = JointTrajectoryPoint()
        point.positions = tucked
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(11.0)
        msg.points.append(point)

        # publish
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        self.pub.publish(msg)

    def untuck(self):
        # prepare a joint trajectory
        msg = JointTrajectory()
        msg.joint_names = servos
        msg.points = list()
        
        point = JointTrajectoryPoint()
        point.positions = to_side
        point.velocities = [ 0.0 for servo in msg.joint_names ]
        point.time_from_start = rospy.Duration(3.0)
        msg.points.append(point)

        # publish
        msg.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        self.pub.publish(msg)
    
if __name__=="__main__":
    rospy.init_node("tuck_arm")
    pub = rospy.Publisher('arm_controller/command', JointTrajectory, latch=True)
    tuck = tuck_arm(pub)
    
    # tucking or untucking?
    if len(sys.argv) > 1 and sys.argv[1] == "u":
        tuck.untuck()
    else:
        tuck.tuck()
    
    # sleep three secs (while latched)
    rospy.sleep(3)

