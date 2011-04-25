#!/usr/bin/env python

import roslib; roslib.load_manifest('smart_ranger')
import rospy

from geometry_msgs.msg import Twist
from arbotix_msgs.msg import ScanParameters

# modes of operation
TELEPRESENCE = 0
AUTONOMOUS = 1

class PMLcontroller():
    """ Controls the range of the PML, as defined in the paper. """

    def __init__(self):
        self.readings = 15
        self.x = 0
        self.th = 0
        rospy.Subscriber('cmd_vel',Twist,self.trajCb)
        self.pub = rospy.Publisher('scan_parameters',ScanParameters)

    def trajCb(self, traj):
        """ Store last trajectory for scan range computation. """
        self.x = traj.linear.x
        self.th = traj.angular.z

    def update(self, mode=TELEPRESENCE):
        """ Update scan range of PML. """
        p = ScanParameters()
        if mode == TELEPRESENCE:
            # compute safe tolerance for scan width
            if abs(self.x) > 0.3:
                width = abs(self.x) * 4
            else:
                width = 1.2
            # center scan on x direction
            p.angle_increment = width/(self.readings-1)
            p.angle_min = self.th - (width/2.0)
        else:
            pass
        p.readings = self.readings
        self.pub.publish(p)

if __name__ == '__main__':
    rospy.init_node('pml_controller')
    controller = PMLcontroller()
    mode = rospy.get_param('mode', 'telepresence')
    r = rospy.Rate(rospy.get_param('rate', 1.0))
    while not rospy.is_shutdown():
        if mode == 'telepresence':
            controller.update(TELEPRESENCE)
        else:
            controller.update(AUTONOMOUS)
        r.sleep()
    
