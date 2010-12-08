#!/usr/bin/env python

# drive around, avoid obstacles
import roslib; roslib.load_manifest('albany_common')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Avoid:
    ranges = [3.0 for i in range(30)]    

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.p.publish(twist)

    def laserCb(self, data):
        print "scan recieved"
        self.ranges = data.ranges

    def __init__(self):
        rospy.init_node('avoid')
        rospy.on_shutdown(self.cleanup)
        # publish to cmd_vel
        self.p = rospy.Publisher('cmd_vel', Twist)
        rospy.Subscriber("base_scan",LaserScan,self.laserCb)
        rospy.sleep(1)

        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.15
            for i in range(10):
                if self.ranges[10+i] < 1.0:
                    twist.angular.z = 1.57/2
                    twist.linear.x = 0
                    self.ranges = [3.0 for i in range(30)] 
            self.p.publish(twist)
            print "published"
            r.sleep()

if __name__ == "__main__":
    a = Avoid()


