#!/usr/bin/env python

import roslib; roslib.load_manifest('office_courier')
import rospy

import tf
import math

from office_courier.srv import *
from arbotix_msgs.msg import *
from geometry_msgs.msg import Twist

class CourierServer:

    def __init__(self):
        rospy.init_node("courier_node")        
        rospy.on_shutdown(self.cleanup)

        # setup tf for translating poses
        self.listener = tf.TransformListener()
        rospy.sleep(2)

        # subscribe to button
        self.buttonValue = 1
        rospy.Subscriber('/arbotix/head_button',Digital,self.buttonCb)

        # get parameters for tolerances
        self.yaw_tolerance = rospy.get_param("~yaw_goal_tolerance",0.10)
        self.xy_tolerance = rospy.get_param("~xy_goal_tolerance",0.20)

        # create a schedule of places to go, empty right now
        self.schedule = list()

        # publish to cmd_vel
        self.p = rospy.Publisher('cmd_vel', Twist)

        # service for adding a request
        rospy.Service('~AddRequest', AddRequest, self.requestCb)

        rospy.loginfo('courier_node initialized.')
    
        # main loop
        while not rospy.is_shutdown():
            # process a request
            if len(self.schedule) > 0:
                # TODO: should probably add a mutex            
                pickup = self.schedule[0][0]         
                dropoff = self.schedule[0][1]
                # pop the request
                self.schedule = self.schedule[1:]

                # head to pickup destination
                self.driveTo(pickup[0], pickup[1], pickup[2])
                # wait for button press                
                self.waitForButton()

                # head to dropoff destination
                self.driveTo(dropoff[0], dropoff[1], dropoff[2])
                # wait for button press                
                self.waitForButton()

            else:
                rospy.sleep(1)                    

    def getPosition(self, frame="/map"):
        """ Get the robot's position in some frame. """
        try:
            ((x,y,z), rot) = self.listener.lookupTransform(frame, 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to get position of robot")
        (phi, psi, theta) = tf.transformations.euler_from_quaternion(rot)
        return [x,y,theta]

    def driveTo(self,x,y,angle):
        """ Drive to a desired position in frame. """
        r = rospy.Rate(10)

        while True:       
            # get current Position
            (xCurrent, yCurrent, tCurrent) = self.getPosition()

            # have we reached goal?
            if abs(x-xCurrent) < self.xy_tolerance and abs(y-yCurrent) < self.xy_tolerance:
                break

            # compute error angle
            tDesired = math.atan2(y-yCurrent, x-xCurrent)

            # turn in place, because we are more than 45 degrees off 
            if abs(tDesired - tCurrent)>0.7:
                twist = Twist()
                if tDesired - tCurrent > 3.14:
                    twist.angular.z = -1.57/2
                elif tDesired - tCurrent > 0:
                    twist.angular.z = 1.57/2
                elif tDesired - tCurrent < - 3.14:
                    twist.angular.z = 1.57/2
                else:
                    twist.angular.z = -1.57/2
                self.p.publish(twist)
                # wait until turn has finished
                while( abs(tCurrent - tDesired) > self.yaw_tolerance):
                    (xCurrent, yCurrent, tCurrent) = self.getPosition()
                        
            # make progress towards goal
            twist=Twist()
            twist.linear.x = 0.15
            twist.angular.z = (tDesired - tCurrent)/2.0
            self.p.publish(twist)

            # sleep a bit
            r.sleep()
        
        # reached xy goal, now correct heading by turning in place
        if abs(tDesired - tCurrent) > self.yaw_tolerance:
            twist = Twist()
            if tDesired - tCurrent > 3.14:
                twist.angular.z = -1.57/2
            elif tDesired - tCurrent > 0:
                twist.angular.z = 1.57/2
            elif tDesired - tCurrent < - 3.14:
                twist.angular.z = 1.57/2
            else:
                twist.angular.z = -1.57/2
            self.p.publish(twist)
            # wait until turn has finished
            while( abs(tCurrent - tDesired) > self.yaw_tolerance):
                (xCurrent, yCurrent, tCurrent) = self.getPosition()
        
        # done - STOP!
        twist=Twist()
        self.p.publish(twist)

    def cleanup(self):
        """ Stop the robot, in case we're exiting prematurely. """
        twist = Twist()
        self.p.publish(twist)

    def requestCb(self, req):
        """ Process a request. """
        p1 = self.lookupCoordinates(req.PickupLocation)
        p2 = self.lookupCoordinates(req.DropoffLocation)

        # TODO: more advanced planning here?

        if p1 != None and p2 != None:
            self.schedule.append( [p1,p2] )
            return AddRequestResponse(5*len(self.schedule))
        else:
            return AddRequestResponse(-1)

    def lookupCoordinates(self, location):
        if location == "Nick's Office":
            return [4.47, 0.71, 0]
        elif location == "Lynne's Office":
            return [4.47, 4.0, 0]
    
        # TODO: add more locations here

        else:
            return None

    def waitForButton(self):
        """ Wait until button pressed. """
        while self.buttonValue == 255:
            pass
        
    def buttonCb(self, msg):
        self.buttonValue = msg.value

if __name__ == '__main__':
    try:
        CourierServer()
    except rospy.ROSInterruptException:
        rospy.loginfo("And that's all folks...")

