#!/usr/bin/env python

""" 
Minimal SSC32 driver for Chiara
Michael E. Ferguson
University at Albany
"""

import roslib; roslib.load_manifest('chiara')
import rospy

from std_msgs.msg import Bool

import serial, thread

GREEN = "12"
RED = "13"
YELLOW = "14"
BLUE = "15"

ON = "H"
OFF = "L"

class ssc32:    
    def __init__(self, port="/dev/ttyS0",baud=115200):
        self.mutex = thread.allocate_lock()
        self.ser = serial.Serial()
        
        self.mutex.acquire()
        self.ser.baudrate = baud
        self.ser.port = port
        self.ser.timeout = 0.1
        self.ser.open()
        self.mutex.release()

    def setLed(self, led, value):
        """ Ex: setLed(GREEN, ON) """
        self.mutex.acquire()
        self.ser.write("#"+led+value+"\n")
        self.mutex.release()

    def getButtons(self):
        """
          Chiara button mappings:
            A = Green
            B = Red
            C = Yellow
        """
        self.mutex.acquire()
        try:      
            self.ser.flushInput()
        except:
            pass
        self.ser.write("AL BL CL\n")
        b = [ int(self.ser.read()) for i in range(3) ]
        self.mutex.release()
        return b
        
if __name__=="__main__":
    rospy.init_node("ssc32_chiara") 
    ssc = ssc32()

    # button publishers    
    gr_button = rospy.Publisher("~green_button", Bool)
    rd_button = rospy.Publisher("~red_button", Bool)
    yl_button = rospy.Publisher("~yellow_button", Bool)

    # led subscribers
    # TODO

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        # publish buttons
        b = ssc.getButtons()
        gr_button.publish(Bool(b[0]==1))
        if b[0] == 1:
            ssc.setLed(GREEN,ON)
        rd_button.publish(Bool(b[1]==1))
        if b[0] == 1:
            ssc.setLed(BLUE,ON)
        yl_button.publish(Bool(b[2]==1))
        if b[0] == 1:
            ssc.setLed(RED,ON)
        r.sleep()

