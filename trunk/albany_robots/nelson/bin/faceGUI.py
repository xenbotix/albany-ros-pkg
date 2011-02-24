#!/usr/bin/env python

""" Simple virual face
    Michael E. Ferguson, 2009-2010. """

import roslib; roslib.load_manifest('nelson')
import rospy

from sensor_msgs.msg import JointState
from std_srvs.srv import *

# TODO: move this to WX
import pygame
from pygame import *

# colors
botBody = (0,105,30)
botSide = (0,125,40)
black = (0,0,0)
yellow = (255,255,0)
red = (255,0,0)
muted = (110, 110, 0)
mouth = (100,100,100)

class virtualFace:

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((600,500))    
        self.eyecolor = black
        self.joints = {"head_pan_joint":0.0, "head_tilt_joint":0.0, "eye_tilt":0.0, "r_eye_pan":0.0, "l_eye_pan":0.0, 
                       "eyelids":0.0, "top_lip":0.0, "bottom_lip":0.0, "r_eyebrow":0.0, "l_eyebrow":0.0 }

    def angry(self, data = None):
        self.eyecolor = red
        return EmptyResponse()

    def happy(self, data = None):
        self.eyecolor = yellow
        return EmptyResponse()
        
    def drawFace(self): #, eX, eY, sMouth): #rotate, lift, eyesRotate, eyesLift, mouth):  
        # screen props
        cX = 300
        cY = 250
        # size of face
        w = 400
        h = 400
            
        # eyes come in as -1 to 1   
        eL = int(self.joints["l_eye_pan"] * w/10)
        eR = int(self.joints["r_eye_pan"] * w/10)
        eY = int(self.joints["eye_tilt"] * w/10)
        # mouth is between 0 and 1
        sMouth = 50 * (self.joints["top_lip"] - self.joints["bottom_lip"]) #25 
        
        # body of head -- make this 3D-ish, and neck        
        pygame.draw.rect(self.screen, botSide, (cX - w/6, cY,w/3,cY) )
        pygame.draw.rect(self.screen, botBody, (cX - w/2, cY - h/2,w,h) )
            
        # eyes
        pygame.draw.ellipse(self.screen, black, (cX - w/4 - w/6, cY - (5*h/14), w/3, 3*h/7))
        pygame.draw.ellipse(self.screen, black, (cX + w/4 - w/6, cY - (5*h/14), w/3, 3*h/7))
        pygame.draw.ellipse(self.screen, self.eyecolor, (cX-w/4+eL-w/20, cY-(2*h/14)-eY-w/16,w/10,w/8))
        pygame.draw.ellipse(self.screen, self.eyecolor, (cX+w/4+eR-w/20, cY-(2*h/14)-eY-w/16,w/10,w/8))
         
        # mouth
        pygame.draw.line(self.screen, mouth, (cX-w/2, cY+w/4), (cX-w/2+3*w/11,cY+w/4), 4)
        pygame.draw.line(self.screen, mouth, (cX-w/2+4*w/11, cY+w/4), (cX-w/2+5*w/11,cY+w/4), 4)
        pygame.draw.line(self.screen, mouth, (cX-w/2+6*w/11, cY+w/4), (cX-w/2+7*w/11,cY+w/4), 4)
        pygame.draw.line(self.screen, mouth, (cX-w/2+8*w/11, cY+w/4), (cX+w/2-1,cY+w/4), 4)
 
        pygame.draw.line(self.screen, mouth, (cX-w/2+3*w/11, cY+w/4+sMouth), (cX-w/2+4*w/11,cY+w/4+sMouth), 4)
        pygame.draw.line(self.screen, mouth, (cX-w/2+5*w/11, cY+w/4+sMouth), (cX-w/2+6*w/11,cY+w/4+sMouth), 4)
        pygame.draw.line(self.screen, mouth, (cX-w/2+7*w/11, cY+w/4+sMouth), (cX-w/2+8*w/11,cY+w/4+sMouth), 4)

        for i in range(6):
            pygame.draw.line(self.screen, mouth, (cX-w/2+(i+3)*w/11, cY+w/4), (cX-w/2+(i+3)*w/11, cY+w/4+sMouth), 4)

        # hair
        for i in range(15):
            pygame.draw.arc(self.screen, muted, ((cX-w/3+(i*w/30),cY-w/2-50),(30,100)), 0, 1.5, 7)

        pygame.display.flip()
    
        # publish current joint positions
        j = JointState()
        for joint in self.joints.keys():
            j.name.append(joint)
            j.position.append(self.joints[joint])
        self.pub.publish(j)

    def cmdJointsCb(self, msg):
        for joint in msg.name:
            self.joints[joint] = msg.position[msg.name.index(joint)]

if __name__ == "__main__":
    # begin initialization    
    rospy.init_node('virtualface')
    face = virtualFace()   

    # we subscribe to joint_states to display the face
    rospy.Subscriber('cmd_joints', JointState, face.cmdJointsCb)
    # we have to publish our joint_states too
    face.pub = rospy.Publisher('joint_states', JointState)
    # simple, stupid services -- these need to go away
    rospy.Service('setHappy',Empty,face.happy)   
    rospy.Service('setAngry',Empty,face.angry)
   
    # logging information is a good idea
    rospy.loginfo('faceGUI.py initialized')

    # Let this run at approximately 30hz
    # rospy.Rate handles all timing for us
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        try:       
            for event in pygame.event.get():
                if event.type is pygame.QUIT:             
                    rospy.signal_shutdown("I quit!")
            #    elif event.type is pygame.MOUSEBUTTONDOWN:
            #        if face.eyecolor == black or face.eyecolor == red:  
            #            face.eyecolor = yellow
            #            eyepos = 0.5
            #        else:
            #            eyepos = 0
            #            face.eyecolor = red
            face.drawFace() #eyepos,0,1)                
            pygame.display.flip()
        except KeyboardInterrupt:
            pass        
        r.sleep()

