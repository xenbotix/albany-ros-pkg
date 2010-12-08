#!/usr/bin/env python
# python based face for a Asus/Create

import pygame
from pygame import *
import socket
import sys
import time

# colors
botBody = (0,105,30)
botSide = (0,125,40)
black = (0,0,0)
yellow = (255,255,0)
red = (255,0,0)
muted = (110, 110, 0)
mouth = (100,100,100)

class pyFace:

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((600,500))    
        self.eyecolor = black

    def angry(self):
        self.eyecolor = red
    def happy(self):
        self.eyecolor = yellow
        
    def drawFace(self, eX, eY, sMouth): #rotate, lift, eyesRotate, eyesLift, mouth):  
        # screen props
        cX = 300
        cY = 250
        # size of face
        w = 400
        h = 400
            
        # eyes come in as -1 to 1   
        eX = int(eX * w/10)
        eY = int(eY * w/10)
        # mouth is between 0 and 1
        sMouth = int(sMouth * w/10)
        
        # body of head -- make this 3D-ish, and neck        
        pygame.draw.rect(self.screen, botSide, (cX - w/6, cY,w/3,cY) )
        pygame.draw.rect(self.screen, botBody, (cX - w/2, cY - h/2,w,h) )
            
        # eyes
        pygame.draw.ellipse(self.screen, black, (cX - w/4 - w/6, cY - (5*h/14), w/3, 3*h/7))
        pygame.draw.ellipse(self.screen, black, (cX + w/4 - w/6, cY - (5*h/14), w/3, 3*h/7))
        pygame.draw.ellipse(self.screen, self.eyecolor, (cX-w/4+eX-w/20, cY-(2*h/14)-eY-w/16,w/10,w/8))
        pygame.draw.ellipse(self.screen, self.eyecolor, (cX+w/4+eX-w/20, cY-(2*h/14)-eY-w/16,w/10,w/8))
         
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


if __name__ == "__main__":
    face = pyFace() 
    #face.angry()

    tekkotsu = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tekkotsu.connect((sys.argv[1],10001))
    eyepos = 0
    print "Connected"

    while True:
        try:       
            for event in pygame.event.get():
                if event.type is pygame.QUIT:             
                    sys.exit()
                elif event.type is pygame.MOUSEBUTTONDOWN:
                    if face.eyecolor == black or face.eyecolor == red:  
                        face.eyecolor = yellow
                        eyepos = 0.5
                        tekkotsu.send("!msg Nice\n")
                    else:
                        eyepos = 0
                        face.eyecolor = red
                        tekkotsu.send("!msg Random\n")
            face.drawFace(eyepos,0,1)                
            pygame.display.flip()
            #print tekkotsu.recv(4096)
            time.sleep(0.01)
        except KeyboardInterrupt:
            sys.exit(0)
    
    
