#!/usr/bin/env python

""" A Simple GUI Controller for the Magic Emotion Box,
    based on testEmotion.java by Ken Stahl. """

import roslib; roslib.load_manifest('nelson')
import rospy

import wx
from social_msgs.msg import AffectiveState

class emotionController(wx.Frame):
    TIMER_ID = 100
    width = 500.0
    height = 300.0

    # Text: [x of dot, y of dot, x offset of text, y offset]
    points = { "Frustrated": [0,0,0,0],
               "Useless": [0,1,0,1],
               "Confused": [1/6.0,0.5,0.5,0.5],
               "Impatient": [2/6.0,0.25,0.5,0.5],
               "Apathetic": [2/6.0,0.75,0.5,0.5],

               "Overstimulated": [0.5,0,0.5,0],
               "Neutral": [0.5,0.5,0.5,0.5],
               "Understimulated": [0.5,1,0.5,1],
               
               "Curious": [4/6.0,0.25,0.5,0.5],
               "Patient": [4/6.0,0.75,0.5,0.5],
               "Informed": [5/6.0,0.5,0.5,0.5],
               "Helpful": [1,0,1,0],
               "Content": [1,1,1,1],
             }

    def __init__(self): 
        wx.Frame.__init__(self, None, -1, "Emotion Controller GUI", wx.Point(100,50), wx.Size(self.width,self.height),style = wx.DEFAULT_FRAME_STYLE)
        self.rate = rospy.get_param("~use_sync",10.0)

        # timer for output
        self.pos = wx.Point(self.width/2,self.height/2)
        self.timer = wx.Timer(self, self.TIMER_ID)
        self.timer.Start(1000/self.rate)
        wx.EVT_CLOSE(self, self.closeCB)
        wx.EVT_TIMER(self, self.TIMER_ID, self.timerCB)

        # bind the panel to the paint event
        wx.EVT_PAINT(self, self.paintCB)
        self.dirty = 1
        self.paintCB()

        self.SetBackgroundColour("E7E7E7")
        self.Show(True)

        # create publisher and allow mouse callbacks    
        self.p = rospy.Publisher('affective_state',AffectiveState)
        self.Bind(wx.EVT_MOUSE_EVENTS, self.clickCB)  

    def closeCB(self, event):
        self.timer.Stop()
        self.Destroy()

    def paintCB(self, event=None):
        # this is the wx drawing surface/canvas
        dc = wx.PaintDC(self)
        dc.SetBackground(wx.Brush("#E7E7E7")) #wx.SystemSettings.GetColour(wx.SYS_COLOUR_HIGHLIGHT))) #wx.Brush("Grey"))
        dc.Clear()
        self.width, self.height = self.GetSizeTuple()
        
        # first, laydown points
        dc.SetPen(wx.Pen("orange",2))
        dc.SetBrush(wx.Brush('orange', wx.SOLID))
        for p in self.points.values():
            dc.DrawCircle(p[0]*self.width,p[1]*self.height,5)

        # now text
        dc.SetPen(wx.Pen("black",2))
        for p in self.points.keys():
            x = self.points[p][0]*self.width
            y = self.points[p][1]*self.height
            xo = self.points[p][2]
            yo = self.points[p][3]
            tx, ty = dc.GetTextExtent(p)
            dc.DrawText(p,x-(tx*xo),y-(ty*yo))

        dc.SetBrush(wx.Brush('green', wx.SOLID))
        dc.SetPen(wx.Pen("black",2))
        dc.DrawCircle(self.pos.x,self.pos.y, 5)  

    def clickCB(self, event):
        # update internal position
        if event.LeftIsDown():  
            # TODO: Cap -1 to 1
            self.pos = event.GetPosition() 
            print (2.0*self.pos.x - self.width)/self.width, (self.height - 2.0*self.pos.y)/self.height
        self.paintCB()          

    def timerCB(self, event):
        # send message update
        msg = AffectiveState()
        msg.valence = (2.0*self.pos.x - self.width)/self.width 
        msg.activation = (self.height- 2.0*self.pos.y)/self.height
        msg.scale = 1.0
        self.p.publish(msg)
        
if __name__ == '__main__':
    # initialize GUI
    rospy.init_node('EmotionController')
    app = wx.PySimpleApp()
    frame = emotionController()
    app.MainLoop()

