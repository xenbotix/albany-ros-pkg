#!/usr/bin/env python

""" A Controller GUI for direct control of expression-based faces.
        Michael E. Ferguson, 2010. """

import roslib; roslib.load_manifest('albany_gui')
import rospy
import wx

from sensor_msgs.msg import JointState

width = 300

class faceControllerGUI(wx.Frame):
    TIMER_ID = 100

    def __init__(self, parent, debug = False):  
        wx.Frame.__init__(self, parent, -1, "Face Controller GUI", style = wx.DEFAULT_FRAME_STYLE & ~ (wx.RESIZE_BORDER | wx.MAXIMIZE_BOX))
        
        sizer = wx.GridBagSizer(10,10)

        # Move Head
        head = wx.StaticBox(self, -1, 'Head Movement')
        head.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        headBox = wx.StaticBoxSizer(head,orient=wx.VERTICAL) 
        self.movehead = wx.Panel(self,size=(width,width-20))
        self.movehead.SetBackgroundColour('WHITE')
        self.movehead.Bind(wx.EVT_MOTION, self.onMove)  
        wx.StaticLine(self.movehead, -1, (width/2, 0), (1,width), style=wx.LI_VERTICAL)
        wx.StaticLine(self.movehead, -1, (0, width/2), (width,1))
        headBox.Add(self.movehead)    
        sizer.Add(headBox,(0,0),wx.GBSpan(2,1),wx.EXPAND|wx.TOP|wx.LEFT|wx.BOTTOM,5)
        self.pan = 0
        self.tilt = 0       

        # Eye Control
        eyes = wx.StaticBox(self, -1, 'Eye Movement')
        eyes.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        eyesBox = wx.StaticBoxSizer(eyes,orient=wx.VERTICAL) 
        eyesSizer = wx.GridBagSizer(5,5)

        eyesSizer.Add(wx.StaticText(self, -1, "Eye Tilt:"),(0,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.eye_tilt = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        eyesSizer.Add(self.eye_tilt,(0,1))

        eyesSizer.Add(wx.StaticText(self, -1, "Right Eye Pan:"),(1,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.r_eye_pan = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        eyesSizer.Add(self.r_eye_pan,(1,1))
        eyesSizer.Add(wx.StaticText(self, -1, "Left Eye Pan:"),(2,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.l_eye_pan = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        eyesSizer.Add(self.l_eye_pan,(2,1))
        eyesSizer.Add(wx.StaticText(self, -1, "Right Eyebrow:"),(3,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.l_eyebrow = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        eyesSizer.Add(self.l_eyebrow,(3,1))
        eyesSizer.Add(wx.StaticText(self, -1, "Left Eyebrow:"),(4,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.r_eyebrow = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        eyesSizer.Add(self.r_eyebrow,(4,1))
        eyesBox.Add(eyesSizer) 
        
        sizer.Add(eyesBox, (0,1), wx.GBSpan(1,1), wx.EXPAND|wx.TOP|wx.RIGHT,5)

        # Mouth
        mouth = wx.StaticBox(self, -1, 'Mouth Movement')
        mouth.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD))
        mouthBox = wx.StaticBoxSizer(mouth,orient=wx.VERTICAL) 
        mouthSizer = wx.GridBagSizer(5,5)

        mouthSizer.Add(wx.StaticText(self, -1, "Top Lip:"),(0,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.top_lip = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        mouthSizer.Add(self.top_lip,(0,1))

        mouthSizer.Add(wx.StaticText(self, -1, "Bottom Lip:"),(1,0), wx.GBSpan(1,1),wx.ALIGN_CENTER_VERTICAL)
        self.bot_lip = wx.Slider(self, -1, 0, -100, 100, wx.DefaultPosition, (200, -1), wx.SL_HORIZONTAL | wx.SL_LABELS)
        mouthSizer.Add(self.bot_lip,(1,1))
        mouthBox.Add(mouthSizer) 
        
        sizer.Add(mouthBox, (1,1), wx.GBSpan(1,1), wx.EXPAND|wx.BOTTOM|wx.RIGHT,5)

        # timer for output
        self.timer = wx.Timer(self, self.TIMER_ID)
        self.timer.Start(100)
        wx.EVT_CLOSE(self, self.onClose)
        wx.EVT_TIMER(self, self.TIMER_ID, self.onTimer)

        self.SetSizerAndFit(sizer)
        self.Show(True)

        #self.panService = rospy.ServiceProxy('head_pan_setangle',SetAngle)
        self.pub = rospy.Publisher('cmd_joints', JointState)

    def onClose(self, event):
        self.timer.Stop()
        self.Destroy()

    def onMove(self, event=None):
        if event.LeftIsDown():        
            pt = event.GetPosition()
            self.tilt = ((width/2)-pt[1])/2
            self.pan = (pt[0]-(width/2))/2           
        else:
            self.tilt = 0
            self.pan = 0
            pass

    def onTimer(self, event=None):
        # send updates
        j = JointState()
        j.name = ["head_pan", "head_tilt", "eye_tilt", "r_eye_pan", "l_eye_pan", 
                       "eyelids", "top_lip", "bottom_lip", "r_eyebrow", "l_eyebrow"]
        j.position = [self.pan/100.0, self.tilt/100.0, self.eye_tilt.GetValue()/100.0, self.r_eye_pan.GetValue()/100.0, self.l_eye_pan.GetValue()/100.0,
                        0, self.top_lip.GetValue()/100.0, self.bot_lip.GetValue()/100.0, self.r_eyebrow.GetValue()/100.0, self.l_eyebrow.GetValue()/100.0]
        self.pub.publish(j)
        
if __name__ == '__main__':
    # initialize and listen to joint_states topic
    rospy.init_node('face_controller_GUI')

    app = wx.PySimpleApp()
    frame = faceControllerGUI(None, True)
    app.MainLoop()

