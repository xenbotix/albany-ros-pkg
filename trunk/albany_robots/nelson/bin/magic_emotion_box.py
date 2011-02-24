#!/usr/bin/env python

""" The Magic Emotion Box """

import roslib; roslib.load_manifest('nelson')
import rospy

import wx
from social_msgs.msg import AffectiveState
from sensor_msgs.msg import JointState

class Pose:
    """ Class to hold a single pose. """
    def __init__(self, string):
        s = string.split(",")        
        self.name = s[0].split(":")[0]
        self.x = float(s[0].split(":")[1])
        self.y = float(s[1])
        self.data = dict()
        for x in s[2:]:
            x = x.strip().rstrip().strip("{}").split(":")
            self.data[x[0]] = int(x[1])

    def distance(self, x, y):
        """ Find distance from current valence (x) and activation (y). """
        return (self.x-x)*(self.x-x)+(self.y-y)*(self.y-y)

class MagicEmotionController:
    """ Class to hold poses, compute mixed poses. """
    def __init__(self, posefile):
        self.poses = dict()
        for line in open(posefile).readlines():
            pose = Pose(line)
            self.poses[pose.name] = pose

    def compute(self, valence, activation):
        """ Compute the pose for a given valence and activation. """
        new_pose = dict()
        # compute distances
        dist_pose = dict()
        for p in self.poses.keys():
            dist_pose[self.poses[p].distance(valence, activation)] = p
        distances = sorted(dist_pose.keys())
        if abs(distances[0]) < 0.01:
            # on a pose exactly, no need for multiple contributions
            for servo, value in self.poses[dist_pose[distances[0]]].data.items():
                try:
                    new_pose[servo] = self.poses["Neutral"].data[servo]
                except:
                     new_pose[servo] = 512 + value
        else:            
            normal = sum([ 1/(x) for x in distances[0:3] ])
            k = 0
            # compose pose
            for idx in range(3):
                distance = distances[idx]
                posename = dist_pose[distance]
                score = (1/(distance))/normal
                k += score
                print score, posename, 
                for servo, value in self.poses[posename].data.items():
                    try:
                        new_pose[servo] += value * score
                    except:        
                        try:
                            new_pose[servo] = self.poses["Neutral"].data[servo] + score
                        except:
                            new_pose[servo] = 512 + value * score
            print k
        return new_pose

class MagicEmotionBox:
    def __init__(self): 
        self.valence = 0.0
        self.activation = 0.0
        rospy.init_node("magic_emotion_box")
        self.controller = MagicEmotionController(rospy.get_param("~poses"))
        rospy.Subscriber("affective_state", AffectiveState, self.affectCb)
        self.pub = rospy.Publisher("face_controller/command", JointState)
        rospy.spin()

    def affectCb(self, msg):
        # process messages
        if msg.scale > 0:
            self.valence = msg.valence
            self.activation = msg.activation
        else:
            return
        #    self.valence += msg.valence    TODO: difference updates
        #    self.activation += msg.activation
        pose = self.controller.compute(self.valence, self.activation)
        # and publish a joint states message
        msg = JointState()
        msg.name = list()
        msg.position = list()        
        for servo in ["eye_tilt", "r_eye_pan", "l_eye_pan", "eyelids", "top_lip", "bottom_lip", "r_eyebrow", "l_eyebrow"]:
        #for servo, value in pose.items():
            msg.name.append(servo)  
            try:
                msg.position.append( (pose[servo]-512)*(5.2359877/1024.0) ) # 5.23 is rad(300)
            except:
                msg.position.append(0)
            msg.velocity.append(0.0)
        self.pub.publish(msg)


if __name__=="__main__":
    MagicEmotionBox()

