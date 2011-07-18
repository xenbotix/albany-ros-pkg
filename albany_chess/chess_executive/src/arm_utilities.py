#!/usr/bin/env python

""" 
  Copyright (c) 2011 Michael E. Ferguson.  All right reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import roslib; roslib.load_manifest('chess_executive')
import pexpect

import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from chess_msgs.msg import *
from simple_arm_server.msg import *
from simple_arm_server.srv import * 

from chess_utilities import SQUARE_SIZE
from tuck_arm import *

class ArmPlanner:
    """ Connection to the arm server. """
    SQUARE_SIZE = 0.05715
    
    def __init__(self, srv=None):
        if srv==None:
            rospy.wait_for_service('simple_arm_server/move')
            self.move = rospy.ServiceProxy('simple_arm_server/move', MoveArm) 
        else:
            self.move = srv
        self.tuck_server = tuck_arm()

    def execute(self, move, board):
        """ Execute a move. """

        # untuck arm
        self.tuck_server.untuck()
        rospy.sleep(3)

        # get info about move        
        (col_f, rank_f) = board.toPosition(move[0:2])
        (col_t, rank_t) = board.toPosition(move[2:])
        fr = board.getPiece(col_f, rank_f)
        to = board.getPiece(col_t, rank_t)

        req = MoveArmRequest()  
        req.header.frame_id = fr.header.frame_id

        # is this a capture?
        if to != None: 
            #self.addTransit(req, to.pose, off_board)
            pass
        
        to = ChessPiece()
        to.header.frame_id = fr.header.frame_id
        if board.side == board.WHITE:
            to.pose.position.x = (col_t * SQUARE_SIZE) + SQUARE_SIZE/2
            to.pose.position.y = ((rank_t-1) * SQUARE_SIZE) + SQUARE_SIZE/2
            to.pose.position.z = fr.pose.position.z
        else:
            to.pose.position.x = ((7-col_t) * SQUARE_SIZE) + SQUARE_SIZE/2
            to.pose.position.y = ((8-rank_t) * SQUARE_SIZE) + SQUARE_SIZE/2
            to.pose.position.z = fr.pose.position.z

        self.addTransit(req, fr.pose, to.pose)
        
        # execute
        try:
            r = self.move(req)
            print r
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)

        # tuck arm
        self.tuck_server.tuck()
        rospy.sleep(5.0)
        return to.pose


    def addTransit(self, req, fr, to):
        """ Move a piece from 'fr' to 'to' """
        # hover over piece
        action = ArmAction()
        action.type = ArmAction.MOVE_ARM
        action.goal.position.x = fr.position.x
        action.goal.position.y = fr.position.y
        action.goal.position.z = fr.position.z + 0.1
        q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
        action.goal.orientation.x = q[0]
        action.goal.orientation.y = q[1]
        action.goal.orientation.z = q[2]
        action.goal.orientation.w = q[3]
        action.move_time = rospy.Duration(5.0)
        req.goals.append(action)

        # open gripper
        action = ArmAction()
        action.type = ArmAction.MOVE_GRIPPER
        action.command = 0.05
        action.move_time = rospy.Duration(1.0)
        req.goals.append(action)

        # lower gripper
        action = ArmAction()
        action.type = ArmAction.MOVE_ARM
        action.goal.position.x = fr.position.x
        action.goal.position.y = fr.position.y
        action.goal.position.z = fr.position.z + 0.03
        q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
        action.goal.orientation.x = q[0]
        action.goal.orientation.y = q[1]
        action.goal.orientation.z = q[2]
        action.goal.orientation.w = q[3]
        action.move_time = rospy.Duration(5.0)
        req.goals.append(action)

        # close gripper
        action = ArmAction()
        action.type = ArmAction.MOVE_GRIPPER
        action.command = 0.01
        action.move_time = rospy.Duration(1.0)
        req.goals.append(action)

        # raise gripper
        action = ArmAction()
        action.type = ArmAction.MOVE_ARM
        action.goal.position.x = fr.position.x
        action.goal.position.y = fr.position.y
        action.goal.position.z = fr.position.z + 0.1
        q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
        action.goal.orientation.x = q[0]
        action.goal.orientation.y = q[1]
        action.goal.orientation.z = q[2]
        action.goal.orientation.w = q[3]
        action.move_time = rospy.Duration(5.0)
        req.goals.append(action)

        # over over goal
        action = ArmAction()
        action.type = ArmAction.MOVE_ARM
        action.goal.position.x = to.position.x
        action.goal.position.y = to.position.y
        action.goal.position.z = to.position.z + 0.1
        q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
        action.goal.orientation.x = q[0]
        action.goal.orientation.y = q[1]
        action.goal.orientation.z = q[2]
        action.goal.orientation.w = q[3]
        action.move_time = rospy.Duration(5.0)
        req.goals.append(action)

        # lower gripper
        action = ArmAction()
        action.type = ArmAction.MOVE_ARM
        action.goal.position.x = to.position.x
        action.goal.position.y = to.position.y
        action.goal.position.z = to.position.z + 0.03
        q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
        action.goal.orientation.x = q[0]
        action.goal.orientation.y = q[1]
        action.goal.orientation.z = q[2]
        action.goal.orientation.w = q[3]
        action.move_time = rospy.Duration(5.0)
        req.goals.append(action)
        
        # open gripper
        action = ArmAction()
        action.type = ArmAction.MOVE_GRIPPER
        action.command = 0.05
        action.move_time = rospy.Duration(1.0)
        req.goals.append(action)
        
        # raise gripper
        action = ArmAction()
        action.type = ArmAction.MOVE_ARM
        action.goal.position.x = to.position.x
        action.goal.position.y = to.position.y
        action.goal.position.z = to.position.z + 0.1
        q = quaternion_from_euler(0.0, 1.57, 0.0, 'sxyz')
        action.goal.orientation.x = q[0]
        action.goal.orientation.y = q[1]
        action.goal.orientation.z = q[2]
        action.goal.orientation.w = q[3]
        action.move_time = rospy.Duration(5.0)
        req.goals.append(action)

