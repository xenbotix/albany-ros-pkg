#!/usr/bin/env python

""" 
  Simple executive for playing AAAI robot chess
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
import rospy

from chess_msgs.msg import *
from std_srvs.srv import *

from chess_utilities import *
from arm_utilities import *

###############################################################################
# Executive for managing chess game

class ChessExecutive:
    def __init__(self):
        """ Start the executive node """
        rospy.init_node("chess_executive")

        # connect to camera_turnpike service
        rospy.loginfo('exec: Waiting for /camera_turnpike/trigger')
        rospy.wait_for_service('/camera_turnpike/trigger')
        self.trigger = rospy.ServiceProxy('/camera_turnpike/trigger', Empty)

        # get arm planner TODO
        rospy.loginfo('exec: Waiting for /simple_arm_server/move')
        rospy.wait_for_service('/simple_arm_server/move')
        self.planner = ArmPlanner( rospy.ServiceProxy('/simple_arm_server/move', MoveArm) )

        # subscribe to input
        self.board = BoardState()
        rospy.Subscriber('/extract_pieces/output', ChessBoard, self.board.applyUpdate) 

        # subscribe to your move services
        self.yourMove = self.yourMoveKeyboard
        #self.yourMove = self.yourMovePerception

        rospy.loginfo("exec: Done initializing...")


    ###########################################################################
    # your move prototypes

    def yourMoveKeyboard(self):
        # stupid little function to know when move has been made
        print "Please press enter after making a move"
        x= raw_input()        
        if x.rstrip() == "exit":
            self.engine.exit()
            exit()            

    def yourMovePerception(self):
        # 
        pass

    ###########################################################################
    # game playing

    def playGame(self):
        """ This function plays a complete game. """

        # default board representation
        self.engine = GnuChessEngine()
        
        # are we white/black?
        self.board.newGame()
        self.board.up_to_date = False        
        rospy.loginfo("exec: Calibrate board side...")
        self.trigger()
        while not rospy.is_shutdown():
            if self.board.up_to_date:
                # can't used update fx, as "none" is acceptable
                if self.board.last_move == "fail":
                    self.board.up_to_date = False 
                    rospy.loginfo("exec: Triggering again to calibrate board side...")
                    self.trigger()
                else:
                    self.board.computeSide()
                    self.board.newGame()
                    break

        if self.board.side == self.board.BLACK:
            # wait for opponents move
            self.yourMove()
            # update board state
            self.updateBoardState()
        
        # loop! TODO: termination?
        while True: 
            # do move
            move = self.engine.nextMove(self.board.last_move)
            while move == None:
                # update board state    
                rospy.loginfo("exec: Bad move, triggering again...")
                self.updateBoardState()
                move = self.engine.nextMove(self.board.last_move)
            # do move
            rospy.loginfo("exec: My move: %s", move)
            self.board.applyMove(move, self.planner.execute(move,self.board)) 

            # wait for opponents move
            self.yourMove()

            # update board state
            self.updateBoardState()
    
    def updateBoardState(self):
        """ Updates board state by triggering pipeline. """
        self.board.up_to_date = False
        rospy.loginfo("exec: Triggering...")
        self.trigger()
        while not rospy.is_shutdown():
            if self.board.up_to_date:
                if self.board.last_move == "fail" or self.board.last_move == "none":
                    self.board.up_to_date = False 
                    rospy.loginfo("exec: Triggering again...")   
                    self.trigger()
                else:
                    break


if __name__=="__main__":
    try:
        executive = ChessExecutive()
        executive.playGame()
    except KeyboardInterrupt:
        pass

