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

SQUARE_SIZE = 0.05715


###############################################################################
# Executive for managing chess game

class ChessExecutive:
    def __init__(self):
        # start node
        rospy.init_node("chess_executive")

        # connect to camera_turnpike service
        rospy.wait_for_service('/camera_turnpike/trigger')
        self.trigger = rospy.ServiceProxy('/camera_turnpike/trigger', Empty)

        # get arm planner
        #rospy.wait_for_service('/chess_arm_server/move_piece')
        #self.planner = rospy.ServiceProxy('/chess_arm_server/move_piece', MovePiece)

        # subscribe to input
        self.updated = False
        rospy.Subscriber('/extract_pieces/output', ChessBoard, self.boardCb) 

        rospy.loginfo("Done initializing...")


    ###########################################################################
    # your move prototypes

    def yourMoveKeyboard(self):
        # stupid little function to know when move has been made
        print "Please press enter after making a move"
        raw_input()        


    ###########################################################################
    # game playing

    def playGame(self):
        """ This function plays a complete game. """

        # wait for readiness signal
        #print "Press enter when ready"
        #reply = raw_input()

        # default board representation
        self.board = BoardState()
        self.board.newGame()
        self.planner = GnuChessPlanner()
        
        # are we white/black?
        #self.updated = False
        #rospy.loginfo("Triggering...")
        #self.trigger()
        #while self.updated == False:
        #    pass
        
        # TODO: if white, send "go" to GNU
        #if True:
        #    self.engine.sendline('go')
        #    self.myMove()

        # loop! TODO: termination?
        while True:        
            # wait
            self.yourMoveKeyboard()
            # update board state
            self.updated = False
            rospy.loginfo("Triggering...")
            self.trigger()
            while self.updated == False:
                pass
            # move
            #self.myMove()

    

    def boardCb(self, msg):
        
        rospy.loginfo("Message recieved")

        loc_from = None
        loc_to = None   
        move_cnt = 0
        # process ChessBoard message  
        new_board = BoardState()  
        for piece in msg.pieces:
            row = int(piece.pose.position.y/SQUARE_SIZE + SQUARE_SIZE/2)
            col = int(piece.pose.position.x/SQUARE_SIZE + SQUARE_SIZE/2)
            if row >= 8 or col >= 8 or row < 0 or col < 0:
                continue
            if new_board.getPiece(row, col) == None:
                p = self.board.getPiece(row, col)
                if p == None:
                    move_cnt = move_cnt + 1
                    if loc_to == None:
                        loc_to = [row, col, p] 
                    rospy.loginfo("Piece moved to: %d, %d" % (row, col))
                else:
                    piece.type = p.type
                new_board.setPiece(row, col, piece)
            else:
                rospy.logerr("Duplicate piece! (%d, %d)" % (row, col))            
        new_board.printBoard()
        # see how board has changed
        for i in range(8):
            for j in range(8):
                #piece = chr(ord('a')+i
                old = self.board.getPiece(i,j)
                new = new_board.getPiece(i,j)                    
                if old != None and new == None:   # this piece is gone!
                    move_cnt = move_cnt + 1
                    loc_fr = [i, j, old] 
                    rospy.loginfo("Piece moved from: %d, %d" % (i, j))
                #elif old != None and new != None and new.type = old.type:  # capture!      
        #            loc_to = piece
        #self.engine.sendline(loc_from+loc_to)
        if move_cnt > 2:
            rospy.loginfo("try again, %d" % move_cnt)        
            self.trigger()
            return
            
        self.board = new_board
        self.updated = True





if __name__=="__main__":
    try:
        executive = ChessExecutive()
        executive.playGame()
    except KeyboardInterrupt:
        pass

