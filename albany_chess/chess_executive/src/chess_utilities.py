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
import rospy    # for logging
import pexpect  # for connecting to gnu chess

from chess_msgs.msg import *

SQUARE_SIZE = 0.05715

class BoardState:
    """ A representation of a chess board state. """
    WHITE = 1
    BLACK = -1

    def __init__(self):
        """ 
        Initialize an empty board
        """
        self.values = [None for i in range(64)]
        self.last_move = "go"
        self.side = self.WHITE
        self.up_to_date = True
        self.max_changes = 2

    def newGame(self):
        """ 
        Initialize a new board
        """
        self.last_move = "go"
        self.values = [None for i in range(64)]
        for i in range(8):
            self.setPiece(i, 2, self.makePiece(ChessPiece.WHITE_PAWN))
            self.setPiece(i, 7, self.makePiece(ChessPiece.BLACK_PAWN))

        self.setPiece('a', 1, self.makePiece(ChessPiece.WHITE_ROOK))
        self.setPiece('b', 1, self.makePiece(ChessPiece.WHITE_KNIGHT))
        self.setPiece('c', 1, self.makePiece(ChessPiece.WHITE_BISHOP))
        self.setPiece('d', 1, self.makePiece(ChessPiece.WHITE_QUEEN))
        self.setPiece('e', 1, self.makePiece(ChessPiece.WHITE_KING))
        self.setPiece('f', 1, self.makePiece(ChessPiece.WHITE_BISHOP))
        self.setPiece('g', 1, self.makePiece(ChessPiece.WHITE_KNIGHT))
        self.setPiece('h', 1, self.makePiece(ChessPiece.WHITE_ROOK))

        self.setPiece('a', 8, self.makePiece(ChessPiece.BLACK_ROOK))
        self.setPiece('b', 8, self.makePiece(ChessPiece.BLACK_KNIGHT))
        self.setPiece('c', 8, self.makePiece(ChessPiece.BLACK_BISHOP))
        self.setPiece('d', 8, self.makePiece(ChessPiece.BLACK_QUEEN))
        self.setPiece('e', 8, self.makePiece(ChessPiece.BLACK_KING))
        self.setPiece('f', 8, self.makePiece(ChessPiece.BLACK_BISHOP))
        self.setPiece('g', 8, self.makePiece(ChessPiece.BLACK_KNIGHT))
        self.setPiece('h', 8, self.makePiece(ChessPiece.BLACK_ROOK))

    def makePiece(self, val):
        """ 
        Helper function to generate ChessPiece messages.
        """
        p = ChessPiece()
        p.type = val
        return p

    def setPiece(self, column, rank, piece):
        """ 
        Set the value of a piece on the board. The piece
        should be a chess_msgs/ChessPiece, which has a 
        pose and type, or None
        
        Column: 0 or 'a' = column A
        Rank:   1 = rank 1
        """
        try:
            self.values[int(rank-1)*8+self.getColIdx(column)] = piece
        except:
            rospy.loginfo("setPiece: invalid row/column")
        
    def getPiece(self, column, rank):
        try:
            return self.values[int(rank-1)*8+self.getColIdx(column)]
        except:
            return None

    def printBoard(self):
        """ Print board state to screen. """
        for c in 'abcdefgh':
            for r in [1,2,3,4,5,6,7,8]:
                p = self.getPiece(c,r)    # print a1 first
                if p == None:
                    print " ",
                elif p.type > 0:
                    print "w",
                else:
                    print "b",
            print ""

    def applyUpdate(self, message):
        """ Update the board state, given a new ChessBoard message. """
        piece_fr  = None   # location moved from
        piece_to  = None   # location moved to
        piece_cnt = 0      # number of pieces moved
        # process ChessBoard message  
        temp_board = BoardState()  
        for piece in message.pieces:
            # get col, rank as "x0"
            col = self.getColName(int(piece.pose.position.x/SQUARE_SIZE + SQUARE_SIZE/2))
            rank = int(piece.pose.position.y/SQUARE_SIZE + SQUARE_SIZE/2) + 1
            if not self.valid(col, rank):
                continue
            # update temp board
            if temp_board.getPiece(col, rank) == None:
                p = self.getPiece(col, rank)
                if p == None:
                    piece_cnt = piece_cnt + 1
                    if piece_to == None:
                        piece_to = [col, rank, p] 
                    rospy.loginfo("Piece moved to: %s%s" % (col,str(rank)))
#               else:
#                   piece.type = p.type
                temp_board.setPiece(col, rank, piece)
            else:
#               rospy.logerr("Duplicate piece! %s" % self.toString(col, rank))  
                pass          
        temp_board.printBoard()
        # see how board has changed
        for col in 'abcdefgh':
            for rank in [1,2,3,4,5,6,7,8]:
                old = self.getPiece(col,rank)
                new = temp_board.getPiece(col,rank)                    
                if old != None and new == None:
                    # this piece is gone!
                    piece_cnt = piece_cnt + 1
                    piece_fr = [col, rank, old] 
                    rospy.loginfo("Piece moved from: %s%s" % (col,str(rank)))
                elif old != None and new != None and new.type/abs(float(new.type)) != old.type/abs(float(old.type)):
                    # capture!
                    piece_cnt = piece_cnt + 1
                    piece_to = [col, rank, new]
                    rospy.loginfo("Piece captured: %s%s" % (col,str(rank)))
        # is this plausible?
        if piece_cnt > self.max_changes:
            rospy.loginfo("Try again, %d" % piece_cnt)        
            self.last_move = "fail"
        else:
            try:
                self.last_move = piece_fr[0] + str(piece_fr[1]) + piece_to[0] + str(piece_to[1]) 
                self.values = temp_board.values
            except:
                if piece_fr == None and piece_to == None:
                    rospy.loginfo("No change")
                    self.last_move = "none"
                else:
                    rospy.loginfo("Try again, from or to not set")             
                    self.last_move = "fail"    
        self.up_to_date = True

    def applyMove(self, move):
        """ Update the board state, given a move from GNU chess. """
        (col_f, rank_f) = self.toPosition(move[0:2])
        (col_t, rank_t) = self.toPosition(move[2:])
        piece = self.getPiece(col_f, rank_f)
        self.setPiece(col_t, rank_t, piece) 
        self.setPiece(col_f, rank_f, None)       

    def computeSide(self):
        """ Determine which side of the board we are on. """
        side = 0
        for c in 'abcdefgh':
            side += self.getPiece(c,1).type
            side += self.getPiece(c,2).type
            side -= self.getPiece(c,7).type
            side -= self.getPiece(c,8).type       
            rospy.loginfo("Computed side value of: %d" % side)
        if side > 0:
            self.side = self.WHITE
        else:
            self.side = self.BLACK

    #######################################################
    # helpers
    def toPosition(self, pos):
        """ Get position for a string name like 'x0'. """
        return [ord(pos[0])-ord('a'), int(pos[1])]

    def getColName(self, col):
        """ Convert to column string name. """
        try:
            return chr(ord('a') + col)
        except:
            return col        

    def getColIdx(self, col):
        """ Convert to column integer index. """
        try: 
            return int(col)
        except:
            return ord(col)-ord('a')      

    def valid(self, col, rank):
        """ Is a particular position valid? """
        return rank <= 8 and rank > 0 and self.getColIdx(col) < 8 and self.getColIdx(col) >= 0


class GnuChessEngine:
    """ Connection to a GNU chess engine. """

    def __init__(self):
        """
        Start a connection to GNU chess.
        """ 
        self.engine = pexpect.spawn('/usr/games/gnuchess -x')
        self.history = list()  

    def startNewGame(self):
        self.engine.sendline('new')
        self.history = list()

    def nextMove(self, move="go"):
        """
        Give opponent's move, get back move to make. 
            returns None if given an invalid move.
        """
        self.engine.sendline(move)        
        if self.engine.expect(['My move is','Illegal move']) == 1:
            return None     
        self.engine.expect('([a-h][1-8][a-h][1-8][RrNnBbQq(\r\n)])')
        m = self.engine.after.rstrip()
        self.history.append(m)
        return m


