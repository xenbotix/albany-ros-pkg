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
import rospy
import pexpect

from chess_msgs.msg import *


class BoardState:
    """ A representation of a chess board state. """

    def __init__(self):
        """ 
        Initialize an empty board
        """
        self.values = [None for i in range(64)]

    def newGame(self):
        """ 
        Initialize a new board
        """
        self.values = [None for i in range(64)]
        for i in range(8):
            self.setPiece(1, i, self.makePiece(ChessPiece.WHITE_PAWN))
            self.setPiece(6, i, self.makePiece(ChessPiece.BLACK_PAWN))

        self.setPiece(0, 0, self.makePiece(ChessPiece.WHITE_ROOK))
        self.setPiece(0, 1, self.makePiece(ChessPiece.WHITE_KNIGHT))
        self.setPiece(0, 2, self.makePiece(ChessPiece.WHITE_BISHOP))
        self.setPiece(0, 3, self.makePiece(ChessPiece.WHITE_QUEEN))
        self.setPiece(0, 4, self.makePiece(ChessPiece.WHITE_KING))
        self.setPiece(0, 5, self.makePiece(ChessPiece.WHITE_BISHOP))
        self.setPiece(0, 6, self.makePiece(ChessPiece.WHITE_KNIGHT))
        self.setPiece(0, 7, self.makePiece(ChessPiece.WHITE_ROOK))

        self.setPiece(7, 0, self.makePiece(ChessPiece.BLACK_ROOK))
        self.setPiece(7, 1, self.makePiece(ChessPiece.BLACK_KNIGHT))
        self.setPiece(7, 2, self.makePiece(ChessPiece.BLACK_BISHOP))
        self.setPiece(7, 3, self.makePiece(ChessPiece.BLACK_QUEEN))
        self.setPiece(7, 4, self.makePiece(ChessPiece.BLACK_KING))
        self.setPiece(7, 5, self.makePiece(ChessPiece.BLACK_BISHOP))
        self.setPiece(7, 6, self.makePiece(ChessPiece.BLACK_KNIGHT))
        self.setPiece(7, 7, self.makePiece(ChessPiece.BLACK_ROOK))

    def makePiece(self, val):
        """ 
        Helper function to generate ChessPiece messages.
        """
        p = ChessPiece()
        p.type = val
        return p

    def setPiece(self, rank, column, piece):
        """ 
        Set the value of a piece on the board. The piece
        should be a chess_msgs/ChessPiece, which has a 
        pose and type, or None
        
        Column: 0 = column A
        Rank:   0 = rank 1
        """
        try:
            self.values[int(rank)*8+int(column)] = piece
        except:
            try:
                self.values[int(rank)*8+(ord(column)-ord('a'))] = piece
            except:
                rospy.loginfo("setPiece: invalid row/column")
        
    def getPiece(self, rank, column):
        try:
            return self.values[int(rank)*8+int(column)]
        except:
            try:
                return self.values[int(rank)*8+(ord(column)-ord('a'))]
            except:
                return None

    def printBoard(self):
        for i in range(8):
            for j in range(8):
                p = self.getPiece(j,i)    # print a1 first
                if p == None:
                    print " ",
                elif p.type > 0:
                    print "w",
                else:
                    print "b",
            print ""


class GnuChessPlanner:
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
        """
        self.engine.sendline(move)        
        self.engine.expect('My move is')
        self.engine.expect('([a-h][1-8][a-h][1-8][RrNnBbQq(\r\n)])')
        m = self.engine.after.rstrip()
        self.history.append(m)
        return m


