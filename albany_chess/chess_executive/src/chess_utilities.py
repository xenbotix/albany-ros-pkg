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

    def __init__(self, side=None):
        """ 
        Initialize an empty board
        """
        self.values = [None for i in range(64)]
        self.last_move = "go"
        self.side = side
        self.up_to_date = True
        self.max_changes = 2
        self.output = False

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

    def makePiece(self, val, copy=None):
        """ 
        Helper function to generate ChessPiece messages.
        """
        p = ChessPiece()
        p.header.frame_id = "chess_board"
        if copy != None:
            p.pose = copy.pose
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
    def getPieceType(self, column, rank):
        try:
            return self.values[int(rank-1)*8+self.getColIdx(column)].type
        except:
            return 0

    def printBoard(self):
        """ Print board state to screen. """
        if self.side == self.WHITE or self.side == None:
            for r in [8,7,6,5,4,3,2,1]:
                for c in 'abcdefgh':
                    p = self.getPiece(c,r)    # print a8 first
                    if p == None:
                        print " ",
                    else:
                        print self.getPieceName(p.type),
                print ""
        else:
            for r in [1,2,3,4,5,6,7,8]:
                for c in 'hgfedcba':
                    p = self.getPiece(c,r)    # print h1 first
                    if p == None:
                        print " ",
                    else:
                        print self.getPieceName(p.type),
                print ""

    def applyUpdate(self, message):
        """ Update the board state, given a new ChessBoard message. """
        piece_fr  = None   # location moved from
        piece_to  = None   # location moved to
        piece_cnt = 0      # number of pieces moved
        piece_change = 0   # number of pieces changed color
        # process ChessBoard message  
        temp_board = BoardState(self.side)  
        for piece in message.pieces:
            # get col, rank as "x0"
            if self.side == self.WHITE or self.side == None:
                col = self.getColName(int(piece.pose.position.x/SQUARE_SIZE))
                rank = int(piece.pose.position.y/SQUARE_SIZE) + 1
            else:
                col = self.getColName(7 - int(piece.pose.position.x/SQUARE_SIZE))
                rank = 8 - int(piece.pose.position.y/SQUARE_SIZE)
            if not self.valid(col, rank):
                continue
            # update temp board
            if temp_board.getPiece(col, rank) == None:
                p = self.getPiece(col, rank)
                if p == None and not self.side == None:
                    piece_cnt = piece_cnt + 1
                    if piece_to == None:
                        piece_to = [col, rank, piece] 
                    rospy.logdebug("Piece moved to: %s%s" % (col,str(rank)))
#               else:
#                   piece.type = p.type
                temp_board.setPiece(col, rank, piece)
#           else:
#               rospy.logerr("Duplicate piece! %s" % self.toString(col, rank))  
#               pass          
        # see how board has changed
        for col in 'abcdefgh':
            for rank in [1,2,3,4,5,6,7,8]:
                old = self.getPiece(col,rank)
                new = temp_board.getPiece(col,rank)                    
                if new == None and old != None:
                    # this piece is gone!
                    piece_cnt = piece_cnt + 1
                    piece_fr = [col, rank, old]
                    rospy.logdebug("Piece moved from: %s%s" % (col,str(rank)))
                elif old != None and new != None and new.type/abs(float(new.type)) != old.type/abs(float(old.type)):
                    # capture!
                    piece_cnt = piece_cnt + 1
                    piece_change = piece_change + 1
                    piece_to = [col, rank, new]
                    rospy.logdebug("Piece captured: %s%s" % (col,str(rank))) 
                elif old != None and new != None:
                    # boring, but update types!
                    new.type = old.type
                    temp_board.setPiece(col,rank,new)
        # update types
        if piece_to != None and piece_fr != None:
            fr = self.getPiece(piece_fr[0], piece_fr[1])
            to = temp_board.getPiece(piece_to[0], piece_to[1])
            to.type = fr.type
            temp_board.setPiece(piece_to[0],piece_to[1],to)
        if self.output: 
            temp_board.printBoard()
            self.output = False
        # is this plausible?
        if self.side == None:
            if piece_cnt == 0 or piece_cnt >= 32:
                rospy.loginfo("No side set, done") 
                self.last_move = "none"
                self.values = temp_board.values
            else:
                rospy.logdebug("Try again, %d" % piece_cnt)        
                self.last_move = "fail"
        elif piece_cnt > self.max_changes:
            rospy.logdebug("Try again, %d" % piece_cnt)        
            self.last_move = "fail"
        else:
            try:
                self.previous = [self.values, self.last_move] 
                self.last_move = piece_fr[0] + str(piece_fr[1]) + piece_to[0] + str(piece_to[1])
                self.values = temp_board.values
            except:
                if piece_fr == None and piece_to == None:
                    rospy.logdebug("No change")
                    self.last_move = "none"
                else:
                    rospy.logdebug("Try again, from or to not set")             
                    self.last_move = "fail"    
        self.up_to_date = True

    def revert(self):
        self.values = self.previous[0]
        self.last_move = self.previous[1]

    def applyMove(self, move, pose=None):
        """ Update the board state, given a move from GNU chess. """
        (col_f, rank_f) = self.toPosition(move[0:2])
        (col_t, rank_t) = self.toPosition(move[2:])
        piece = self.getPiece(col_f, rank_f)
        piece.pose = pose
        self.setPiece(col_t, rank_t, piece) 
        self.setPiece(col_f, rank_f, None)       

    def computeSide(self):
        """ Determine which side of the board we are on. """
        side = 0
        for c in 'abcdefgh':
            side += self.getPieceType(c,1)
            side += self.getPieceType(c,2)
            side -= self.getPieceType(c,7)
            side -= self.getPieceType(c,8)       
            rospy.loginfo("Computed side value of: %d" % side)
        if side > 0:
            self.side = self.WHITE   # good to go
        else:
            self.side = self.BLACK   
            # need to setup board 
            temp_board = BoardState(self.side) 
            for i in range(8):
                temp_board.setPiece(i, 2, self.makePiece(ChessPiece.WHITE_PAWN, self.getPiece(7-i, 7)) )
                temp_board.setPiece(i, 7, self.makePiece(ChessPiece.BLACK_PAWN, self.getPiece(7-i, 2)) )

            temp_board.setPiece('a', 1, self.makePiece(ChessPiece.WHITE_ROOK, self.getPiece('h',8)) )
            temp_board.setPiece('b', 1, self.makePiece(ChessPiece.WHITE_KNIGHT, self.getPiece('g',8)))
            temp_board.setPiece('c', 1, self.makePiece(ChessPiece.WHITE_BISHOP, self.getPiece('f',8)))
            temp_board.setPiece('d', 1, self.makePiece(ChessPiece.WHITE_QUEEN, self.getPiece('e',8)))
            temp_board.setPiece('e', 1, self.makePiece(ChessPiece.WHITE_KING, self.getPiece('d',8)))
            temp_board.setPiece('f', 1, self.makePiece(ChessPiece.WHITE_BISHOP, self.getPiece('c',8)))
            temp_board.setPiece('g', 1, self.makePiece(ChessPiece.WHITE_KNIGHT, self.getPiece('b',8)))
            temp_board.setPiece('h', 1, self.makePiece(ChessPiece.WHITE_ROOK, self.getPiece('a',8)))

            temp_board.setPiece('a', 8, self.makePiece(ChessPiece.BLACK_ROOK, self.getPiece('h',1)) )
            temp_board.setPiece('b', 8, self.makePiece(ChessPiece.BLACK_KNIGHT, self.getPiece('g',1)) )
            temp_board.setPiece('c', 8, self.makePiece(ChessPiece.BLACK_BISHOP, self.getPiece('f',1)) )
            temp_board.setPiece('d', 8, self.makePiece(ChessPiece.BLACK_QUEEN, self.getPiece('e',1)) )
            temp_board.setPiece('e', 8, self.makePiece(ChessPiece.BLACK_KING, self.getPiece('d',1)) )
            temp_board.setPiece('f', 8, self.makePiece(ChessPiece.BLACK_BISHOP, self.getPiece('c',1)) )
            temp_board.setPiece('g', 8, self.makePiece(ChessPiece.BLACK_KNIGHT, self.getPiece('b',1)) )
            temp_board.setPiece('h', 8, self.makePiece(ChessPiece.BLACK_ROOK, self.getPiece('a',1)) ) 

            self.values = temp_board.values
            self.printBoard()

        self.last_move = "go"

    #######################################################
    # helpers
    def toPosition(self, pos):
        """ Get position for a string name like 'a1'. """
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

    def getPieceName(self, piece_type):
        if piece_type == ChessPiece.WHITE_PAWN:
            return "P"
        elif piece_type == ChessPiece.WHITE_ROOK:
            return "R"
        elif piece_type == ChessPiece.WHITE_KNIGHT:
            return "N"
        elif piece_type == ChessPiece.WHITE_BISHOP:
            return "B"
        elif piece_type == ChessPiece.WHITE_QUEEN:
            return "Q"
        elif piece_type == ChessPiece.WHITE_KING:
            return "K"
        elif piece_type == ChessPiece.BLACK_PAWN:
            return "p"
        elif piece_type == ChessPiece.BLACK_ROOK:
            return "r"
        elif piece_type == ChessPiece.BLACK_KNIGHT:
            return "n"
        elif piece_type == ChessPiece.BLACK_BISHOP:
            return "b"
        elif piece_type == ChessPiece.BLACK_QUEEN:
            return "q"
        elif piece_type == ChessPiece.BLACK_KING:
            return "k"
        return "x"

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
        #rospy.loginfo(str(self.engine))
        return m

    def exit(self):
        print "game review:"
        for h in self.history:
            print h
        self.engine.sendline('exit')

