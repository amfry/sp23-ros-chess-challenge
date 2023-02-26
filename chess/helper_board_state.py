#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from chess.msg import Chessboard, Chesspiece, Player
import math

def create_board_state_dict():
    """Create nested dictionary to hold board state"""
    board_state_dict = {}
    for i in range(1,65):   
        board_state_dict[i] = {'row','column','piece'}
    return board_state_dict

def init_board_state_dict(board_state):
    """Create intial board_state to represent a chessboard at start of game"""
    white_pawns = Chesspiece(Player(Player.WHITE),"p")
    white_king = Chesspiece(Player(Player.WHITE),"k")
    white_queen = Chesspiece(Player(Player.WHITE),"q")
    white_rook = Chesspiece(Player(Player.WHITE),"r")
    white_knight = Chesspiece(Player(Player.WHITE),"n")
    white_bishop = Chesspiece(Player(Player.WHITE),"b")
    black_pawns = Chesspiece(Player(Player.BLACK),"p")
    black_king = Chesspiece(Player(Player.BLACK),"k")
    black_queen = Chesspiece(Player(Player.BLACK),"q")
    black_rook = Chesspiece(Player(Player.BLACK),"r")
    black_knight = Chesspiece(Player(Player.BLACK),"n")
    black_bishop = Chesspiece(Player(Player.BLACK),"b")
    empty_spaces = Chesspiece(Player(Player.WHITE),"")
    #WHITE
    for i in range(9,17):
        board_state[i] = {'row':'2','column': i,'piece': white_pawns}
    board_state[1] = {'row':'1','column': 1,'piece': white_rook}
    board_state[8] = {'row':'1','column': 8,'piece': white_rook }
    board_state[2] = {'row':'1','column': 2,'piece': white_knight}
    board_state[7] = {'row':'1','column': 7,'piece': white_knight}
    board_state[3] = {'row':'1','column': 3,'piece': white_bishop}
    board_state[6] = {'row':'1','column': 6,'piece': white_bishop}
    board_state[4] = {'row':'1','column': 4,'piece': white_queen}
    board_state[5] = {'row':'1','column': 5,'piece': white_king}
    #BLACK
    for i in range(49,57):
        board_state[i] = {'row':'2','column': i,'piece': black_pawns}
    board_state[57] = {'row':'8','column': 1,'piece': black_rook}
    board_state[64] = {'row':'8','column': 8,'piece': black_rook }
    board_state[58] = {'row':'8','column': 2,'piece': black_knight}
    board_state[63] = {'row':'8','column': 7,'piece': black_knight}
    board_state[59] = {'row':'8','column': 3,'piece': black_bishop}
    board_state[62] = {'row':'8','column': 6,'piece': black_bishop}
    board_state[60] = {'row':'8','column': 4,'piece': black_queen}
    board_state[61] = {'row':'8','column': 5,'piece': black_king}
    #EMPTY
    for i in range(17,49):
        row_index = math.ceil((i)/8)
        board_state[i] = {'row':row_index,'column': i,'piece': empty_spaces}
    return board_state

def create_chessboard_msg(board_state):
    """Unpacks board_state to create chessboard message of all pieces/empty spaces"""
    board = []
    for i in range(1,65):
        new_piece = board_state[i]['piece']
        #print(new_piece)
        board.append(new_piece)
    board = Chessboard(board)
    return board

def move_to_board_state_ids(move):
    """ 
    Returns the index(s) in board_state where the move started and ended

        Parameters:
                move (chess/Move):

        Returns:
                src_id(int): board_state index that the move originated from
                dst_id(int): board_state index that the move ended at"""
    s_row = move.src_row
    s_column = move.src_col
    d_row = move.dst_row
    d_column = move.dst_col
    src_id = ((s_row-1) *8) + s_column
    dst_id = ((d_row-1) *8) + d_column
    return src_id, dst_id

def update_board_state_from_move(move, board_state):
    """Use reccent chess move to update the board_state"""
    move_piece = move.piece
    empty_space = Chesspiece(Player(Player.BLACK),Chesspiece.EMPTY) #hardcoded cuz location empty
    src_id, dst_id = move_to_board_state_ids(move)
    board_state[src_id]["piece"] = empty_space  #src sq is empty
    board_state[dst_id]["piece"] = move_piece
    return board_state
    