#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import re
from chess.msg import Move, Chesspiece, Player

def substr_search(substring, search_str):
    """Find substring in string"""
    if substring in search_str:
        return True
    else:
        return False
    
def re_string_search(regex, search_str):
    """ 
    Returns group that in search_str that matches regular expression

        Parameters:
                regex(str): move from chess engine using universal chess interface 
                search_Str(int): string to search for regex expression

        Returns:
                matching group with extra spaces removed """
    match = re.search(regex, search_str).group(1)
    return match.strip()

def make_move_msg(move_read, player_id, board_state):
    """ 
    Packages move from UCI protocal into chess/Move message

        Parameters:
                move_read(str): move from chess engine using universal chess interface 
                player_id(int): 1 or 0 representing white or black player
                board_state(dict): current board_state

        Returns:
                move_msg(chess/Move): """
    src_col = ord(move_read[0]) - 96 #convert string to its index in alphabet
    src_row = int(move_read[1])
    dst_col = ord(move_read[2]) - 96 
    dst_row = int(move_read[3])
    src_id = ((src_row-1) *8) + src_col
    piece_moved = board_state[src_id]["piece"].kind
    is_promotion = False
    if len(move_read) > 4:
        piece_moved = move_read[4]
        is_promotion = True
    is_castling, board_state = check_castling(move_read, player_id, board_state)
    piece = Chesspiece(player=Player(player_id),kind=piece_moved)
    move_msg = Move(src_row, src_col, dst_row, dst_col, is_castling, is_promotion, piece)
    return move_msg

def unpack_moves_to_str(move):
    """Convert Move msg to a string uci command"""
    move_str = ''
    move_str = move_str + chr(move.src_col + 96)
    move_str = move_str + str(move.src_row)
    move_str = move_str + chr(move.dst_col + 96)
    move_str = move_str + str(move.dst_row)
    return move_str

def move_msg_list_to_space_del_list(move_list):
    """Convert list of Move messages to space  delimited string"""
    space_del_move = ''
    for move in move_list:
        move_str = unpack_moves_to_str(move)
        move_str_space = '{} '.format(move_str) #add space btw commands 
        space_del_move = space_del_move + move_str_space
    return space_del_move

def check_castling(move_read, player_id, board_state):
    """Check for basic long and short castling based off king move"""
    castle_flag = False
    if player_id == 0:
        if move_read ==  "e1g1":
            castle_flag = True
            board_state[6]["piece"].kind = "r"
        if move_read ==  "e1c1":
            castle_flag = True
            board_state[4]["piece"].kind = "r"
    if player_id == 1:
        if move_read ==  "e8g8":
            castle_flag = True
            board_state[61]["piece"].kind = "r"
        if move_read ==  "e8c8":
            castle_flag = True
            board_state[59]["piece"].kind = "r"
    return castle_flag, board_state



