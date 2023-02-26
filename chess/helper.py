#!/usr/bin/env python3

import re
from queue import Queue
from chess.msg import Move, Chesspiece, Player
from helper_board_state import *

def substr_search(substring, search_str):
    if substring in search_str:
        return True
    else:
        return False
    
def re_string_search(regex, search_str):
    match = re.search(regex, search_str).group(1)
    return match.strip()

def queue_to_space_delimited_str(queue):
    spaced_str = ''
    size = queue.qsize()
    for i in range(0,size):
        spaced_str = str(spaced_str + queue.get() + ' ')
    print("---")
    print(spaced_str)
    print("---")
    return spaced_str

def make_move_msg(move_read, player_id, board_state):
    """does not handle castle or promotion!!!"""
    #print(type(move_read))
    is_castling = False
    is_promotion = False
    src_col = ord(move_read[0]) - 96
    src_row = int(move_read[1])
    dst_col = ord(move_read[2]) - 96 #convert string of letter to its index in alphabet
    dst_row = int(move_read[3])
    src_id = ((src_row-1) *8) + src_col
    piece_moved = board_state[src_id]["piece"].kind
    if len(move_read) > 4:
        piece_moved = move_read[4]
        print("PROMOTED!!!!!!!!!!!")
        is_promotion = True
    is_castling = False
    
    
    # print(player_id)
    piece = Chesspiece(player=Player(player_id),kind=piece_moved) #TODO how do I know what piece it is
    return Move(src_row, src_col, dst_row, dst_col, is_castling, is_promotion, piece)

def unpack_moves_to_str(move):
    move_str = ''
    move_str = move_str + chr(move.src_col + 96)
    move_str = move_str + str(move.src_row)
    move_str = move_str + chr(move.dst_col + 96)
    move_str = move_str + str(move.dst_row)
    if not move.is_castling:
        return move_str
    else:
        #move_str = move_str + str(move.dst_col) #TODO how do I know what piece it is
        return move_str


def add_space(move):
    val = '{} '.format(move) 
    return val

def move_msg_list_to_space_del_list(move_list):
    space_del_move = ''
    for move in move_list:
        move_str = unpack_moves_to_str(move)
        move_str_space = add_space(move_str)
        space_del_move = space_del_move + move_str_space
    return space_del_move


if __name__ == '__main__':
    board = create_board_state_dict()
    board = init_board_state_dict(board)
    msg1 = make_move_msg('d2d4q',1,board)
    # msg2 = make_move_msg('e7e6',0)
    # msg3 = make_move_msg('c2c4',1)
    msg1
   

    