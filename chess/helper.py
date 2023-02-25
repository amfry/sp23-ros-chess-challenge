#!/usr/bin/env python3

import re
from queue import Queue
from chess.msg import Move, Chesspiece, Player

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

def make_move_msg(move_read, player_id):
    """does not handle castle or promotion!!!"""
    #print(type(move_read))
    src_row = ord(move_read[0]) - 96
    src_col = int(move_read[1])
    dst_row = ord(move_read[2]) - 96 #convert string of letter to its index in alphabet
    dst_col = int(move_read[3])
    is_castling = False
    is_promotion = False
    # print(player_id)
    piece = Chesspiece(player=Player(player_id),kind=Chesspiece.PAWN) #TODO how do I know what piece it is
    return Move(src_row, src_col, dst_row, dst_col, is_castling, is_promotion, piece)

def unpack_moves_to_str(move):
    move_str = ''
    move_str = move_str + chr(move.src_row + 96)
    move_str = move_str + str(move.src_col)
    move_str = move_str + chr(move.dst_row + 96)
    move_str = move_str + str(move.dst_col)
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
    msg1 = make_move_msg('d2d4',1)
    msg2 = make_move_msg('e7e6',0)
    msg3 = make_move_msg('c2c4',1)
    vals  = [msg1,msg2,msg3]
    print(move_msg_list_to_space_del_list(vals))
   

    