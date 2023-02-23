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
    print(type(move_read))
    src_row = ord(move_read[0]) - 96
    src_col = int(move_read[1])
    dst_row = ord(move_read[2]) - 96 #convert string of letter to its index in alphabet
    dst_col = int(move_read[3])
    is_castling = False
    is_promotion = False
    # print(player_id)
    piece = Chesspiece(player=Player(Player.WHITE),kind=Chesspiece.EMPTY)
    return Move(src_row, src_col, dst_row, dst_col, is_castling, is_promotion, piece)



if __name__ == '__main__':
    q = Queue()
    q.put('d2d4')
    q.put('g8f6')
    q.put('e2e4q')
    q.put('e7e6')
    print(queue_to_space_delimited_str(q))

    