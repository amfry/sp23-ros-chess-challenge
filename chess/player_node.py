#!/usr/bin/env python3

import rospy
from serial_comms import SerialComms
from helper import substr_search, re_string_search, queue_to_space_delimited_str, make_move_msg
from queue import Queue
from chess.msg import Chessboard, Chesspiece, Player, Move
from std_msgs.msg import Empty


class ChessPlayer():
    def __init__(self, player, device):
        self.player = player
        self.device = device
        self.conn = SerialComms(self.device)
        self.game_init = False
        self.moves_queue = Queue()
        self.uci_move_regex = r'bestmove(.*)ponder'
        self.player_type = self.check_player_type()
        self.players_turn = False

    def check_player_type(self):
        print(self.player)
        if self.player == '/white':
            return 0
        else:
            return 1 # for blacks

    def close_game(self):
        self.conn.close_port()

    def intialize_game(self):
        rospy.sleep(2)
        self.conn.open_port()
        uci_flag = self.send_uci()
        new_game_flag = self.send_ucinewgame()
        ready_flag = self.send_isready()
        if uci_flag and new_game_flag and ready_flag:
            self.game_init = True
            print("Game Intialized!")
      

    def send_uci(self):
        try:
            self.conn.write("uci")
            flag = True
            #read_data = self.conn.read()
            #flag = substr_search("uciok",read_data)
        except:
            flag = False
        return flag
    
    def send_ucinewgame(self):
        try:
            self.conn.write("ucinewgame")
            flag = True
        except:
            flag = False
        return flag
    
    def send_isready(self):
        try:
            self.conn.write("isready")
            read_data = self.conn.read()
            flag = substr_search("readyok",read_data)
        except:
            flag = False
        return flag
    
    def send_moves(self):
        all_moves = queue_to_space_delimited_str(self.moves_queue)
        self.conn.write("position startpos moves" + all_moves)
        return
    
    def make_move(self, time=1000):
        # self.conn.write("go movetime " + str(time))
        # read_data = self.conn.read()
        # print(read_data)
        # if len(read_data) == 0:
        #     print("trying again")
        #     read_data = self.conn.read() #try again if no data collected
        # move_read = re_string_search(self.uci_move_regex,read_data)
        move_read ='d2d4' #hardcoding first move for dev!
        move_msg = make_move_msg(move_read, self.player_type)
        return move_msg
    
    def first_turn(self):
        if self.player_type == 0:
            self.players_turn = True
        else:
            self.players_turn = False


def main():
    rospy.sleep(1)
    rospy.init_node('chess_player')
    r = rospy.Rate(10) # 10hz
    player = rospy.get_name()
    device = rospy.get_param(player+'/device')
    chess_player = ChessPlayer(player, device) #
    chess_player.intialize_game()
    print(chess_player.game_init)
    rospy.sleep(1)
    move_topic = player+'/move'
    move_pub = rospy.Publisher(move_topic, Move, queue_size=3) #publisher for moves
    capture_topic = player+'/capture'
    capture_pub = rospy.Publisher(capture_topic, Chesspiece, queue_size=3) #publisher for captures
    board_topic = '/chessboard'
    board_pub = rospy.Publisher(board_topic, Chessboard, queue_size=1) #publisher for updated board
    done_topic = player+'/done'
    done_pub = rospy.Publisher(done_topic, Empty, queue_size=3) #publisher to notify end of turn
    chess_player.game_init = True
    if chess_player.game_init:
        while not rospy.is_shutdown():
            move_to_pub= chess_player.make_move()
            move_pub.publish(move_to_pub)
            print(move_to_pub)
            done_pub.publish(Empty)
            rospy.sleep(15)
            break
    chess_player.close_game()

if __name__ == '__main__':
    main()