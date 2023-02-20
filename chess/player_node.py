#!/usr/bin/env python3

import rospy
from serial_comms import SerialComms
from helper import substr_search, re_string_search, queue_to_space_delimited_str
from queue import Queue

class ChessPlayer():
    def __init__(self, player,device):
        self.player = player
        self.port_name = device
        self.conn = SerialComms(self.port_name)
        self.game_init = False
        self.moves_queue = Queue()
        self.uci_move_regex = r'bestmove(.*)ponder'

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
        self.conn.write("go movetime " + str(time))
        read_data = self.conn.read()
        move = re_string_search(self.uci_move_regex,read_data)
        self.moves_queue.put(move)
        print(move)
        return 


    # def main(self):
        # rospy.sleep(1)
        # device = rospy.get_param('/black/device') 
        # print(device)
        # print("!!!!!!!!!")
        # player = ChessPlayer(device)


if __name__ == '__main__':
    print("yo")
    # rospy.init_node('black')
    # rospy.init_node('white')
    # device1 = rospy.get_param('/black/device') 
    # device2 = rospy.get_param('/white/device') 
    # black = ChessPlayer(device2)
    # white = ChessPlayer(device2)
    # black.intialize_game()
    # white.intialize_game()
    # rospy.sleep(1)
    # while not rospy.is_shutdown():
    #     rospy.spin()

    # if white.game_init:
    #     white.send_moves()
    #     white.make_move()
    #     white.send_moves()
    #     white.make_move()
    #     white.send_moves()
    # white.close_game()