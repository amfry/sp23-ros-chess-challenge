#!/usr/bin/env python3

import rospy
from serial_comms import SerialComms
from helper import substr_search

class ChessPlayer():
    def __init__(self, port_name):
        self.port_name = port_name
        self.conn = SerialComms(self.port_name)
        self.game_init = False

    def intialize_game(self):
        self.conn.open_port()
        self.conn.write("uci")
        self.conn.write("ucinewgame")
        self.conn.write("isready")
        read_data = self.conn.read_decode()
        print(read_data)
        self.game_init = substr_search("readyok",read_data)
        print(self.game_init)
        self.conn.close_port()



if __name__ == '__main__':
    port = "/tmp/stockfish"
    white = ChessPlayer(port)
    white.intialize_game()