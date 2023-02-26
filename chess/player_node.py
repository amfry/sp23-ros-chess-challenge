#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from serial_comms import SerialComms
from chess.msg import Chessboard, Chesspiece, Move
from std_msgs.msg import Empty
from helper import substr_search, re_string_search, make_move_msg, move_msg_list_to_space_del_list
from helper_board_state import update_board_state_from_move, create_chessboard_msg
from helper_board_state import create_board_state_dict, init_board_state_dict, move_to_board_state_ids


class ChessPlayer():
    """ A Chess Player class to interact with Stockfish chess engine over
    UCI commands through a serial device """
    def __init__(self, player, device):
        self.player = player
        self.device = device
        self.conn = None
        self.uci_move_regex = r'bestmove(.*)ponder'
        self.player_type = self.check_player_type()
        self.opponent = self.set_opponent()
        self.all_moves = []
        self.board_state = self.init_board_state()
        self.players_turn = False
        self.game_init = False

    def __enter__(self):
        self.conn = SerialComms(self.device)
        self.conn.open_port()
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close_game()
        if exc_type:
            print(exc_type, exc_value, traceback)
        return
    
    def close_game(self):
        print("Exiting...")
        self.conn.close_port()

    def intialize_game(self):
        uci_flag = self.send_uci()
        new_game_flag = self.send_ucinewgame()
        ready_flag = self.send_isready()
        if uci_flag and new_game_flag and ready_flag:
            self.game_init = True
            print("Game Intialized!")
        return self.game_init

    def check_player_type(self):
        """Determine what "color" instantiated class is"""
        if self.player == '/white':
            self.opponent = '/black'
            return 0
        else:
            self.opponent = '/white'
            return 1
        
    def set_opponent(self):
        """Determine chess opponet for instantiated class"""
        if self.player == '/white':
            return '/black'
        else:
            return'/white'
        
    def first_turn(self):
        """Determine player order for instantiated class"""
        if self.player_type == 0:
            self.players_turn = True
        else:
            self.players_turn = False

    def send_uci(self):
        try:
            self.conn.write("uci")
            flag = True
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
        """Send UCI commands to set-up game to serail device"""
        try:
            self.conn.write("isready")
            read_data = self.conn.read()
            flag = substr_search("readyok",read_data)
        except:
            flag = False
        return flag
    
    def send_moves(self):
        """Send all prior moves in game using UCI to serial device"""
        all_moves = move_msg_list_to_space_del_list(self.all_moves)
        self.conn.write("position startpos moves " + all_moves)
        rospy.sleep(1)
        return
    
    def make_move(self, time=1000):
        """Request chess engine to make move, reads result, and pacakges
            info into chess/Move message type

        Parameters:
                time(int): time in miliseconds for chess engine to generate
                    best move

        Returns:
                move_msg (cheess/Move):"""
        self.send_moves()
        self.conn.write("go movetime " + str(time))
        read_data = self.conn.read()
        move_read = re_string_search(self.uci_move_regex,read_data)
        move_msg = make_move_msg(move_read, self.player_type, self.board_state)
        self.all_moves.append(move_msg) #add your move to list
        return move_msg
    
    def update_board_state(self):
        """Update board_state to reflect all completed moves in game"""
        for move in self.all_moves:
            self.board_state = update_board_state_from_move(move,self.board_state)
        return self.board_state
    
    def init_board_state(self):
        """Intialize board_state data structure with default config"""
        empty_board_dict = create_board_state_dict()
        board_state = init_board_state_dict(empty_board_dict)
        return board_state

    def check_move_for_capture(self,move):
        """Check if move will lead to capture of piece on the board"""
        capture_flag = False
        src_id, dst_id = move_to_board_state_ids(move)
        chesspiece_msg = self.board_state[dst_id]["piece"]
        piece_type = chesspiece_msg.kind
        if piece_type != "":
            print("CAPTURE!")
            capture_flag = True
        return capture_flag, chesspiece_msg

    def turn_listener(self):
        rospy.Subscriber(self.opponent+'/done', Empty, self.opponent_finished_turn)
       
    def opponent_finished_turn(self, data):
        """Check message if opponent has indicated end of turn"""
        if data == Empty():
            self.players_turn = True
        return
    
    def opponent_last_move_listener(self):
        name = self.opponent+'/move'
        rospy.Subscriber(name, Move, self.opponent_last_move)
        return

    def opponent_last_move(self, data):
        """Add opponents last move to record of all moves in game"""
        if len(self.all_moves) == 0:
            self.all_moves.append(data) 
        elif data != self.all_moves[-1]:
            self.all_moves.append(data) 
        return

def main():
    rospy.init_node("chess_player")
    player = rospy.get_name()
    device = rospy.get_param(player+'/device')
    rate = rospy.Rate(5)
    print(player, device)
    with ChessPlayer(player, device) as chess_player:
        rospy.sleep(1)
        move_topic = player+'/move'
        move_pub = rospy.Publisher(move_topic, Move, queue_size=3) #publisher for moves
        chessboard_topic = '/chessboard'
        chessboard_pub = rospy.Publisher(chessboard_topic, Chessboard, queue_size=3)
        capture_topic = player+'/capture'
        capture_pub = rospy.Publisher(capture_topic, Chesspiece, queue_size=3) #publisher for moves
        player_done_topic = player+'/done'
        player_done_pub = rospy.Publisher(player_done_topic, Empty, queue_size=3) #publisher to notify end of turn
        chess_player.intialize_game()
        if chess_player.game_init:
            if chess_player.player_type == 0:
                #white makes first move
                print(player+': Move!')
                chess_player.players_turn = True
                move_to_pub= chess_player.make_move()
                print(move_msg_list_to_space_del_list(chess_player.all_moves))
                move_pub.publish(move_to_pub)
                updated_board_state = chess_player.update_board_state()
                chessboard_pub.publish(create_chessboard_msg(updated_board_state))
                player_done_pub.publish(Empty())
                chess_player.players_turn = False
            while not rospy.is_shutdown():
                if chess_player.players_turn != True:
                    chess_player.turn_listener() #check if it is my turn
                    chess_player.opponent_last_move_listener() #check for last move
                else:
                    print(player+': Move!')
                    move_to_pub=chess_player.make_move()
                    print("all plays: ", move_msg_list_to_space_del_list(chess_player.all_moves))
                    move_pub.publish(move_to_pub)
                    capture_flag, captured_piece = chess_player.check_move_for_capture(move_to_pub)
                    if capture_flag:
                        capture_pub.publish(captured_piece)
                    updated_board_state = chess_player.update_board_state()
                    chessboard_pub.publish(create_chessboard_msg(updated_board_state))
                    player_done_pub.publish(Empty())
                    chess_player.players_turn = False
                rate.sleep()

if __name__ == '__main__':
    main()