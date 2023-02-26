#!/usr/bin/env python3

import rospy
from serial_comms import SerialComms
from helper import substr_search, re_string_search, make_move_msg, move_msg_list_to_space_del_list
from helper_board_state import *
from chess.msg import Chessboard, Chesspiece, Player, Move
from std_msgs.msg import Empty



class ChessPlayer():
    def __init__(self, player, device):
        self.player = player
        self.device = device
        self.conn = SerialComms(self.device)
        self.game_init = False
        #self.moves_queue = Queue()
        self.uci_move_regex = r'bestmove(.*)ponder'
        self.player_type = self.check_player_type()
        self.players_turn = False
        self.opponent = self.set_opponent()
        self.all_moves = [] #order list acting like queue
        self.board_state = self.init_board_state()

    def check_player_type(self):
        if self.player == '/white':
            self.opponent = '/black'
            return 0
        else:
            self.opponent = '/white'
            return 1 # for blacks
        
    def set_opponent(self):
        if self.player == '/white':
            return '/black'
        else:
            return'/white'

    def close_game(self):
        print("Exiting...")
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
        return self.game_init
      

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
        all_moves = move_msg_list_to_space_del_list(self.all_moves)
        #print("info: ", len(all_moves), all_moves)
        self.conn.write("position startpos moves " + all_moves)
        rospy.sleep(1)
        return
    
    def make_move(self, time=1000):
        self.send_moves()
        self.conn.write("go movetime " + str(time))
        read_data = self.conn.read()
        move_read = re_string_search(self.uci_move_regex,read_data)
        move_msg = make_move_msg(move_read, self.player_type, self.board_state)
        #print(move_read)
        self.all_moves.append(move_msg) #add your move to list
        return move_msg
    
    def update_board_state(self):
        for move in self.all_moves:
            self.board_state = move_to_updated_board_state(move,self.board_state)
        return self.board_state

    def first_turn(self):
        if self.player_type == 0:
            self.players_turn = True
        else:
            self.players_turn = False

    def turn_listener(self):
        rospy.Subscriber(self.opponent+'/done', Empty, self.opponent_finished_turn)
       
    def opponent_finished_turn(self, data):
        #print("Marker 1")
        if data == Empty():
            self.players_turn = True
        return
    
    def opponent_last_move_listener(self):
        name = self.opponent+'/move'
        #print(name)
        rospy.Subscriber(name, Move, self.opponent_last_move)
        return

    def opponent_last_move(self, data):
        if len(self.all_moves) == 0:
            #print("new data!")
            #first move recorded by player
            self.all_moves.append(data) 
        elif data != self.all_moves[-1]:
            #print("new data!")
            #if this is a new move
            self.all_moves.append(data) 
        else:
            #print("stale data")
            return
        
    def init_board_state(self):
        empty_board_dict = create_board_state_dict()
        board_state = init_board_state_dict(empty_board_dict)
        return board_state
    
    def check_move_for_capture(self,move):
        capture_flag = False
        src_id, dst_id = move_to_board_state_ids(move)
        chesspiece_msg = self.board_state[dst_id]["piece"]
        piece_type = chesspiece_msg.kind
        print("capture?",piece_type)
        if piece_type != "":
            print("CAPTURE!")
            capture_flag = True
        return capture_flag, chesspiece_msg



def main():
    rospy.init_node("chess_player")
    player = rospy.get_name()
    device = rospy.get_param(player+'/device')
    print(player, device)
    chess_player = ChessPlayer(player, device)
    rate = rospy.Rate(5)
    rospy.sleep(1)

    move_topic = player+'/move'
    move_pub = rospy.Publisher(move_topic, Move, queue_size=3) #publisher for moves
    chessboard_topic = '/chessboard'
    chessboard_pub = rospy.Publisher(chessboard_topic, Chessboard, queue_size=3)
    capture_topic = player+'/capture'
    capture_pub = rospy.Publisher(capture_topic, Chesspiece, queue_size=3) #publisher for moves
    player_done_topic = player+'/done'
    player_done_pub = rospy.Publisher(player_done_topic, Empty, queue_size=3) #publisher to notify end of turn
 

    print(chess_player.intialize_game())
    if chess_player.game_init:
        rospy.sleep(5)
        #chessboard_pub.publish(create_chessboard_msg(chess_player.board_state)) #show starting board
        if chess_player.player_type == 0:
            print("White has 1st move")
            ### MAke a move
            chess_player.players_turn = True
            move_to_pub= chess_player.make_move()
            #print(move_to_pub)
            print(move_msg_list_to_space_del_list(chess_player.all_moves))
            move_pub.publish(move_to_pub)
            updated_board_state = chess_player.update_board_state()
            chessboard_pub.publish(create_chessboard_msg(updated_board_state))
            player_done_pub.publish(Empty())
            chess_player.players_turn = False
            ### 
        while not rospy.is_shutdown():
            if chess_player.players_turn != True:
               chess_player.turn_listener() #check if it is my turn
               chess_player.opponent_last_move_listener() #check for last move
            else:
                #players turn
                print(player+': Move!')
                #### Make a move
                #print(move_msg_list_to_space_del_list(chess_player.all_moves))
                print("---")
                move_to_pub=chess_player.make_move()
                print(move_msg_list_to_space_del_list(chess_player.all_moves))
                move_pub.publish(move_to_pub)
                capture_flag, captured_piece = chess_player.check_move_for_capture(move_to_pub)
                if capture_flag:
                    capture_pub.publish(captured_piece)
                updated_board_state = chess_player.update_board_state()
                chessboard_pub.publish(create_chessboard_msg(updated_board_state))
                player_done_pub.publish(Empty())
                chess_player.players_turn = False
                ####
            rate.sleep()
   
    chess_player.close_game()

if __name__ == '__main__':
    main()