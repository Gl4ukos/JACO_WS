#!/usr/bin/env python3
import chess
import chess.engine
import chess.uci
import rospy
import time
from std_msgs.msg import String
import inputs
import threading
import sys
import termios
import tty
import select

STOCKFISH_PATH = "/usr/games/stockfish"
arm_status = "ready"
mode ='NONE'
mutex=False

current_keys = set()  # Set to keep track of currently pressed keys


"""Clears all input from stdin."""
def clear_input():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        while select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def switch_mode(new_mode):
    global mutex
    global mode 
    if(mutex==False):
        mutex = True
        mode = new_mode
        mutex=False
        rospy.loginfo("MODE SWITCHED TO: "+ new_mode)

    else:
        rospy.loginfo("CANNOT CHANGE MODE, MUTEX DOWN (change mode during a move)")

    clear_input()

def listen_for_keys():
    global mode, current_keys
    while True:
        events = inputs.get_key()
        for event in events:
            if event.ev_type == 'Key':
                if event.state == 1:  # Key pressed
                    current_keys.add(event.code)
                elif event.state == 0:  # Key released
                    current_keys.discard(event.code)
                
                if 'KEY_LEFTCTRL' in current_keys:
                    if 'KEY_A' in current_keys:
                        switch_mode('AUTO')
                    elif 'KEY_M' in current_keys:
                        switch_mode('MANUAL')
                    elif 'KEY_N' in current_keys:
                        switch_mode('NONE')
                

key_listener_thread = threading.Thread(target=listen_for_keys, daemon=True)
key_listener_thread.start()




def get_next_move(engine):
    return engine.go()

def update_arm_status(data):
    global arm_status
    arm_status= data.data

def parse_move(move_str):
    start_square = move_str[:2]
    end_square = move_str[2:]

    start_square_uci = chess.SQUARE_NAMES.index(start_square)


def enter_play_routine():
    global arm_status
    global mode, mutex
    board = chess.Board()

    engine = chess.uci.popen_engine(STOCKFISH_PATH)
    engine.uci()

    pub = rospy.Publisher('/jaco_driver/chess_moves', String, queue_size=10)
    sub = rospy.Subscriber('/jaco_driver/arm_movement_status', String, update_arm_status)
    rospy.init_node('move_generator', anonymous=True)
    
    while not(rospy.is_shutdown()):

        rospy.loginfo("ARM STATUS: " + arm_status)

        time.sleep(2)
        if(arm_status=="ready" and mutex==False):
        
            if(mode=='MANUAL'):
                mutex=True
                try:
                    move_str = input("Type a move: ")
                    
                    move = chess.Move.from_uci(move_str)
                    if(board.piece_at(move.to_square)):
                        rospy.loginfo("CAPTURE")
                        capture_move_str = move_str[2] + move_str[3] + 'i' +'9'
                        pub.publish(capture_move_str)
                        rospy.loginfo("CAP MOVE: "+capture_move_str)
                        time.sleep(2)

                        arm_status="busy" #letting the arm move
                        while(arm_status=="busy"): #wait for arm to finish capture_move
                            rospy.loginfo("ARM STATUS: " + arm_status)
                            time.sleep(1)

                    
                    pub.publish(move_str)
                    rospy.loginfo("ORD MOVE: "+ move_str)
                    board.push(move)
                    rospy.loginfo("move generated")
                    print(board)

                except ValueError:
                    pub.publish(move_str)
                finally:
                    mutex=False

            elif(mode=='AUTO'):
                mutex = True
                engine.position(board)
                engine_move = get_next_move(engine)
                engine_move_str = str(engine_move[0])
                move = chess.Move.from_uci(engine_move_str)           
    

                if(board.piece_at(move.to_square)):
                    rospy.loginfo("CAPTURE")
                    capture_move_str = engine_move_str[2] + engine_move_str[3] + 'i' + '9'
                    pub.publish(capture_move_str)
                    rospy.loginfo("CAP MOVE: "+ capture_move_str)
                    time.sleep(2)

                    arm_status="busy"
                    while(arm_status=="busy"):
                        rospy.loginfo("ARM_STATUS: "+ arm_status)
                        time.sleep(1)


                pub.publish(engine_move_str)
                board.push(move)
                rospy.loginfo("move generated")
                print(board)
                mutex=False
            else:
                rospy.loginfo("No move generation mode given")
                rospy.loginfo("Press Ctr+A: switch to AUTO  -  Ctr+M: switch to MANUAL  -  Ctr+N: To NONE")
                while(mode=='NONE'):
                    time.sleep(1)
        
            
       

enter_play_routine() 




