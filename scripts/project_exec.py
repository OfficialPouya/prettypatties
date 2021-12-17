#!/usr/bin/env python
import copy
import time
import rospy
import sys
import rospkg
import argparse
import cv2
import os

# from geometry_msgs.msg import Point
import numpy as np
from project_header import *
from project_kinematics_func import *
from blob_search_edit import *

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = [0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]
go_away = [270*PI/180.0, -90*PI/180.0, 90*PI/180.0, -90*PI/180.0, -90*PI/180.0, 135*PI/180.0]
# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

    global digital_in_0
    global analog_in_0
    digital_in_0 = msg.DIGIN
    digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
    analog_in_0 = msg.AIN0

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True

"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0   #?????????????????
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            #rospy.loginfo("Goal is reached!")
            at_goal = 1
        
        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
        
        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error

def move_food(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    destination = lab_invk(start_xw_yw_zw[0],start_xw_yw_zw[1],start_xw_yw_zw[2],0)
    high_destination = lab_invk(start_xw_yw_zw[0],start_xw_yw_zw[1],150,0)
    move_arm(pub_cmd, loop_rate, high_destination, vel, accel)
    move_arm(pub_cmd, loop_rate, destination, vel, accel)
    gripper(pub_cmd,loop_rate,suction_on)
    time.sleep(.5)
    if(digital_in_0 == False):
        move_arm(pub_cmd, loop_rate, go_away, vel, accel)
        print("Sorry, we are out of this ingredient right now! Check back later!")
        # kill robot
        gripper(pub_cmd,loop_rate,suction_off)
        return -1

    move_arm(pub_cmd, loop_rate, high_destination, vel, accel)
    time.sleep(.5)
    new_high_destination = lab_invk(target_xw_yw_zw[0],target_xw_yw_zw[1],150,0)
    new_dest = lab_invk(target_xw_yw_zw[0],target_xw_yw_zw[1],target_xw_yw_zw[2],0)
    move_arm(pub_cmd, loop_rate, new_high_destination, vel, accel)
    move_arm(pub_cmd, loop_rate, new_dest, vel, accel)
    gripper(pub_cmd,loop_rate,suction_off)
    move_arm(pub_cmd, loop_rate, new_high_destination, vel, accel)
    error = 0

    return error


def main():

    global go_away

    rospy.init_node('projectnode')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
    sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    # Initialize the rate to publish to ur3/command
    loop_rate = rospy.Rate(SPIN_RATE)

    rospack = rospkg.RosPack()
    path = rospack.get_path('prettypatties')
    image_path = os.path.join(path, 'scripts', 'configs', 'Capture.JPG')
    # Read image
    raw_image = cv2.imread(image_path)

    # Flip the image (Legacy webcam config)
    cv_image = cv2.flip(raw_image, -1)

    # # Call blob search (row, col)
    burger_image_rc = blob_search(cv_image.copy(), 'burger')
    bun_image_rc = blob_search(cv_image.copy(), 'bun')

    bbun_count = 4
    tbun_count = 4
    cheese_count = 1
    burger_count = 3
    tomato_count = 2
    order_pos = 1

    top_bun_1_position = [-210,550,30]
    top_bun_2_position = [-210,660,25]
    top_bun_3_position = [-50,550,30]
    top_bun_4_position = [-50,660,30]
    bottom_bun_1_position = [-210,550,15]
    bottom_bun_2_position = [-210,660,15]
    bottom_bun_3_position = [-50,550,15]
    bottom_bun_4_position = [-50,660,15]
    burger_1_position = [150,450,40]
    burger_2_position = [150,450,32]
    burger_3_position = [150,450,15]
    tomato_1_position = [-210,400,25]
    tomato_2_position = [-210,400,20]
    cheese_1_position = [100,620,5]
    
    order_1_position_base_x = 330
    order_1_position_base_y = 207

    order_2_position_base_x = 286
    order_2_position_base_y = -116

    order_3_position_base_x = -82
    order_3_position_base_y = -265

    position_top_bun_x = 330
    position_top_bun_y = 30
    

    time.sleep(5)

    while(order_pos < 4):

        print("Welcome to the Pretty Patties Resturant. Here is our menu: ")
        print("1: Cheeseburger - Two Buns with a slice of American Cheese, a tomato slice, and an all beef patty")
        print("2: Double Cheeseburger - Two Buns with a slice of American Cheese, a tomato slice, and 2 all beef patties")
        print("3: Triple Cheeseburger - Two Buns with a slice of American Cheese, a tomato slice, and 3 all beef patties")
        print("4: Hamburger - Two Buns and an all beef patty")
        print("5: Double Hamburger - Two Buns, a tomato slice and 2 all beef patties")
        order = input("What would you like to eat?")

        vel = 1.0
        accel = 1.0
        # move_arm(pub_command, loop_rate, go_away, vel, accel)
        # time.sleep(5)
        if(order_pos == 1):
            order_pos = 2
            print("Yum! Once your burger is done, hit Crt+C to exit or order again!")
            # Double Hamburger
            if(order == 5):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_1_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_1_position, [order_1_position_base_x,order_1_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_1_position, [order_1_position_base_x,order_1_position_base_y,10], vel, accel)
                # Burger Patty 2
                move_food(pub_command, loop_rate, burger_2_position, [order_1_position_base_x,order_1_position_base_y,20], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_1_position, [order_1_position_base_x,order_1_position_base_y,35], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_1_position_base_x,order_1_position_base_y,55], vel, accel)
                # Finish Movement
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Cheese Burger
            elif(order == 1):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_1_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_1_position, [order_1_position_base_x,order_1_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_1_position, [order_1_position_base_x,order_1_position_base_y,10], vel, accel)
                # Cheese Slice 
                move_food(pub_command, loop_rate, cheese_1_position, [order_1_position_base_x,order_1_position_base_y,15], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_1_position, [order_1_position_base_x,order_1_position_base_y,40], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_1_position_base_x,order_1_position_base_y,60], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Double Cheeseburger
            elif(order == 2):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_1_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_1_position, [order_1_position_base_x,order_1_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_1_position, [order_1_position_base_x,order_1_position_base_y,10], vel, accel)
                # Burger Patty 2
                move_food(pub_command, loop_rate, burger_2_position, [order_1_position_base_x,order_1_position_base_y,20], vel, accel)
                # Cheese Slice 
                move_food(pub_command, loop_rate, cheese_1_position, [order_1_position_base_x,order_1_position_base_y,30], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_1_position, [order_1_position_base_x,order_1_position_base_y,55], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_1_position_base_x,order_1_position_base_y,75], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Triple Cheeseburger
            elif(order == 3):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_1_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_1_position, [order_1_position_base_x,order_1_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_1_position, [order_1_position_base_x,order_1_position_base_y,10], vel, accel)
                # Burger Patty 2
                move_food(pub_command, loop_rate, burger_2_position, [order_1_position_base_x,order_1_position_base_y,20], vel, accel)
                # Burger Patty 3
                move_food(pub_command, loop_rate, burger_3_position, [order_1_position_base_x,order_1_position_base_y,30], vel, accel)
                # Cheese Slice 
                move_food(pub_command, loop_rate, cheese_1_position, [order_1_position_base_x,order_1_position_base_y,55], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_1_position, [order_1_position_base_x,order_1_position_base_y,65], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_1_position_base_x,order_1_position_base_y,85], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Hamburger
            elif(order == 4):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_1_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_1_position, [order_1_position_base_x,order_1_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_1_position, [order_1_position_base_x,order_1_position_base_y,10], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_1_position, [order_1_position_base_x,order_1_position_base_y,25], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_1_position_base_x,order_1_position_base_y,45], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            else:
                print("Thats not a valid number!")
        elif(order_pos == 2):
            print("Yum! Once your burger is done, hit Crt+C to exit or order again!")
            order_pos = 3
            # Double Hamburger
            if(order == 5):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_2_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_2_position, [order_2_position_base_x,order_2_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_2_position, [order_2_position_base_x,order_2_position_base_y,10], vel, accel)
                # Burger Patty 2
                move_food(pub_command, loop_rate, burger_3_position, [order_2_position_base_x,order_2_position_base_y,20], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_2_position_base_x,order_2_position_base_y,35], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_2_position_base_x,order_2_position_base_y,55], vel, accel)
                # Finish Movement
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Cheese Burger
            elif(order == 1):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_2_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_2_position, [order_2_position_base_x,order_2_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_2_position, [order_2_position_base_x,order_2_position_base_y,10], vel, accel)
                # Cheese Slice 
                move_food(pub_command, loop_rate, cheese_1_position, [order_2_position_base_x,order_2_position_base_y,15], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_2_position_base_x,order_2_position_base_y,40], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_2_position_base_x,order_2_position_base_y,60], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Double Cheeseburger
            elif(order == 2):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_2_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_2_position, [order_2_position_base_x,order_2_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_2_position, [order_2_position_base_x,order_2_position_base_y,10], vel, accel)
                # Burger Patty 2
                move_food(pub_command, loop_rate, burger_3_position, [order_2_position_base_x,order_2_position_base_y,20], vel, accel)
                # Cheese Slice 
                move_food(pub_command, loop_rate, cheese_1_position, [order_2_position_base_x,order_2_position_base_y,30], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_2_position_base_x,order_2_position_base_y,55], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_2_position_base_x,order_2_position_base_y,75], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Triple Cheeseburger
            elif(order == 3):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_2_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_2_position, [order_2_position_base_x,order_2_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_1_position, [order_2_position_base_x,order_2_position_base_y,10], vel, accel)
                # Burger Patty 2
                move_food(pub_command, loop_rate, burger_2_position, [order_2_position_base_x,order_2_position_base_y,20], vel, accel)
                # Burger Patty 3
                move_food(pub_command, loop_rate, burger_3_position, [order_2_position_base_x,order_2_position_base_y,30], vel, accel)
                # Cheese Slice 
                move_food(pub_command, loop_rate, cheese_1_position, [order_2_position_base_x,order_2_position_base_y,55], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_2_position_base_x,order_2_position_base_y,65], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_2_position_base_x,order_2_position_base_y,85], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Hamburger
            elif(order == 4):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_2_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_2_position, [order_2_position_base_x,order_2_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_2_position, [order_2_position_base_x,order_2_position_base_y,10], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_2_position_base_x,order_2_position_base_y,25], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_2_position_base_x,order_2_position_base_y,45], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            else:
                print("Thats not a valid number!")
        elif(order_pos == 3):
            print("Yum! Once your burger is done, hit Crt+C to exit or order again!")
            order_pos = 4
            # Double Hamburger
            if(order == 5):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_3_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_3_position, [order_3_position_base_x,order_3_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_3_position, [order_3_position_base_x,order_3_position_base_y,10], vel, accel)
                # Burger Patty 2
                move_food(pub_command, loop_rate, burger_3_position, [order_3_position_base_x,order_3_position_base_y,20], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_3_position_base_x,order_3_position_base_y,35], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_3_position_base_x,order_3_position_base_y,55], vel, accel)
                # Finish Movement
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Cheese Burger
            elif(order == 1):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_3_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_3_position, [order_3_position_base_x,order_3_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_3_position, [order_3_position_base_x,order_3_position_base_y,10], vel, accel)
                # Cheese Slice 
                move_food(pub_command, loop_rate, cheese_1_position, [order_3_position_base_x,order_3_position_base_y,15], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_3_position_base_x,order_3_position_base_y,40], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_3_position_base_x,order_3_position_base_y,60], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Double Cheeseburger
            elif(order == 2):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_3_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_3_position, [order_3_position_base_x,order_3_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_3_position, [order_3_position_base_x,order_3_position_base_y,10], vel, accel)
                # Burger Patty 2
                move_food(pub_command, loop_rate, burger_3_position, [order_3_position_base_x,order_3_position_base_y,20], vel, accel)
                # Cheese Slice 
                move_food(pub_command, loop_rate, cheese_1_position, [order_3_position_base_x,order_3_position_base_y,30], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_3_position_base_x,order_3_position_base_y,55], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_3_position_base_x,order_3_position_base_y,75], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Triple Cheeseburger
            elif(order == 3):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_3_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_3_position, [order_3_position_base_x,order_3_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_1_position, [order_3_position_base_x,order_3_position_base_y,10], vel, accel)
                # Burger Patty 2
                move_food(pub_command, loop_rate, burger_2_position, [order_3_position_base_x,order_3_position_base_y,20], vel, accel)
                # Burger Patty 3
                move_food(pub_command, loop_rate, burger_3_position, [order_3_position_base_x,order_3_position_base_y,30], vel, accel)
                # Cheese Slice 
                move_food(pub_command, loop_rate, cheese_1_position, [order_3_position_base_x,order_3_position_base_y,55], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_3_position_base_x,order_3_position_base_y,65], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_3_position_base_x,order_3_position_base_y,85], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            # Hamburger
            elif(order == 4):
                # Top Bun 
                move_food(pub_command, loop_rate, top_bun_3_position, [position_top_bun_x,position_top_bun_y,0], vel, accel)
                # Bottom Bun
                move_food(pub_command, loop_rate, bottom_bun_3_position, [order_3_position_base_x,order_3_position_base_y,0], vel, accel)
                # Burger Patty
                move_food(pub_command, loop_rate, burger_3_position, [order_3_position_base_x,order_3_position_base_y,10], vel, accel)
                # Tomato
                move_food(pub_command, loop_rate, tomato_2_position, [order_3_position_base_x,order_3_position_base_y,25], vel, accel)
                # Top Bun
                move_food(pub_command, loop_rate, [position_top_bun_x,10,10], [order_3_position_base_x,order_3_position_base_y,45], vel, accel)
                # Home
                move_arm(pub_command, loop_rate, go_away, vel, accel)
            else:
                print("Thats not a valid number!")
    rospy.spin()

if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass