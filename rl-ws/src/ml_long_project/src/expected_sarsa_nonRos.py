#!/usr/bin/env python

import rospy
from random import randint
#import time
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32MultiArray, Bool
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from scipy.spatial import distance
import copy
import pickle
from getpass import getuser
from nav_msgs.msg import Odometry
import sys
import numpy as np
#from os import getcwd
from datetime import datetime
import pandas as pd
import os

## Global Variables
# mobile_base velocity publisher
command_pub = None
# String command that will be sent to the robot
cmd_msg = String()

# current time
#cur_time = 0
# current state. can be either "init", "drive, "halt", "turn_r", "turn_l"
cur_state = "init"

# Are we training or executing?
train = None
map_name = ""

vel = Twist()
current_position = Pose2D(-2,-2,0)

goal_point =  Pose2D(4,4,0) # Weird coord transform error
path = []
global_map =np.genfromtxt("/home/lelliott/turtlepath/rl-ws/map.csv", delimiter=',')
map = {}

finished = False


def reset_stage():
    global current_position
    current_position = Pose2D(-2,-2,0)

def delete_q(map_name):
    if os.path.exists(map_name + ".sarsa"):
        os.remove(map_name + ".sarsa")
    else:
        print("The file does not exist") 

def master_train(map_name,goal_point,train):
    global map
    delete_q(map_name)
    # train several times in real life to build a map
    count = 0
    accelerate = True
    episode_num = 50
    dt = datetime.now()
    training_data = [[i,0, True] for i in range(episode_num)]
    satisfied = False
    s_count = 0
    while not satisfied:
        s_count +=1
        count = 0
        for i in range(0,episode_num):
            path, crashed = train_sarsa_real_life(map_name,goal_point,train, count,episode_num)
            print(len(path))
            reset_stage()
            
            training_data[int(i)][1] = len(path)
            training_data[int(i)][2] = crashed
            count += 1
        if(len(path) > 19):
            satisfied = False
        else:
            print("Took: " + str(s_count*episode_num))
            satisfied = True

    # save data each count from training to use for plots and analysis
    filepath = "/home/"+getuser()+"/turtlepath/rl-ws/data/"
    filename = "expected_sarsa_" + map_name + "_" + dt.strftime("%Y-%m-%d-%H-%M-%S") + "_c" + str(count)
    np.savetxt(filepath + filename + ".csv", training_data, delimiter=",")

    
    return path

def retrieve_q(map_name):
    # Save out data to map_name.sarsa so we don't have to retrain on familiar maps
    #pickle.dump(map, open("sarsa_data/"+map_name + ".sarsa","wb"))
    try:
        q = pickle.load( open(map_name + ".sarsa", "rb" ) )
        q = dict(q)
    except (OSError, IOError) as e:
        # There are states(x,y) and 4 Actions (up, down, right, left)
        q = {}
    return q

def write_q(map_name, q):
    pickle.dump(q, open(map_name + ".sarsa","wb"))


def train_sarsa_real_life(map_name, goal_point, train, count, max_count):
    # set date which is used in data output filenames
    dt = datetime.now()
    global current_position
    alpha = .6
    gamma = .99
    eps  = .2

    q = retrieve_q(map_name)

    if(count < 2):
        eps = 1 # totally random walk to build a better model
    elif(count == max_count -1):
        eps = .05

    timeout = 0
    path = []
    # a is action
    # 0 is North
    # 1 is East
    # 2 is South
    # 3 is West
    # s is state and equals a point in the map
    s = to_str(current_position)
    # Choose A from S using policy dervied from Q (e.g. e-greedy)
    a = e_greedy(eps, q, s)
    # Loop through episode
    crashed = False
    while timeout < 5000 and s != to_str(goal_point) and not crashed:
        path.append(a)
        # Take action A, observe R,S'
        r, s_prime, crashed = execute_rl(a,s)
        # Choose A' from S' using policy dervied from Q (e.g. e-greedy)
        a_prime = e_greedy(eps, q, s_prime)

        if not (s in q):
            d={0:0, 1:0, 2:0, 3:0}
            q[s] = d

        if not (s_prime in q):
            d={0:0, 1:0, 2:0, 3:0}
            q[s_prime] = d
        
        # Q(S,A) <- Q(S,A) + alpha[R+gamma * Q(S',A')- Q(S,A)]
        expectation = 0
        greedy_actions = 0
        q_max = np.max([q[s_prime][action] for action in q.get(s_prime)]) 
        for actions in q.get(s_prime):
            if(q[s_prime][actions] == q_max):
                greedy_actions +=1
        ng_prob = eps/4
        g_prob = (1-eps)/greedy_actions + ng_prob

        for actions in q.get(s_prime):
            if(q[s_prime][actions] == q_max):
                expectation += q[s_prime][actions] * g_prob
            else:
                expectation  += q[s_prime][actions] * ng_prob

        q[s][a] = q[s][a] + alpha * (r+gamma*expectation - q[s][a])

        # S<- S'; A<-A';
        s = s_prime
        a = a_prime
        timeout+=1

    print(path)
    print("Crashed: " + str(crashed))
    write_q(map_name,q)
    return path, crashed

def to_str(s):
    return("X: "+ str(s.x) + " "+"Y: "+ str(s.y))

def decode_action(a):
    # a is action
    # 0 is North
    # 1 is East
    # 2 is South
    # 3 is West

    if a == 0:
        action = "north"
    elif a == 1: 
        action = "east"
    elif a == 2:
        action = "south"
    else:
        action = "west"
    return action

def e_greedy(eps, q, s):
    # If we have been in this state try to epsilon greedy it
    if s in q:
        p = np.random.random() 
        if p < eps: 
            a = np.random.randint(0,4) 
        else: 
            a = np.argmax([q[s][action] for action in q.get(s)]) 
    # If we've never been here we don't know where to go, so it's random
    else:
        a = np.random.randint(0,4) 
    return a

def decode_string(s):
    if(s[8] == '-'):
        y = -1 * int(s[9])
    elif(len(s) >= 10 and s[9] == '-'):
        y = -1 * int(s[10]) 
    elif(s[3] == '-'):
       y = int(s[9])        
    else:
        y = int(s[8])

    if(s[3] == '-'):
        x = -1 * int(s[4])
    else:
        x = int(s[3])
    
    return x,y

def execute_rl(a,s):
    global map
    global vel
    global current_position
    global finished
    prev = copy.deepcopy(current_position)
    #send_command(decode_action(a))
    x,y = decode_string(s)

    if(a == 0):
        x +=1
    elif(a == 1):
        y -= 1
    elif(a == 2):
        x -=1
    elif(a ==3):
        y +=1

    new_position = Pose2D(x,y,0)
    if not (check_global(new_position)):
        current_position = new_position  
    
    crashed = False
    #print("Next")
    # return it's new location
    reward = -1
    s_prime = to_str(current_position)
    if(s == s_prime):
        reward = -5
    if(crashed):
        reward = -10
    #print("Start: "+s+ " Finish: " + s_prime)
    #print(reward)
    if not (s in map): # If Q is not initalized for our action set it to 0
            d={}
            d[a] = {"state":s_prime,"reward":reward}
            map[s] = d
    elif not (a in map[s]):
        map[s][a] = {"state":s_prime,"reward":reward}
    else:
        map[s][a] = {"state":s_prime,"reward":reward}
    return reward, s_prime, crashed

def check_global(point):
    global global_map
    x = point.x + 5
    y = (point.y*-1 + 5) 
    return bool(global_map[y][x])

master_train("test1", goal_point, True)
print("done")