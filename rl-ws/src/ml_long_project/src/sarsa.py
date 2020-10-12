#!/usr/bin/env python

import rospy
from random import randint
#import time
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32MultiArray, Bool
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.spatial import distance
import copy
import pickle
from getpass import getuser

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

map=[]
start_point = Pose2D(7,7,0)
goal_point =  Pose2D(1,1,0)
path = []
cmds = []

def get_scan_ranges(scan_msg):
    global fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan
    # format is [fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan]

    # update the important entries
    fwd_scan = scan_msg.data[0] # directly forward
    rear_scan = scan_msg.data[1] # directly behind
    l45_scan = scan_msg.data[2] # ~45 degrees left
    l_scan = scan_msg.data[3] # 90 degrees left
    r45_scan = scan_msg.data[4] # ~45 degrees right
    r_scan = scan_msg.data[5] # 90 degrees right

def check_state():
    global cur_state
    global map, train, map_name
    global start_point,goal_point, path, cmds
    if cur_state == "init":
        if(len(map) > 0):
            cur_state = "plan"

    elif cur_state == "plan":
        print("Train? " + str(train))
        path = train_sarsa(map_name, map,start_point,goal_point,train)
        if(len(path) > 0):
            cur_state = "execute"
            path_to_cmd()
    elif(cur_state == 'execute'):
        print ("Executing learned path")
        print(path)
        print(cmds)
        cur_state = 'done'

def send_next_cmd(req_status):
    # when the next command is requested, send it and remove it from the list.
    global cmds
    if cur_state == "done" and len(cmds) > 0: 
        # only try to send a command if they are ready.
        cmd = cmds.pop(0)
        print("Sending command: ", cmd)
        send_command(cmd)

# turn the path (set of points) into a set of commands.
def path_to_cmd():
    # build up a list of commands from the path
    global cmds,path
    cmds = []
    while len(path) > 0:
        next = path.pop(0)
        if next == 0:
            cmds.append("forward")
        elif next == 1:
            cmds.append("turn_right")
            cmds.append("forward")
            cmds.append("turn_left")
        elif next == 3:
            cmds.append("turn_left")
            cmds.append("forward")
            cmds.append("turn_right")
        elif next == 2:
            cmds.append("turn_180")
            cmds.append("forward")
            cmds.append("turn_180")
  
    print("path_to_cmd finished.")
    print(cmds)


def train_sarsa(map_name, map,start_point, goal_point, train):
    alpha = .5
    gamma = .9
    eps  = .2

    # Save out data to map_name.sarsa so we don't have to retrain on familiar maps
    #pickle.dump(map, open("sarsa_data/"+map_name + ".sarsa","wb"))
    # Load prior Q or init to zero
    try:
        q = pickle.load( open(map_name + ".sarsa", "rb" ) ) 
    except (OSError, IOError) as e:
        # There are states(x,y) and 4 Actions (up, down, right, left)
        q = np.ones((map.shape[1], map.shape[0], 4))
        q[goal_point.x, goal_point.y, 0] = 0
        q[goal_point.x, goal_point.y, 1] = 0
        q[goal_point.x, goal_point.y, 2] = 0
        q[goal_point.x, goal_point.y, 3] = 0

    # Episode
    count = 0
    # Want to see the path every certain number of episodes
    if(train):
        count_num = 10
        episode_num = 5000
    else:
        count_num = 1
        episode_num = 1

    while count < count_num:
        episode = 0
        if(count == count_num-1):
            print("Turning off epsilon for max greedy")
            eps  = 0

        while episode < episode_num:
            timeout = 0
            path = []
            # a is action
            # 0 is North
            # 1 is East
            # 2 is South
            # 3 is West
            # s is state and equals a point in the map
            s = start_point
            # Choose A from S using policy dervied from Q (e.g. e-greedy)
            a = e_greedy(eps, q, s)
            path.append(a)
            # Loop through episode
            while timeout < 5000 and s != goal_point:
                # Take action A, observe R,S'
                r, s_prime = execute(a,s,map)
                
                # Choose A' from S' using policy dervied from Q (e.g. e-greedy)
                a_prime = e_greedy(eps, q, s_prime)

                # Q(S,A) <- Q(S,A) + alpha[R+gamma * Q(S',A')- Q(S,A)]
                q[s.x,s.y,a] = q[s.x,s.y,a] + alpha * (r+gamma*q[s_prime.x,s_prime.y,a_prime] - q[s.x,s.y,a])
                # S<- S'; A<-A';
                s = s_prime
                a = a_prime
                path.append(a)
            episode +=1

        print(path)
        count +=1
    pickle.dump(q, open(map_name + ".sarsa","wb"))
    return path

def sarsa(map,start_point,goal_point):
    return 0

def e_greedy(eps, q, s):
    p = np.random.random() 
    if p < eps: 
      a = np.random.randint(0,4) 
    else: 
      a = np.argmax([action for action in q[s.x,s.y]]) 
    return a

def execute(action, state, map):
    # IMPORTANT: Execute does not factor in heading. It is only based on finding a coordinate path similar to A* and Potential Fields
    # Ie North goes down a y val, East increments x etc
    # Reward is always -1. Unless it tries to go into a wall, it doesn't move and gets a -5
    if(action == 1): # East
        if((state.x + 1) < map.shape[0] and map[state.y,state.x+1] != 1):
            r = -1
            s_prime = Pose2D(state.x + 1,state.y,0)
        else: 
            s_prime = state 
            r = -5
    elif(action == 2): # South
        if((state.y + 1) < map.shape[0] and map[state.y+1, state.x] != 1):
            r = -1
            s_prime = Pose2D(state.x,state.y+1,0)
        else: 
            s_prime = state 
            r = -5
    elif(action == 3): # West
        if((state.x - 1) > 0 and map[state.y, state.x-1] != 1):
            r = -1
            s_prime = Pose2D(state.x -1,state.y,0)
        else: 
            s_prime = state 
            r = -5    
    else: # North
        if((state.y - 1) > 0 and map[state.y-1, state.x] != 1):
            r = -1
            s_prime = Pose2D(state.x,state.y-1,0)
        else: 
            s_prime = state 
            r = -5

    return r, s_prime
def get_map(map_msg):
    global map
    #print(map_msg.info)
    # Take in the full map and put it into numpy
    big_map = (np.asarray(map_msg.data))
    big_map = np.reshape(big_map, (map_msg.info.height,map_msg.info.width))
    
    print(map_msg.info.origin)

    # Create a downsampled map
    resolution = map_msg.info.resolution
    small_map = np.zeros((int(map_msg.info.width*resolution)+1,int(map_msg.info.width*resolution)+1))
    
    # Save it out to view at as a csv
    #np.savetxt("/home/lelliott/turtlepath/rl-ws/demofile2.csv", big_map, delimiter=",")

    for x in range(small_map.shape[0]):
        for y in range(small_map.shape[1]):
            big_x = int(x/resolution)
            big_y = int(y/resolution+1)
            # Weird Conversion error?
            if(big_y >= big_map.shape[1]):   
                big_y = big_map.shape[1]-1
            if(big_y == 1):
                big_y=0
            # Add a 1 in small map if not 0
            small_map[y,x] = int( big_map[big_y,big_x] != 0)
        
    # Small map is a downsampled map at the 1m x 1m "vertexs" of big map
    small_map = np.fliplr(np.flipud(small_map.transpose()))
    print(small_map)
    map = small_map
    np.savetxt("/home/"+getuser()+"/turtlepath/rl-ws/map.csv", map, delimiter=",")


def send_command(keyword):
    cmd_msg.data = keyword
    command_pub.publish(cmd_msg)

def update_state(timer_event):
    # at each timer step, update the state and send an action
    check_state()

    # tell us the current state for debug
    #print(cur_state)
    #print([fwd_scan, l_scan, rear_scan, r_scan])

def main():
    global command_pub, train, map_name

    # initialize node
    rospy.init_node('sarsa')
    
    train = rospy.get_param('~train')
    map_name = rospy.get_param('~map_name')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/tp/cmd", String, queue_size=1)

    # subscribe to the grouped scan values
    # format is [fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan]
    rospy.Subscriber('/tp/scan', Int32MultiArray, get_scan_ranges, queue_size=1)
    rospy.Subscriber('/map', OccupancyGrid, get_map, queue_size=1)
    rospy.Subscriber('/tp/request', Bool, send_next_cmd, queue_size=1)

    # Set up a timer to update robot's drive state at 1 Hz
    rospy.Timer(rospy.Duration(secs=.25), update_state)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass