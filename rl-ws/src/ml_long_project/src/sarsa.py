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
from nav_msgs.msg import Odometry

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
current_position = Pose2D(0,0,0)

goal_point =  Pose2D(6,6,0) # Weird coord transform error
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
    global train, map_name
    global goal_point, path, cmds
    if cur_state == "init":
        cur_state = "plan"

    elif cur_state == "plan":
        print("Train? " + str(train))
        #path = train_sarsa_accel(map_name, map,start_point,goal_point,train)
        path = train_sarsa_real_life(map_name,goal_point,train)

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
            cmds.append("north")
        elif next == 1:
            cmds.append("east")
        elif next == 3:
            cmds.append("west")
        elif next == 2:
            cmds.append("south")

  
    print("path_to_cmd finished.")
    print(cmds)

def retrieve_q(map_name):
    # Save out data to map_name.sarsa so we don't have to retrain on familiar maps
    #pickle.dump(map, open("sarsa_data/"+map_name + ".sarsa","wb"))
    try:
        q, count = pickle.load( open(map_name + ".sarsa", "rb" ) ) 
    except (OSError, IOError) as e:
        # There are states(x,y) and 4 Actions (up, down, right, left)
        q = {}
        count = 0
    return q, count

def write_q(map_name, q, count):
    pickle.dump((q,count), open(map_name + ".sarsa","wb"))

def train_sarsa_accel(map_name, map,start_point, goal_point, train):
    alpha = .5
    gamma = .9
    eps  = .2
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
    return path

def train_sarsa_real_life(map_name, goal_point, train):
    global current_position
    alpha = .5
    gamma = .9
    eps  = .2

    q, count = retrieve_q(map_name)

    if(count < 3):
        eps = 1 # totally random walk to build a better model
    
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
    path.append(a)
    # Loop through episode
    crashed = False
    while timeout < 50 and s != to_str(goal_point) and not crashed:
        # Take action A, observe R,S'
        r, s_prime, crashed = execute_rl(a,s)
        # Choose A' from S' using policy dervied from Q (e.g. e-greedy)
        a_prime = e_greedy(eps, q, s_prime)

        if not (s in q): # If Q is not initalized for our action set it to 0
            d={}
            d[a] = 0
            q[s] = d
        elif not a in q[s]:
            q[s][a] = 0
        if not (s_prime in q):
            d={}
            d[a_prime] = 0
            q[s_prime] = d
        elif not a_prime in q[s_prime]:
            q[s_prime][a_prime] = 0
    
        
        # Q(S,A) <- Q(S,A) + alpha[R+gamma * Q(S',A')- Q(S,A)]
        q[s][a] = q[s][a] + alpha * (r+gamma*q[s_prime][a_prime] - q[s][a])

        # S<- S'; A<-A';
        s = s_prime
        a = a_prime
        path.append(a)

    print(path)
    write_q(map_name,q, count)
    return path

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
            a = np.argmax([action for action in q.get(s)]) 
    # If we've never been here we don't know where to go, so it's random
    else:
        a = np.random.randint(0,4) 
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

def execute_rl(a,s):
    global vel
    global current_position
    send_command(decode_action(a))
    rospy.sleep(2) # Sleep for half a second
    count = 0
    crashed = False
    prev = current_position
    while count < 5 and not crashed:
      if(((abs(vel.linear.x) < .05) and abs(vel.angular.z) < .05)):
          count +=1
      else:
          count = 0
      crashed = abs(vel.linear.x) > .5 and current_position == prev
      rospy.sleep(.01)  
    # return it's new location
    reward = -1
    cp = to_str(current_position)
    if(s == cp):
        reward = -5
    if(crashed):
        reward = -10
    print("Start: "+s+ " Finish: " + cp)
    print(reward)
    return reward, cp, crashed


def check_odom(odom_msg):
    global vel
    global current_position
    current_position.x = round(odom_msg.pose.pose.position.x)
    current_position.y = round(odom_msg.pose.pose.position.y) # round to nearest int
    vel = odom_msg.twist.twist
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
    rospy.Subscriber('/tp/request', Bool, send_next_cmd, queue_size=1)
    rospy.Subscriber('/odom', Odometry, check_odom, queue_size=1)

    # Set up a timer to update robot's drive state at 1 Hz
    rospy.Timer(rospy.Duration(secs=.25), update_state)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass