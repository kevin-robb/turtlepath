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

goal_point =  Pose2D(4,4,0) # Weird coord transform error
path = []
cmds = []
map = {}

finished = False


def reset_stage():
    send_command('reset')
    rospy.sleep(1)
    send_command('reset')
    rospy.sleep(1)
    send_command('reset')
    rospy.sleep(1)
    send_command('reset')

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
        #print("Train? " + str(train))
        #path = train_sarsa_accel(map_name, map,start_point,goal_point,train)
        path = master_train(map_name,goal_point,train)
       
        if(len(path) > 0):
            cur_state = "execute"
            path_to_cmd()
    elif(cur_state == 'execute'):
        #print ("Executing learned path")
        #print(path)
        #print(cmds)
        cur_state = 'done'

def master_train(map_name,goal_point,train):
    # train several times in real life to build a map
    count = 0
    accelerate = True
    for i in range(0,100):
        path = train_sarsa_real_life(map_name,goal_point,train, count,100)
        print(len(path))
        reset_stage()
        if(accelerate):
            print("Accelerating Training")
            # This path is always "the best found" since epsilon is zero on the last run
            path = train_sarsa_accel(map_name, goal_point, 100)
        print(len(path))
        count += 1

    
    return path
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
        q = pickle.load( open(map_name + ".sarsa", "rb" ) )
        q = dict(q)
    except (OSError, IOError) as e:
        # There are states(x,y) and 4 Actions (up, down, right, left)
        q = {}
    return q

def write_q(map_name, q):
    pickle.dump(q, open(map_name + ".sarsa","wb"))

def train_sarsa_accel(map_name, goal_point, iter_val):
    global current_position

    alpha = .5
    gamma = .9
    eps  = .2
    # Episode
    episode = 0
    # Want to see the path count_num times, which happens every episode_num iterations
    episode_num = iter_val

    q = retrieve_q(map_name)
   
    while episode < episode_num:
        if(episode == episode_num-1):
            #print("Turning off epsilon for max greedy")
            eps  = 0


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
        # We don't monitor crashed in simulation since it's hard to monitor (and shouldn't happen)
        while timeout < 5000 and s != to_str(goal_point):
            # Take action A, observe R,S'
            r, s_prime = execute_sim(a,s,q)
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
            timeout +=1

        episode +=1

    write_q(map_name,q)
    return path

def train_sarsa_real_life(map_name, goal_point, train, count, max_count):
    global current_position
    alpha = .5
    gamma = .9
    eps  = .2

    q = retrieve_q(map_name)

    if(count < 2):
        eps = 1 # totally random walk to build a better model
    elif(count == max_count -1):
        eps = 0

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
    while timeout < 1000 and s != to_str(goal_point) and not crashed:
        # Take action A, observe R,S'
        r, s_prime, crashed = execute_rl(a,s)
        # Choose A' from S' using policy dervied from Q (e.g. e-greedy)
        a_prime = e_greedy(eps, q, s_prime)

        if not (s in q): # If Q is not initalized for our action set it to 0
            d={0:0, 1:0, 2:0, 3:0}
            q[s] = d
       # elif not a in q[s]:
           # q[s][a] = 0
        if not (s_prime in q):
            d={0:0, 1:0, 2:0, 3:0}
           # d[a_prime] = 0
            q[s_prime] = d
        #elif not a_prime in q[s_prime]:
         #   q[s_prime][a_prime] = 0
    
        
        # Q(S,A) <- Q(S,A) + alpha[R+gamma * Q(S',A')- Q(S,A)]
        q[s][a] = q[s][a] + alpha * (r+gamma*q[s_prime][a_prime] - q[s][a])

        # S<- S'; A<-A';
        s = s_prime
        a = a_prime
        path.append(a)

    print(path)
    print("Crashed: " + str(crashed))
    write_q(map_name,q)
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
            a = np.argmax([q[s][action] for action in q.get(s)]) 
    # If we've never been here we don't know where to go, so it's random
    else:
        a = np.random.randint(0,4) 
    return a

def execute_sim(action, state, q):
    global map
    # IMPORTANT: Execute does not factor in heading. It is only based on finding a coordinate path similar to A* and Potential Fields
    # Ie North goes down a y val, East increments x etc
    # Reward is always -1. Unless it tries to go into a wall, it doesn't move and gets a -5
    if (not (state in map)) or (not (action in map[state])):
        # we haven't ever seen the requested action in s
        # thus it's too risky to actually take
        # but we don't want to give it a
        # strong negative rewards such that it never gets taken
        r = -1
        s_prime = state
    else:
        s_prime = map[state][action]['state']
        r = map[state][action]['reward']
    return r, s_prime

def execute_rl(a,s):
    global map
    global vel
    global current_position
    global finished
    prev = copy.deepcopy(current_position)
    send_command(decode_action(a))

    count = 0
    crashed = False
    crashed_count = 0
    while count < 5 and crashed_count < 50:
        if (abs(vel.linear.x) < .1 and abs(vel.angular.z) < .5):
            count +=1
        else:
            count =0
        crashed = abs(vel.linear.x) > .5 and current_position != prev
        if(crashed):
            crashed_count +=1
        else:
            crashed_count =0
        rospy.sleep(.1)  
   
    #print("Next")
    # return it's new location
    reward = -1
    s_prime = to_str(current_position)
    if(s == s_prime):
        reward = -5
    if(crashed):
        reward = -1000
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

def action_finished(msg):
    global finished
    finished = msg.data

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
    rospy.Subscriber('/tp/finish', Bool, action_finished, queue_size=1)

    # Set up a timer to update robot's drive state at 1 Hz
    rospy.Timer(rospy.Duration(secs=.25), update_state)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass