#!/usr/bin/env python

import rospy
from random import randint
#import time
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.spatial import distance
import copy
import pickle
from getpass import getuser
from os import getcwd

## Global Variables
# mobile_base velocity publisher
command_pub = None
# String command that will be sent to the robot
cmd_msg = String()

# current time
#cur_time = 0
# current state. can be either "init", "plan", "execute"
cur_state = "init"

# Are we training or executing?
train = None
map_name = ""

# the map and important points
map = []
start_point = Pose2D(7,7,0)
goal_point =  Pose2D(1,1,0)

# a proxy for the map identifying which cells have already been visited.
# will be used to fill the map with 0s at the start of each episode.
visited_reset = None

# the generated set of points for the robot to follow
path = []
# the set of commands which tell the robot how to follow the path
cmds = []

def check_state():
    global cur_state
    global map, train, map_name
    global start_point,goal_point, path, cmds
    if cur_state == "init":
        if(len(map) > 0):
            cur_state = "plan"

    elif cur_state == "plan":
        print("Train? " + str(train))
        path = train_ql(map_name, map, start_point, goal_point, train)
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


def train_ql(map_name, map, start_point, goal_point, train):
    alpha = 0.05
    gamma = 0.9
    eps  = 0.2

    # Save out data to map_name.ql so we don't have to retrain on familiar maps
    #pickle.dump(map, open("ql_data/"+map_name + ".ql","wb"))
    # Load prior Q or init to zero
    try:
        q = pickle.load( open("/home/"+getuser()+"/turtlepath/rl-ws/" + map_name + ".ql", "rb" ) ) 
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
        count_num = 5 #was 10
        episode_num = 1000 #was 5000
    else:
        count_num = 1
        episode_num = 1

    while count < count_num:
        episode = 0
        if(count == count_num-1):
            print("Turning off epsilon for max greedy")
            eps  = 0

        while episode < episode_num:
            # print progress to the console
            if episode % 250 == 0:
                print("Starting episode " + str(episode) + " of run " + str(count))
            # reset our episode-dependent variables
            timeout = 0
            path = []
            visited = visited_reset
            # a is an action: can be 0 (North), 1 (East), 2 (South), or 3 (West)
            # s is a state, where (s.x, s.y) is a point in the map
            s = start_point
            # Choose A from S using policy derived from Q (e.g. e-greedy)
            a = e_greedy(eps, q, s)
            path.append(a)
            # Loop through episode
            while timeout < 5000 and s != goal_point:
                # mark the current state as visited
                visited[s.x][s.y] = True

                # Take action A, observe R, S'
                r, s_prime = execute(a, s, map, visited)
                
                # Choose A' from S' using policy dervied from Q (e.g. e-greedy)
                a_prime = e_greedy(eps, q, s_prime)

                # Q(S,A) <- Q(S,A) + alpha*[R + gamma * max_{any A' for S'}{Q(S',A')} - Q(S,A)]
                q[s.x,s.y,a] = q[s.x,s.y,a] + alpha * (r+gamma*max_q(q,s_prime) - q[s.x,s.y,a])
                # S<- S'; A<-A';
                s = s_prime
                a = a_prime
                path.append(a)
                # increment timeout and stop the episode if it goes 5000 cycles without reaching the goal
                timeout += 1
            episode += 1

        print(path)
        count += 1
    pickle.dump(q, open("/home/"+getuser()+"/turtlepath/rl-ws/" + map_name + ".ql","wb"))
    #print("Working Directory is ", getcwd())
    return path

# def sarsa(map,start_point,goal_point):
#     return 0

def max_q(q, s_prime):
    #print("finding max q for ", s_prime.x, s_prime.y)
    # after we take action A and arrive at state S', 
    #   we will find the highest Q value for S' with any available action A'.
    max_q = 0
    for a_prime in range(4):
        if q[s_prime.x, s_prime.y, a_prime] > max_q:
            max_q = q[s_prime.x, s_prime.y, a_prime]
    return max_q

def e_greedy(eps, q, s):
    p = np.random.random() 
    if p < eps: 
      a = np.random.randint(0,4) 
    else: 
      a = np.argmax([action for action in q[s.x,s.y]]) 
    return a

def execute(action, state, map, visited):
    # Reward is -1 each timestep, with the potential for additional punishments:
    # - Trying to move into a wall or out of bounds will fail,
    #   the robot will not move and will incur a reward of -5.
    # - Returning to a cell already visited this episode will work,
    #   but will incur a reward of -5 to encourage efficient path planning.

    if(action == 1): # East
        if((state.x + 1) < map.shape[0] and map[state.y,state.x+1] != 1):
            # this move is allowed
            r = -1
            if visited[state.x + 1][state.y] == True:
                # this cell has already been visited
                r = -3
            s_prime = Pose2D(state.x + 1,state.y,0)
        else:
            # this move is not allowed (would move into a wall or out of bounds)
            s_prime = state 
            r = -10
    elif(action == 2): # South
        if((state.y + 1) < map.shape[0] and map[state.y+1, state.x] != 1):
            # this move is allowed
            r = -1
            if visited[state.x][state.y + 1] == True:
                # this cell has already been visited
                r = -3
            s_prime = Pose2D(state.x,state.y+1,0)
        else:
            # this move is not allowed (would move into a wall or out of bounds)
            s_prime = state 
            r = -10
    elif(action == 3): # West
        if((state.x - 1) > 0 and map[state.y, state.x-1] != 1):
            # this move is allowed
            r = -1
            if visited[state.x - 1][state.y] == True:
                # this cell has already been visited
                r = -3
            s_prime = Pose2D(state.x -1,state.y,0)
        else:
            # this move is not allowed (would move into a wall or out of bounds)
            s_prime = state 
            r = -10
    else: # North
        if((state.y - 1) > 0 and map[state.y-1, state.x] != 1):
            # this move is allowed
            r = -1
            if visited[state.x][state.y - 1] == True:
                # this cell has already been visited
                r = -3
            s_prime = Pose2D(state.x,state.y-1,0)
        else:
            # this move is not allowed (would move into a wall or out of bounds)
            s_prime = state 
            r = -10
    # check if we have arrived at the goal (BEEG reward to make it all worth it)
    if s_prime == goal_point:
        r = 200
    return r, s_prime

def get_map(map_msg):
    global map, visited_reset
    #print(map_msg.info)
    # Take in the full map and put it into numpy
    big_map = (np.asarray(map_msg.data))
    big_map = np.reshape(big_map, (map_msg.info.height,map_msg.info.width))
    
    print(map_msg.info.origin)

    # Create a downsampled map
    resolution = map_msg.info.resolution
    small_map = np.zeros((int(map_msg.info.width*resolution)+1,int(map_msg.info.width*resolution)+1))
    
    # Save it out to view it as a csv
    #np.savetxt("/home/"+getuser()+"/turtlepath/rl-ws/demofile2.csv", big_map, delimiter=",")

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
    # create the boolean array which will denote whether a cell on the map has been visited
    visited_reset = [[False] * map.shape[0]] * map.shape[1]


def send_command(keyword):
    cmd_msg.data = keyword
    command_pub.publish(cmd_msg)

def update_state(timer_event):
    # at each timer step, update the state and send an action
    check_state()

def main():
    global command_pub, train, map_name

    # initialize node
    rospy.init_node('q_learning')
    
    # get variables from the launch file
    train = rospy.get_param('~train')
    map_name = rospy.get_param('~map_name')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/tp/cmd", String, queue_size=1)

    # subscribe to the map.
    rospy.Subscriber('/map', OccupancyGrid, get_map, queue_size=1)
    # subscribe to the custom topic '/tp/request', which will 
    #   trigger when the control_node requests the next command.
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