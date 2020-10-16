#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
from scipy.spatial import distance
from getpass import getuser

## Global Variables
# mobile_base velocity publisher
command_pub = None
# String command that will be sent to the robot
cmd_msg = String()
path = None

# current state. can be either "init", "plan", "execute"
cur_state = "init"
cmds = []

map = None
start_point = Pose2D(7,7,0)
goal_point =  Pose2D(1,1,0)

def check_state():
    global cur_state
    global map
    global start_point,goal_point,path
    
    if cur_state == "init":
        if(len(map) > 0):
            cur_state = "planning"
    # plan a path using the map
    elif cur_state == "planning":
        print("Planning!")
        if(len(map) > 0):
            cur_state = "planning"
        path = potential_fields(map,start_point,goal_point)
        if(len(path) > 0):
            print("Pathing")
            print(path)
            print("Executing!")
            # follow the path
            path_to_cmd()
            # wait to set the state until path_to_cmd finishes, because
            #   we will use it to check and make sure the commands are ready.
            cur_state = "executing"

# http://kovan.ceng.metu.edu.tr/~asil/old/_1./wh2.html
def potential_fields(map,start_point,goal_point):
    epsilon = 1
    n = 5
    p0 = 3

    obs = []
    potentials = np.zeros(map.shape)
    for x in range(map.shape[0]):
        for y in range(map.shape[1]):
            if(x != 0 and x != map.shape[0]-1 and y != 0 and y != map.shape[1]-1):
                if(map[x,y] == 1):
                    obs.append(Pose2D(x,y,0))

    for x in range(map.shape[0]):
        for y in range(map.shape[1]):
            f_repulse = 0
            for o in obs:
                dist = distance.euclidean((o.x,o.y), (x,y))
                if( dist <= p0):
                    f = n*.5*(1/dist - 1/p0)*(1/dist - 1/p0)
                else:
                    f = 0
                f_repulse += f
            f_attract = distance.euclidean((goal_point.x,goal_point.y), (x,y)) * epsilon
            potentials[x,y] = f_attract + f_repulse
    # make sure the file directory works for any user
    np.savetxt("/home/"+getuser()+"/turtlepath/rl-ws/pf.csv", potentials, delimiter=",")
    print("Potentials Found")
    path = []
    # Sketch do while
    this_point = start_point
    path.append(start_point)
    next_point = greedy_path(potentials, map, this_point)
    path.append(next_point)
    while next_point != this_point and next_point != goal_point:
        print(this_point)
        this_point = next_point
        next_point = greedy_path(potentials, map, this_point)
        path.append(next_point)
    return path

def greedy_path(potentials, map, this_point):
    sucessors = get_succesors(this_point,map)
    sucessors.append(this_point)
    sucessor_pf_map = [(potentials[succ.y,succ.x]) for succ in sucessors]
    max_succ = np.argmin(sucessor_pf_map)
    next_point = sucessors.pop(max_succ)
    return next_point

def get_succesors(point, map):
    succesors = []
    if((point.x + 1) < map.shape[0]):
            succesors.append((Pose2D(point.x+1, point.y,0)))
    if((point.x - 1) > 0):
            succesors.append((Pose2D(point.x-1, point.y,0)))
    if((point.y + 1) < map.shape[0]):
            succesors.append((Pose2D(point.x, point.y+1,0)))
    if((point.y - 1) > 0):
            succesors.append((Pose2D(point.x, point.y-1,0)))
    return succesors

# turn the path (set of points) into a set of commands.
def path_to_cmd():
    # build up a list of commands from the path
    global cmds
    # keep track of the previous point in the path, since commands 
    #   will be based on differences in path points.
    prev_pt = None

    # intermediate step: turn list of points into list of moves
    moves = []
    for p in path:
        #print("entry", p.x, p.y)
        if prev_pt is None:
            # p is the first point
            prev_pt = p
        else:
            x_diff = p.x - prev_pt.x
            y_diff = p.y - prev_pt.y
            moves.append((x_diff, y_diff))
            prev_pt = p

    # turn moves into commands.
    # robot starts facing up on map (-y). 
    # These directions refer to the map printed to console, not the display.
    # The actual direction is not important, as long as the coord system matches control_node.
    for m in moves:
        # figure out the cardinal direction of the move
        if m[0] > 0.001: # moving right on map, in +x
            cmds.append("east")
        elif m[0] < -0.001: # moving left on map, in -x
            cmds.append("west")
        elif m[1] > 0.001: # moving down on map, in +y
            cmds.append("south")
        elif m[1] < -0.001: # moving up on map, in -y
            cmds.append("north")

    print("-------------------path_to_cmd finished-------------------------")
    print(cmds)

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

def send_next_cmd(req_status):
    # when the next command is requested, send it and remove it from the list.
    global cmds
    if cur_state == "executing" and len(cmds) > 0: 
        # only try to send a command if they are ready.
        cmd = cmds.pop(0)
        print("Sending command: ", cmd)
        send_command(cmd)

def send_command(keyword):
    cmd_msg.data = keyword
    command_pub.publish(cmd_msg)

def update_state(timer_event):
    # at each timer step, update the state and send an action
    check_state()

def main():
    global command_pub

    # initialize node
    rospy.init_node('pf_agent')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/tp/cmd", String, queue_size=1)

    # subscribe to the map
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