#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.spatial import distance
import copy

## Global Variables
# String command publisher
command_pub = None
# String command that will be sent to the robot
cmd_msg = String()

# current state. can be either "init", "planning", "executing"
cur_state = "init"

map = []
start_point = Pose2D(7,7,0)
goal_point =  Pose2D(3,8,0) # was initially Pose2D(1,1,0)
# list of Pose2Ds representing points on the path
path = None
# list of commands generated to follow the path
cmds = []

def check_state():
    global cur_state, map, start_point, goal_point, path
    # wait until we get the map
    if cur_state == "init":
        if(len(map) > 0):
            cur_state = "planning"
    # plan a path using the map
    elif cur_state == "planning":
        print("Planning!")
        if(len(map) > 0):
            cur_state = "planning"
        path = a_star(map,start_point,goal_point)
        if(len(path) > 0):
            print(path)
            print("Executing!")
            # follow the path
            path_to_cmd()
            # wait to set the state until path_to_cmd finishes, because
            #   we will use it to check and make sure the commands are ready.
            cur_state = "executing"
    #elif cur_state == "executing":
        # don't want to do anything on loop, just let the queue of commands be run

def send_next_cmd(req_status):
    # when the next command is requested, send it and remove it from the list.
    global cmds
    if cur_state == "executing" and len(cmds) > 0: 
        # only try to send a command if they are ready.
        cmd = cmds.pop(0)
        print("Sending command: ", cmd)
        send_command(cmd)

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

#https://www.researchgate.net/figure/A-search-algorithm-Pseudocode-of-the-A-search-algorithm-operating-with-open-and-closed_fig8_232085273
def a_star(map,start_point,goal_point):
    init_f = distance.euclidean((goal_point.x,goal_point.y), (start_point.x,start_point.y))
    # Format: Point, Fval, G val, Predecessor 
    open = [(start_point, init_f , 0, [])]
    closed = []

    while len(open) > 0:
        # Open must be sorted, we need the lowest cost value
        point,f,g,predeccessor = open.pop(0)
        closed.append((point,f,g,predeccessor))
        if(len(open) == 0):
            open = []
            
        for succ in get_succesors(point, map):
            if(succ.x == goal_point.x and succ.y == goal_point.y):
                print("A* Success")
                predeccessor.append(point)
                predeccessor.append(succ)
                return predeccessor
            g = 1 + g
            h = distance.euclidean((goal_point.x,goal_point.y), (succ.x,succ.y))
            f = g+h                
            # Check for duplicates in open and closed
            dupes_closed = []
            dupes_open = []
            for point_dupes,f_dupes,g_dupes,predeccessor_dupes in closed :
                if(point_dupes.x == succ.x and point_dupes.y == succ.y):
                  dupes_closed.append((point_dupes,f_dupes,g_dupes,predeccessor_dupes))  
            for point_dupes,f_dupes,g_dupes,predeccessor_dupes in open :
                if(point_dupes.x == succ.x and point_dupes.y == succ.y):
                  dupes_open.append((point_dupes,f_dupes,g_dupes,predeccessor_dupes))  
            if(len(dupes_open) == 1 and dupes_open[0][2] > g):
                open.remove(dupes_open[0])
                dupes_open.pop(0)
            if(len(dupes_closed) == 1 and dupes_closed[0][2] > g):
                closed.remove(dupes_closed[0])
                dupes_closed.pop(0)
            if(len(dupes_closed) == 0 and len(dupes_open) == 0):
                #print(predeccessor)
                pred = copy.deepcopy(predeccessor)
                pred.append(point)
                open.append((succ,f,g,pred))
    print("A* Failure")
    return []
        
def get_succesors(point, map):
    succesors = []
    if((point.x + 1) < map.shape[0]):
        if(map[point.y,point.x+1] != 1):
            succesors.append((Pose2D(point.x+1, point.y,0)))
    if((point.x - 1) > 0):
        if(map[point.y, point.x-1] != 1):
            succesors.append((Pose2D(point.x-1, point.y,0)))
    if((point.y + 1) < map.shape[0]):
        if(map[point.y+1, point.x] != 1):
            succesors.append((Pose2D(point.x, point.y+1,0)))
    if((point.y - 1) > 0):
        if(map[point.y-1, point.x] != 1):
            succesors.append((Pose2D(point.x, point.y-1,0)))
    return succesors


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
    # np.savetxt("/home/lelliott/turtlepath/rl-ws/demofile2.csv", big_map, delimiter=",")

    for x in range(small_map.shape[0]):
        for y in range(small_map.shape[1]):
            big_x = int(x/resolution)
            big_y = int(y/resolution+1)
            # Weird Conversion error?
            if(big_y >= big_map.shape[1]):   
                big_y = big_map.shape[1]-1
            if(big_y == 1):
                big_y = 0
            # Add a 1 in small map if not 0
            small_map[y,x] = int( big_map[big_y,big_x] != 0)
        
    # Small map is a downsampled map at the 1m x 1m "vertexes" of big map
    small_map = np.fliplr(np.flipud(small_map.transpose()))
    print(small_map)
    map = small_map


def send_command(keyword):
    cmd_msg.data = keyword
    command_pub.publish(cmd_msg)

def update_state(timer_event):
    # at each timer step, update the state and send an action
    check_state()

def main():
    global command_pub

    # initialize node
    rospy.init_node('as_agent')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/tp/cmd", String, queue_size=1)

    # subscribe to the map
    rospy.Subscriber('/map', OccupancyGrid, get_map, queue_size=1)
    # subscribe to the control_node requesting commands on the custom topic '/tp/request'
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