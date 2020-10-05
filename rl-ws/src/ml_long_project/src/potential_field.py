#!/usr/bin/env python

import rospy
from random import randint
#import time
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32MultiArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
from scipy.spatial import distance

## Global Variables
# mobile_base velocity publisher
command_pub = None
# String command that will be sent to the robot
cmd_msg = String()

# current time
#cur_time = 0
# current state. can be either "init", "drive, "halt", "turn_r", "turn_l"
cur_state = "init"

global_map=None
start_point = Pose2D(7,7,0)
goal_point =  Pose2D(1,1,0)

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
    global global_map
    global start_point,goal_point
    if cur_state == "init":
        if(global_map.any() != None ):
            cur_state = "plan"

    elif cur_state == "plan":
        print("Planning!")
        cur_state = "plan"
        potential_fields(global_map,start_point,goal_point)

# http://www.diag.uniroma1.it/~oriolo/amr/slides/MotionPlanning3_Slides.pdf
def potential_fields(global_map,start_point,goal_point):
    epsilon = 1
    sigma = .2
    l = 3
    s = 4
    #p0 is distance of influence and is infinite

    obs = []
    potentials = np.zeros(global_map.shape)
    for x in range(global_map.shape[0]):
        for y in range(global_map.shape[1]):
            if(global_map[x,y] == 1):
                obs.append(Pose2D(y,x,0))
    
    print([(o.x,o.y, global_map[x,y])for o in obs])

    for x in range(global_map.shape[0]):
        for y in range(global_map.shape[1]):
            # max_repulse = 0
            gi = [calc_g(o, Pose2D(x,y,0),l) for o in obs]
            # ind = np.argmax(obs_dist)
            # p_q = obs_dist[ind]
            # repulse = n*(1/p_q - 0)*(1/(p_q*p_q)) #p0 would replace 0 term as 1/inf = 0
            p_g = distance.euclidean((goal_point.x,goal_point.y), (x,y))
            p_ha = .3 # estimated atm

            potentials[x,y] = p_g + max(gi)+p_ha
    
    np.savetxt("/home/lelliott/turtlepath/rl-ws/pf.csv", potentials, delimiter=",")

    #print(potentials)

def calc_g(obstacle_point, current_point,l):
    x0 = obstacle_point.x
    y0 = obstacle_point.y
    x = current_point.x 
    y = current_point.y
    return (x0-l/2-x) + abs(x0-l/2-x) + (x-x0-l/2+1) + abs(x-x0-l/2+1) + (y0-l/2-y) + abs(y0-l/2-y) + (y-y0-l/2+1)+ abs(y-y0-l/2+1)

def calc_p_ha(convex_region, sigma,l):
    ret = []
    for r in convex_region:
        ret.append(1/(sigma + sum([calc_g(g,r,l) for g in convex_region])))
    return ret

def get_map(map_msg):
    global global_map
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
                big_y=0
            # Add a 1 in small map if not 0
            small_map[y,x] = int( big_map[big_y,big_x] != 0)
        
    # Small map is a downsampled map at the 1m x 1m "vertexs" of big map
    small_map = np.fliplr(np.flipud(small_map.transpose()))
    print(small_map)
    global_map = small_map
    np.savetxt("/home/lelliott/turtlepath/rl-ws/map.csv", global_map, delimiter=",")


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
    global command_pub

    # initialize node
    rospy.init_node('pf_agent')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/tp/cmd", String, queue_size=1)

    # subscribe to the grouped scan values
    # format is [fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan]
    rospy.Subscriber('/tp/scan', Int32MultiArray, get_scan_ranges, queue_size=1)
    rospy.Subscriber('/map', OccupancyGrid, get_map, queue_size=1)

    # Set up a timer to update robot's drive state at 1 Hz
    rospy.Timer(rospy.Duration(secs=.25), update_state)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass