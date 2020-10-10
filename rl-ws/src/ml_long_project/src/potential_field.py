#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Quaternion, Pose2D
from std_msgs.msg import String
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

# current state. can be either "init", "plan", "execute"
cur_state = "init"

global_map = None
start_point = Pose2D(7,7,0)
goal_point =  Pose2D(1,1,0)

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

# http://kovan.ceng.metu.edu.tr/~asil/old/_1./wh2.html
def potential_fields(global_map,start_point,goal_point):
    epsilon = 1
    n = 5
    p0 = 3

    obs = []
    potentials = np.zeros(global_map.shape)
    for x in range(global_map.shape[0]):
        for y in range(global_map.shape[1]):
            if(x != 0 and x != global_map.shape[0]-1 and y != 0 and y != global_map.shape[1]-1):
                if(global_map[x,y] == 1):
                    obs.append(Pose2D(x,y,0))

    for x in range(global_map.shape[0]):
        for y in range(global_map.shape[1]):
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
    np.savetxt("/home/"+getuser()+"/turtlepath/rl-ws/map.csv", global_map, delimiter=",")


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

    # Set up a timer to update robot's drive state at 1 Hz
    rospy.Timer(rospy.Duration(secs=.25), update_state)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass