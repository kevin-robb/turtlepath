#!/usr/bin/env python

## The Random Heuristic agent has access to the scan data provided by control_node.
#   It will choose the cardinal direction with the most open space, and break ties
#   randomly.

import rospy
from random import randint
#import time
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32MultiArray, Bool

## Global Variables
# command publisher to control_node
command_pub = None
# String command that will be sent to the robot
cmd_msg = String()
# selected entries in the lidar's range
fwd_scan = None
rear_scan = None
l45_scan = None
l_scan = None
r45_scan = None
r_scan = None
# current state. can be either "init", "drive, "halt", "turn_r", "turn_l"
cur_state = "init"
# current forward and turn speeds
fwd_spd = None
turn_spd = None

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

def send_command(keyword):
    # Send a high-level command (string) to control_node to be executed.
    cmd_msg.data = keyword
    command_pub.publish(cmd_msg)

def update_state(req_status):
    # determine move direction(s) with most space before obstacle
    options = []
    for check_dist in range(3, 0, -1):
        if fwd_scan >= check_dist:
            options.append("forward")
        if rear_scan >= check_dist:
            options.append("back")
        if l_scan >= check_dist:
            options.append("left")
        if r_scan >= check_dist:
            options.append("right")
        if len(options) > 0:
            break
    # randomly choose one of the available options
    selection = 0
    if len(options) != 0:
        selection = randint(0,len(options)-1)
    print(options, selection)
    send_command(options[selection])

def main():
    global command_pub

    # initialize node
    rospy.init_node('rand_heuristic')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/tp/cmd", String, queue_size=1)

    # subscribe to the grouped scan values on the custom topic '/tp/scan'
    # format is [fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan]
    rospy.Subscriber('/tp/scan', Int32MultiArray, get_scan_ranges, queue_size=1)
    # subscribe to the control_node requesting commands on the custom topic '/tp/request'
    rospy.Subscriber('/tp/request', Bool, update_state, queue_size=1)

    # Set up a timer to update robot's drive state at 4 Hz
    #rospy.Timer(rospy.Duration(secs=0.25), update_state)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass