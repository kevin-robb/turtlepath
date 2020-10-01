#!/usr/bin/env python

import rospy
from random import randint
#import time
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

## Global Variables
# mobile_base velocity publisher
command_pub = None
# Twist command that will be sent to the robot
rand_cmd = Twist()
# boolean to perma stop the robot
halt = False
# selected entries in the lidar's range
fwd_scan = None
rear_scan = None
l45_scan = None
l_scan = None
r45_scan = None
r_scan = None
# current time
#cur_time = 0
# current state. can be either "init", "drive, "halt", "turn_r", "turn_l"
cur_state = "init"
# current forward and turn speeds
fwd_spd = None
turn_spd = None

def check_scan(scan_msg):
    global fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan
    # scan_msg.ranges is an array of 640 elements representing 
    # distance measurements in a full circle around the robot (0=fwd, CCW?)

    # update the important entries
    fwd_scan = scan_discrete(scan_msg.ranges[0]) # directly forward
    rear_scan = scan_discrete(scan_msg.ranges[320]) # directly behind
    l45_scan = scan_discrete(scan_msg.ranges[80]) # ~45 degrees left
    l_scan = scan_discrete(scan_msg.ranges[160]) # 90 degrees left
    r45_scan = scan_discrete(scan_msg.ranges[560]) # ~45 degrees right
    r_scan = scan_discrete(scan_msg.ranges[480]) # 90 degrees right

def scan_discrete(scan_val):
    # turns continuous scan values into either 1, 2, or 3 to 
    # represent dist to obstacle in a discrete format (shrink state space)
    if scan_val == 0:
        # a lidar reading of 0 means it doesn't see anything
        return 3
    elif scan_val < 2:
        # close obstacle
        return 1
    elif scan_val < 4:
        # visible obstacle
        return 2
    else:
        # far away obstacle
        return 0

def check_state():
    global fwd_spd, turn_spd, cur_state
    if cur_state == "init":
        # start by going into the halt state
        cur_state = "drive"
        fwd_spd = 0.4
        turn_spd = 0
    elif cur_state == "drive" and fwd_scan == 1:
        # if an obstacle is in the "close" range, halt
        cur_state = "halt"
        fwd_spd = 0
        turn_spd = 0
    elif cur_state == "halt":
        # choose a random direction to turn
        if randint(0, 1) == 0:
            cur_state = "turn_r"
            fwd_spd = 0
            turn_spd = -1
        else:
            cur_state = "turn_l"
            fwd_spd = 0
            turn_spd = 1
    elif cur_state == "turn_r" or cur_state == "turn_l":
        # turn in place until there is free space ahead
        # or TODO turn for a set amount of time that causes a 90 degree turn
        if fwd_scan > 1:
            cur_state = "drive"
            fwd_spd = 1
            turn_spd = 0

def send_command(timer_event):
    global fwd_spd, turn_spd

    check_state()

    # tell us the current state for debug
    print(cur_state)
    print([fwd_scan, l_scan, rear_scan, r_scan])

    # don't try to do things before fwd_spd and turn_spd have been set
    if cur_state == "init":
        return

    # x-direction is forward
    lin_vel = Vector3(fwd_spd, 0, 0)
    # turn around z-axis to stay within xy-plane
    ang_vel = Vector3(0, 0, turn_spd)

    rand_cmd.linear = lin_vel
    rand_cmd.angular = ang_vel

    command_pub.publish(rand_cmd)


def main():
    global command_pub

    # initialize node
    rospy.init_node('rand_kevin')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # subscribe to the lidar scan values
    rospy.Subscriber('/scan', LaserScan, check_scan, queue_size=1)

    # Set up a timer to update robot's drive state at 4 Hz
    rospy.Timer(rospy.Duration(secs=0.25), send_command)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass