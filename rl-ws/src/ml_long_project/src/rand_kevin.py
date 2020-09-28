#!/usr/bin/env python

import rospy
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
l15_scan = None
l_scan = None
r15_scan = None
r_scan = None
# current time
cur_time = 0
# current state. can be either "init", "drive, "halt", "turn_r", "turn_l"
cur_state = "init"
# current forward and turn speeds
fwd_spd = None
turn_spd = None

def check_scan(scan_msg):
    global fwd_scan, rear_scan, l15_scan, l_scan, r15_scan, r_scan
    # scan_msg.ranges is an array of 640 elements representing 
    # distance measurements in a full circle around the robot (0=fwd, CCW?)

    # update the important entries (TODO verify these indexes)
    fwd_scan = scan_msg.ranges[0] # directly forward
    rear_scan = scan_msg.ranges[320] # directly behind
    l15_scan = scan_msg.ranges[30] # ~15 degrees left
    l_scan = scan_msg.ranges[160] # 90 degrees left
    r15_scan = scan_msg.ranges[610] # ~15 degrees right
    r_scan = scan_msg.ranges[480] # 90 degrees right

def check_state():
    global fwd_spd, turn_spd, cur_state
    if cur_state == "init":
        # start by just going straight forwards
        cur_state = "drive"
        fwd_spd = 1
        turn_spd = 0
    elif cur_state == "drive" and fwd_scan > 0 and fwd_scan < 1:
        cur_state = "halt"
        fwd_spd = 0
        turn_spd = 0
    elif cur_state == "halt":
        # check surroundings and decide where to go
        if r_scan < 0.001 or r_scan > 2:
            cur_state = "turn_r"
            fwd_spd = 0.1
            turn_spd = 1
        elif l_scan < 0.001 or l_scan > 2:
            cur_state = "turn_l"
            fwd_spd = 0.1
            turn_spd = -1
        #else #do nothing for now, stay in halt
    elif cur_state == "turn_r" or cur_state == "turn_l":
        # turn in place until there is free space ahead
        # or TODO turn for a set amount of time that causes a 90 degree turn
        if fwd_scan < 0.001 or fwd_scan > 2:
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

    if halt:
        fwd_spd = 0
        turn_spd = 0
    # else:
    #     # set random forward and turn speeds (TODO turn away from walls)
    #     fwd_spd = 1
    #     turn_spd = 0

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

    # Set up a timer to update robot's drive state at 20 Hz
    rospy.Timer(rospy.Duration(secs=0.05), send_command)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

## Results of rostopic list
# /amcl/parameter_descriptions
# /amcl/parameter_updates
# /amcl_pose
# /base_pose_ground_truth
# /clock
# /cmd_vel
# /diagnostics
# /initialpose
# /joint_states
# /map
# /map_metadata
# /map_updates
# /mobile_base/commands/velocity
# /move_base/DWAPlannerROS/cost_cloud
# /move_base/DWAPlannerROS/global_plan
# /move_base/DWAPlannerROS/local_plan
# /move_base/DWAPlannerROS/parameter_descriptions
# /move_base/DWAPlannerROS/parameter_updates
# /move_base/DWAPlannerROS/trajectory_cloud
# /move_base/NavfnROS/plan
# /move_base/cancel
# /move_base/current_goal
# /move_base/feedback
# /move_base/global_costmap/costmap
# /move_base/global_costmap/costmap_updates
# /move_base/global_costmap/footprint
# /move_base/global_costmap/inflation_layer/parameter_descriptions
# /move_base/global_costmap/inflation_layer/parameter_updates
# /move_base/global_costmap/obstacle_layer/parameter_descriptions
# /move_base/global_costmap/obstacle_layer/parameter_updates
# /move_base/global_costmap/parameter_descriptions
# /move_base/global_costmap/parameter_updates
# /move_base/global_costmap/static_layer/parameter_descriptions
# /move_base/global_costmap/static_layer/parameter_updates
# /move_base/goal
# /move_base/local_costmap/costmap
# /move_base/local_costmap/costmap_updates
# /move_base/local_costmap/footprint
# /move_base/local_costmap/inflation_layer/parameter_descriptions
# /move_base/local_costmap/inflation_layer/parameter_updates
# /move_base/local_costmap/obstacle_layer/parameter_descriptions
# /move_base/local_costmap/obstacle_layer/parameter_updates
# /move_base/local_costmap/parameter_descriptions
# /move_base/local_costmap/parameter_updates
# /move_base/parameter_descriptions
# /move_base/parameter_updates
# /move_base/result
# /move_base/status
# /move_base_simple/goal
# /odom
# /particlecloud
# /rosout
# /rosout_agg
# /scan
# /tf
# /tf_static
