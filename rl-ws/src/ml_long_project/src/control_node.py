#!/usr/bin/env python

import rospy
from random import randint
from time import time
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32MultiArray
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
import math

## Global Variables
# mobile_base velocity publisher
command_pub = None
# scan data publisher
scan_pub = None
# Twist command that will be sent to the robot
cmd = Twist()
# boolean to perma stop the robot
#halt = False
# current time
cur_time = 0

initvar = True

# Where we are and where we want to go
current_position = Pose2D(0,0,0)
goal_position = Pose2D(0,0,0)

def check_scan(scan_msg):
    # scan_msg.ranges is an array of 640 elements representing 
    # distance measurements in a full circle around the robot (0=fwd, CCW?)

    # update and discretize the important entries
    fwd_scan = scan_discrete(scan_msg.ranges[0]) # directly forward
    rear_scan = scan_discrete(scan_msg.ranges[320]) # directly behind
    l45_scan = scan_discrete(scan_msg.ranges[80]) # ~45 degrees left
    l_scan = scan_discrete(scan_msg.ranges[160]) # 90 degrees left
    r45_scan = scan_discrete(scan_msg.ranges[560]) # ~45 degrees right
    r_scan = scan_discrete(scan_msg.ranges[480]) # 90 degrees right

    # group and publish the relevant scan ranges
    scan_group = Int32MultiArray()
    scan_group.data = [fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan]
    scan_pub.publish(scan_group)

def check_odom(odom_msg):
    global current_position
    current_position.x = odom_msg.pose.pose.position.x
    current_position.y = odom_msg.pose.pose.position.y
    quat_orien = odom_msg.pose.pose.orientation
    r = R.from_quat([quat_orien.x, quat_orien.y, quat_orien.z, quat_orien.w])
    #print(str(r.as_rotvec()[2] * 180/3.1))
    # Theta is positive ccw of start until 180, Theta is negative cw of start until -180
    current_position.theta = r.as_rotvec()[2]* 180/3.1
    if(current_position.theta > 180):
        current_position.theta -= 360

    # 2D plane, we will stay at 0 on x,y

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
        return 3

def get_cmd(str_cmd):
    # interpret the command (string) and execute the given command
    if str_cmd.data == "turn_r_90":
        set_goal(0,-90)
    if str_cmd.data == "turn_r_45":
        set_goal(0,-45)
    elif str_cmd.data == "turn_l_90":
        set_goal(0,90)
    elif str_cmd.data == "turn_l_45":
        set_goal(0,45)
    elif str_cmd.data == "forward":
        set_goal(1,0)
    elif str_cmd.data == "backward":
        set_goal(-1,0)
    elif str_cmd.data == "halt":
        set_goal(0,0)
    
def set_goal(forward, theta):
    global goal_position
    global current_position


    ## Facing Left
    if(goal_position.theta == 90.0):
        goal_position.y -= forward
        goal_position.theta += theta

    ## Facing Forward
    elif(goal_position.theta == 0.0):
        print("Here!")
        goal_position.x += forward
        goal_position.theta += theta

    ## Facing Right
    elif(abs(goal_position.theta) == 180.0):
        goal_position.x += forward
        goal_position.theta += theta

    ## Facing Backwards
    elif(goal_position.theta == -90.0):
        goal_position.y -= forward
        goal_position.theta += theta

    if(goal_position.theta == -270.0):
        goal_position.theta = 90
    elif(goal_position.theta == 270):
        goal_position.theta = -90

def execute_goal(event):
    global initvar
    global current_position
    global goal_position

    if(initvar):
        initvar = False
        goal_position = deepcopy(current_position)

 
    print("Where are we")
    print(current_position.y)
    print(goal_position.y)

    #print("How are we going to get there?")

    x_dist = goal_position.x - current_position.x
    y_dist = goal_position.y - current_position.y

    forward = max(x_dist,y_dist)

    angle = goal_position.theta - current_position.theta
    if(abs(angle)>180):
        angle = angle*-1

    lin_vel = Vector3(forward, 0, 0)
    # turn around z-axis to stay within xy-plane
    ang_vel = Vector3(0, 0, angle)
    print(lin_vel.x)

    cmd.linear = lin_vel
    cmd.angular = ang_vel
    command_pub.publish(cmd)

def turn_r_90():
    # 90 degree right turn
    send_cmd(0, 1)
    stall(1.5)

def turn_r_45():
    # 45 degree right turn
    send_cmd(0, 1)
    stall(0.75)

def turn_l_90():
    # 90 degree left turn
    send_cmd(0, 1)
    stall(1.5)

def turn_l_45():
    # 45 degree left turn
    send_cmd(0, 1)
    stall(0.75)

def forward():
    # move straight forwards
    send_cmd(1, 0)
    stall(1)

def backward():
    # move straight backwards
    send_cmd(-1, 0)
    stall(1)

def halt():
    # stop moving
    send_cmd(0, 0)
    # no stalling necessary

def stall(wait_time):
    cur_time = time()
    while time() < cur_time + wait_time:
        pass

def send_cmd(fwd_spd, turn_spd):
    # x-direction is forward
    lin_vel = Vector3(fwd_spd, 0, 0)
    # turn around z-axis to stay within xy-plane
    ang_vel = Vector3(0, 0, turn_spd)

    cmd.linear = lin_vel
    cmd.angular = ang_vel
    command_pub.publish(cmd)

def main():
    global command_pub, scan_pub

    # initialize node
    rospy.init_node('control_node')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # publish scan data for agents to use as input
    # format is [fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan]
    scan_pub = rospy.Publisher("/tp/scan", Int32MultiArray, queue_size=1)

    # subscribe to the lidar scan values
    rospy.Subscriber('/scan', LaserScan, check_scan, queue_size=1)
    # subscribe to custom topic /tp/cmd which is used for discrete commands
    rospy.Subscriber('/tp/cmd', String, get_cmd, queue_size=1)

    rospy.Subscriber('/odom', Odometry, check_odom, queue_size=1)
    rospy.Timer(rospy.Duration(.1), execute_goal)

    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass