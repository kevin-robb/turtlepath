#!/usr/bin/env python

import rospy
from random import randint
#from time import time
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
# useful entries in the lidar's range for checking validity of commands
fwd_scan = None
rear_scan = None
l_scan = None
r_scan = None
# Twist command that will be sent to the robot
cmd = Twist()
# boolean to perma stop the robot
#halt = False
# current time
#cur_time = 0

# boolean for checking whether robot is still in initialization
initvar = True

# Where we are and where we want to go
current_position = Pose2D(0,0,0)
goal_position = Pose2D(0,0,0)

integral_prior = 0
error_prior = 0
iteration_time = .1
command_list = []

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
    if(current_position.theta < 0):
        current_position.theta += 360
    # 2D plane, we will stay at 0 on x,y

def scan_discrete(scan_val):
    # Turns continuous scan values into either 1, 2, or 3 to 
    #   represent distance to obstacles in a discrete format.
    # The goal of this is to shrink the state space and make learning easier.

    if scan_val == 0:
        # a lidar reading of 0 means it doesn't see anything.
        # group with "far away"
        return 3
    ## TODO Set threshold to correspond to the adjacent vertex being occupied
    elif scan_val < 2:
        # close obstacle
        return 1
    ## TODO Set threshold to correspond to the adjacent vertex being free but the next one occupied
    elif scan_val < 4:
        # visible obstacle
        return 2
    else:
        # far away obstacle
        return 3

def add_cmd(str_cmd):
    global command_list
    # commands will be either "forward", "left", "back", or "right".
    # we need to handle turning here by creating two commands for each one.
    if is_cmd_valid(str_cmd):
        if str_cmd == "forward" or str_cmd == "back":
            command_list.append(str_cmd)
        elif str_cmd == "right":
            command_list.append("turn_right")
            command_list.append("forward")
        elif str_cmd == "left":
            command_list.append("turn_left")
            command_list.append("forward")
        print(str_cmd)
    else:
        print("Invalid Command:" + str_cmd)

def is_cmd_valid(str_cmd):
    # check to ensure a given command will not cause 
    #   the robot to move to an occupied vertex.
    if str_cmd == "forward":
        return fwd_scan > 1
    elif str_cmd == "left":
        return l_scan > 1
    elif str_cmd == "back":
        return rear_scan > 1
    elif str_cmd == "right":
        return r_scan > 1
    else:
        # not a valid command
        return False

def set_cmd(str_cmd):
    # interpret the command (string) and execute the given command.
    if str_cmd.data == "turn_right":
        set_goal(0,-90)
    elif str_cmd.data == "turn_left":
        set_goal(0,90)
    elif str_cmd.data == "forward":
        set_goal(1,0)
    elif str_cmd.data == "backward":
        set_goal(-1,0)
    # elif str_cmd.data == "halt":
    #     set_goal(0,0)
    
def set_goal(forward, theta):
    global goal_position, current_position
    ## Check current heading relative to start ("forward"),
    # and apply the command to set a new goal position.

    # facing forward
    if(goal_position.theta == 0.0):
        goal_position.x += forward
        goal_position.theta += theta
    # facing left
    elif(goal_position.theta == 90.0):
        goal_position.y += forward
        goal_position.theta += theta
    # facing backward
    elif(abs(goal_position.theta) == 180.0):
        goal_position.x -= forward
        goal_position.theta += theta
    # facing right
    elif(goal_position.theta == 270.0):
        goal_position.y -= forward
        goal_position.theta += theta

    # normalize angle to be in range 0 to 360
    goal_position.theta = goal_position.theta % 360

def execute_goal(event):
    global initvar, current_position, goal_position, integral_prior, error_prior, iteration_time, command_list
    # check how far off the robot is from the goal position and heading
    delta_x = goal_position.x - current_position.x
    delta_y = goal_position.y - current_position.y
    delta_theta = goal_position.theta - current_position.theta

    # initialize with the current position as the goal.
    if(initvar):
        initvar = False
        goal_position = deepcopy(current_position)

    # if the robot is within 0.1 units of the goal in both x and y 
    #   and within 5 degrees, get the next command.
    elif(abs(delta_x) < .1 and abs(delta_y) < .1 and abs(delta_theta) < 5):
        # check if there are any commands in the queue.
        if(len(command_list) > 0):
            # pop the next command off the queue and set it to run next.
            next_cmd = command_list.pop(0)
            set_cmd(next_cmd)

    # the goal is in front or behind the robot.
    if(abs(goal_position.theta) == 180 or goal_position.theta == 0):
        forward = goal_position.x - current_position.x
        if (abs(goal_position.theta) == 180):
            forward *= -1
    # the goal is to the left or right of the robot.
    else:
        forward = goal_position.y - current_position.y
        if (goal_position.theta == 270):
            forward *= -1

    # shift angles to be in the range -180 to 180
    if(delta_theta < -180):
        delta_theta += 360
    elif(delta_theta > 180):
        delta_theta -= 360

    # send the command to the robot.
    send_cmd(forward, delta_theta*.1)

def send_cmd(fwd_spd, turn_spd):
    # x-direction is forward.
    lin_vel = Vector3(fwd_spd, 0, 0)
    # turn around z-axis to stay within xy-plane.
    ang_vel = Vector3(0, 0, turn_spd)

    cmd.linear = lin_vel
    cmd.angular = ang_vel
    command_pub.publish(cmd)

def main():
    global command_pub, scan_pub, iteration_time

    # initialize node.
    rospy.init_node('control_node')

    # publish command to the turtlebot.
    command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # publish scan data to custom topic '/tp/scan' for agents to use as input.
    # format is [fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan].
    scan_pub = rospy.Publisher("/tp/scan", Int32MultiArray, queue_size=1)

    # subscribe to the lidar scan values.
    rospy.Subscriber('/scan', LaserScan, check_scan, queue_size=1)
    # subscribe to custom topic /tp/cmd which is used for discrete commands.
    rospy.Subscriber('/tp/cmd', String, add_cmd, queue_size=1)
    # subscrive to odometry info.
    rospy.Subscriber('/odom', Odometry, check_odom, queue_size=1)

    # execute commands at 1/iteration_time Hz.
    rospy.Timer(rospy.Duration(iteration_time), execute_goal)

    # pump callbacks.
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass