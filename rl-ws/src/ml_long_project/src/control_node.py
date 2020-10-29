#!/usr/bin/env python

import rospy
from random import randint
#from time import time
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32MultiArray, Bool
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
import math
from std_srvs.srv import Empty

## Global Variables
# mobile_base velocity publisher
command_pub = None
# scan data publisher
scan_pub = None
# publisher to request commands from the agent
req_pub = None
# useful entries in the lidar's range for checking validity of commands
fwd_scan = None
rear_scan = None
l_scan = None
r_scan = None
off_left = None
off_right = None
# Twist command that will be sent to the robot
cmd = Twist()

# boolean for checking whether robot is still in initialization
initvar = True
stall_count = 0
stalled = False
# Where we are and where we want to go
current_position = Pose2D(0,0,0)
goal_position = Pose2D(0,0,0)
start_position = Pose2D(0,0,0)

integral_prior_angle = 0
integral_prior_forward = 0
error_prior_forward = 0
error_prior_angle = 0

# keep track of the command currently being executed
current_cmd = "halt"

# 1/frequency of timer events
iteration_time = .1
# list of commands (strings, not String msgs)
command_list = []

def check_scan(scan_msg):
    global fwd_scan, rear_scan, r_scan, l_scan,off_right,off_left
    # scan_msg.ranges is an array of 640 elements representing 
    # distance measurements in a full circle around the robot (0=fwd, CCW?)

    # update and discretize the important entries
    fwd_scan = scan_discrete(scan_msg.ranges[0]) # directly forward
    rear_scan = scan_discrete(scan_msg.ranges[320]) # directly behind
    l45_scan = scan_discrete(scan_msg.ranges[80]) # ~45 degrees left
    l_scan = scan_discrete(scan_msg.ranges[160]) # 90 degrees left
    r45_scan = scan_discrete(scan_msg.ranges[560]) # ~45 degrees right
    r_scan = scan_discrete(scan_msg.ranges[480]) # 90 degrees right

    off_left = scan_discrete(scan_msg.ranges[40])
    off_right = scan_discrete(scan_msg.ranges[600])

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
    elif scan_val < 1.4:
        # close obstacle (adjacent vertex is occupied)
        return 1
    elif scan_val < 3:
        # visible obstacle, but not at the adjacent vertex
        return 2
    else:
        # far away obstacle
        return 3

def add_cmd(str_msg):
    global command_list

    # commands will be either "north", "south", "east", or "west".
    # we need to handle turning here by creating two commands for each one.
    
    # first determine the amount we need to turn to get 
    #   from current heading to the desired heading.
    # goal_position.theta keeps track of the heading we 
    #   currently have or are actively trying to get onto.
    if str_msg.data == "north" and goal_position.theta == 0 \
        or str_msg.data == "east" and goal_position.theta == 270 \
        or str_msg.data == "south" and goal_position.theta == 180 \
        or str_msg.data == "west" and goal_position.theta == 90:
        # we are already facing the right way.
        pass
    elif str_msg.data == "north" and goal_position.theta == 180 \
        or str_msg.data == "east" and goal_position.theta == 90 \
        or str_msg.data == "south" and goal_position.theta == 0 \
        or str_msg.data == "west" and goal_position.theta == 270:
        # we are facing the opposite way.
        command_list.append("turn_180")
    elif str_msg.data == "north" and goal_position.theta == 90 \
        or str_msg.data == "west" and goal_position.theta == 180 \
        or str_msg.data == "south" and goal_position.theta == 270 \
        or str_msg.data == "east" and goal_position.theta == 0:
        # we need to turn right
        command_list.append("turn_right")
    elif str_msg.data == "north" and goal_position.theta == 270 \
        or str_msg.data == "east" and goal_position.theta == 180 \
        or str_msg.data == "south" and goal_position.theta == 90 \
        or str_msg.data == "west" and goal_position.theta == 0:
        # we need to turn left
        command_list.append("turn_left")
    elif str_msg.data == "reset":
        print("Reset!!!!")
        reset_stage()
        return
    else:
        print("Invalid Command:" + str_msg.data)
        return
    
    # the robot has now been told to turn the correct way. Next, move.
    command_list.append("forward")

def reset_stage():
    global goal_position, current_position, current_cmd, start_position
    global command_list
    print("Resetting Stage")
    reset_positions = rospy.ServiceProxy('reset_positions', Empty)
    command_list = []
    reset_positions()
    goal_position =  deepcopy(start_position)
    current_cmd = ""

def is_cmd_valid(str_cmd):
    global fwd_scan, rear_scan, r_scan, l_scan,off_right,off_left
    # check to ensure a given command (string) will not cause 
    #   the robot to move to an occupied vertex.
    if str_cmd == "forward":
        print("Trying to Move Forward", fwd_scan)
        return fwd_scan > 1.75 and off_right > 1 and off_left > 1
    elif str_cmd == "turn_left" or str_cmd == "turn_right" or str_cmd == "turn_180":
        print("Turning in place")
        return True
    else:
        # not a valid command
        return False

def set_cmd(str_cmd):
    # interpret the command (string) and execute the given command.
    if str_cmd == "forward":
        set_goal(1,0)
    elif str_cmd == "turn_right":
        set_goal(0,-90)
    elif str_cmd == "turn_left":
        set_goal(0,90)
    elif str_cmd == "turn_180":
        set_goal(0,180)
    
def set_goal(forward, theta):
    global goal_position, current_position
    ## Check current heading relative to start ("forward"),
    # and apply the command to set a new goal position.

    # facing forward (north)
    if(goal_position.theta == 0.0):
        goal_position.x += forward
        goal_position.theta += theta
    # facing left (west)
    elif(goal_position.theta == 90.0):
        goal_position.y += forward
        goal_position.theta += theta
    # facing backward (south)
    elif(abs(goal_position.theta) == 180.0):
        goal_position.x -= forward
        goal_position.theta += theta
    # facing right (east)
    elif(goal_position.theta == 270.0):
        goal_position.y -= forward
        goal_position.theta += theta

    # normalize angle to be in range 0 to 360
    goal_position.theta = goal_position.theta % 360

    if(goal_position.theta < 0):
        goal_position.theta += 360

def execute_goal(event):
    global initvar, current_position, goal_position, integral_prior_forward,integral_prior_angle, error_prior_forward, error_prior_angle, iteration_time, command_list,current_cmd
    global start_position
    # check how far off the robot is from the goal position and heading
    delta_theta = goal_position.theta - current_position.theta

    # initialize with the current position as the goal.
    if(initvar):
        initvar = False
        goal_position = deepcopy(current_position)
        start_position = deepcopy(current_position)
    # if the robot is within 0.1 units of the goal in both x and y 
    #   and within 5 degrees, get the next command.

    # robot is facing (or is turning to face) south or north
    if(abs(goal_position.theta) == 180 or goal_position.theta == 0):
        forward = goal_position.x - current_position.x
        if (abs(goal_position.theta) == 180):
            forward *= -1
    # robot is facing (or is turning to face) east or west
    else:
        forward = goal_position.y - current_position.y
        if (goal_position.theta == 270): # facing east
            forward *= -1\

    # shift angles to be in the range -180 to 180
    if(delta_theta < -180):
        delta_theta += 360
    elif(delta_theta > 180):
        delta_theta -= 360

    integral_prior_forward = integral_prior_forward + forward*iteration_time
    integral_prior_angle = integral_prior_angle + delta_theta*iteration_time

    fw_pid = forward*7+ integral_prior_forward*.15 + (forward - error_prior_forward)/iteration_time * .04
    an_pid = delta_theta*.1+ integral_prior_angle*0+ (delta_theta - error_prior_angle)/iteration_time * 0

    if(current_cmd == "forward"):
        send_cmd(fw_pid,0)
    else:
        send_cmd(0,an_pid)

    error_prior_forward = forward
    error_prior_angle = delta_theta

    finish_msg = Bool()
    finish_msg.data = False

    # if we are moving forward, check that we have basically arrived at the desired location.
    # if we are turning, check that we have basically gotten onto the desired heading.
    if((current_cmd == "forward" and abs(fw_pid) < .1) or (current_cmd != "forward" and abs(an_pid) < .1)):
            # check if we should request another command
            if len(command_list) < 2:
                request_cmd()
              #  if current_cmd == "forward":
              #      print("switching")
              #      finish_msg.data = True
            # check if there are any commands in the queue.
            if(len(command_list) > 0 ):
                # pop the next command off the queue and if it's valid, set it to run next.
                #print(command_list)
                next_cmd = command_list.pop(0)
                print(current_cmd)
                current_cmd = next_cmd

                if is_cmd_valid(next_cmd):
                    set_cmd(next_cmd)
                    integral_prior_forward = 0
                    integral_prior_angle = 0
                    error_prior_forward = 0
                    error_prior_angle = 0
    finish_pub.publish(finish_msg)

def send_cmd(fwd_spd, turn_spd):
    # x-direction is forward.
    lin_vel = Vector3(fwd_spd, 0, 0)
    # turn around z-axis to stay within xy-plane.
    ang_vel = Vector3(0, 0, turn_spd)

    cmd.linear = lin_vel
    cmd.angular = ang_vel
    command_pub.publish(cmd)

def request_cmd():
    # request a command from the agent
    req_msg = Bool()
    req_msg.data = True
    req_pub.publish(req_msg)

def main():
    global command_pub, scan_pub, req_pub, iteration_time, finish_pub

    # initialize node.
    rospy.init_node('control_node')

    # publish command to the turtlebot.
    command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # publish scan data to custom topic '/tp/scan' for agents to use as input.
    # format is [fwd_scan, rear_scan, l45_scan, l_scan, r45_scan, r_scan].
    scan_pub = rospy.Publisher("/tp/scan", Int32MultiArray, queue_size=1)
    # publish to a custom topic '/tp/request' to let the agent know we are ready for another command.
    req_pub = rospy.Publisher('/tp/request', Bool, queue_size=1)
    finish_pub = rospy.Publisher('/tp/finish', Bool, queue_size=1)

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