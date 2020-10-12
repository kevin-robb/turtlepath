#!/usr/bin/env python

## The True Random agent receives no input or feedback from the environment.
#   It takes a random action (moving either forward, backward, left, or right) at each timestep.
#   It is prevented from hitting walls by the control_node.

import rospy
from random import randint
from std_msgs.msg import String, Bool

## Global Variables
# command publisher to control_node
command_pub = None
# String command that will be sent to the robot
cmd_msg = String()

def random_cmd(req_status):
    # randomly choose what action to take.
    selection = randint(0, 3)
    if selection == 0:
        cmd_msg.data = "north"
    elif selection == 1:
        cmd_msg.data = "south"
    elif selection == 2:
        cmd_msg.data = "east"
    else: # selection == 3
        cmd_msg.data = "west"
    # send the high-level command to control_node to be executed.
    command_pub.publish(cmd_msg)

def main():
    global command_pub

    # initialize node
    rospy.init_node('true_rand')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/tp/cmd", String, queue_size=1)

    # subscribe to the control_node requesting commands on the custom topic '/tp/request'
    rospy.Subscriber('/tp/request', Bool, random_cmd, queue_size=1)

    # Set up a timer to update robot's drive state at 1 Hz
    #rospy.Timer(rospy.Duration(secs=2), random_cmd)

    # pump callbacks
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass