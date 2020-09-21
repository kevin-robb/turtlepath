#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

## Global Variables
# mobile_base velocity publisher
command_pub = None
# Twist command that will be sent to the robot
rand_cmd = Twist()


def send_command(timer_event):
    # set random forward and turn speeds
    fwd_speed = 1
    turn_speed = 1
    # x-direction is forward
    lin_vel = Vector3(fwd_speed, 0, 0)
    # turn around z-axis to stay within xy-plane
    ang_vel = Vector3(0, 0, turn_speed)

    rand_cmd.linear = lin_vel
    rand_cmd.angular = ang_vel

    command_pub.publish(rand_cmd)


def main():
    global command_pub

    # initialize node
    rospy.init_node('rand_kevin')

    # publish command to the turtlebot
    command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Set up a timer to update robot's drive state at 20 Hz
    rospy.Timer(rospy.Duration(secs=0.05), send_command)
    # pump callbacks
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass