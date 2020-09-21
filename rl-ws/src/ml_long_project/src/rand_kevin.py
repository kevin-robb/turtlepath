#!/usr/bin/env python

import rospy
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
r15_scan = None


def check_scan(scan_msg):
    global halt, fwd_scan, rear_scan, l15_scan, r15_scan
    # scan_msg.ranges is an array of 640 elements representing 
    # distance measurements in a full circle around the robot (0=fwd, CCW?)

    # update the important entries (TODO verify these indexes)
    fwd_scan = scan_msg.ranges[0]
    rear_scan = scan_msg.ranges[320]
    l15_scan = scan_msg.ranges[30]
    r15_scan = scan_msg.ranges[610]
    print([fwd_scan, l15_scan, rear_scan, r15_scan])

    # if scan_msg.ranges[0] < 1.0:
    #     halt = True


def send_command(timer_event):
    if halt:
        fwd_speed = 0
        turn_speed = 0
    else:
        # set random forward and turn speeds (TODO turn away from walls)
        fwd_speed = 1
        turn_speed = 0
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
