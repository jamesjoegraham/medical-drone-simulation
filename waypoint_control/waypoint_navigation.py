#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
import json
import os
import sys, signal

# Used to allow for CTRL-C (SIGINT) to exit when in while loop
def signal_handler(signal, frame):
    print("\nProgram Exiting")
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

cur_pose = Pose()
def gazebo_pose_callback(msg):
    global cur_pose
    cur_pose = msg.pose

def arrived_at_waypoint(current, waypoint):
    dx = abs(waypoint.position.x - current.position.x)
    dy = abs(waypoint.position.y - current.position.y)
    dz = abs(waypoint.position.z - current.position.z)

    close_enough = 0.5

    # If we're in this range
    if dx < close_enough and dy < close_enough and dz < close_enough:
        return True

    return False


# Open Waypoints JSON File

input_file =  input("Enter .json file name for waypoints: ")
waypoint_file = open(os.path.dirname(__file__) + "/" + input_file)
waypoints = json.load(waypoint_file)

# Initialize ROS Functionality
rospy.init_node('waypoint_manager')
pose_pub = rospy.Publisher('/command/pose', PoseStamped, queue_size=1)
rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, gazebo_pose_callback)

# Wait for connections before publishing messsage
while(pose_pub.get_num_connections() < 1):
    pass

for waypoint in waypoints:
    print("Moving to waypoint")
    print(waypoint)
    pose = Pose()
    pose.position.x = waypoint["x"]
    pose.position.y = waypoint["y"]
    pose.position.z = waypoint["z"]

    pose_msg = PoseStamped()
    pose_msg.pose = pose

    pose_pub.publish(pose_msg)

    # Wait until waypoint has been reached
    while(not arrived_at_waypoint(cur_pose, pose_msg.pose)):
        pass

    print("Arrived at waypoint!")

    # Wait a small period of time before moving to the next waypoint
    rospy.sleep(0.2)