#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Quaternion
import json
import os
import sys, signal
from tf.transformations import quaternion_from_euler

# Used to allow for CTRL-C (SIGINT) to exit when in while loop
def signal_handler(signal, frame):
    print("\nProgram Exiting")
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

cur_pose = Pose()
def gazebo_pose_callback(msg):
    global cur_pose
    cur_pose = msg.pose

def arrived_at_waypoint(current, waypoint, close_enough):
    dx = abs(waypoint.position.x - current.position.x)
    dy = abs(waypoint.position.y - current.position.y)
    dz = abs(waypoint.position.z - current.position.z)

    # If we're in this range
    if dx < close_enough and dy < close_enough and dz < close_enough:
        return True

    return False

# Open Waypoints JSON File
input_file =  input("Enter .json file name for waypoints: ")

if not input_file:
    input_file = "CStest"
if not input_file.endswith('.json'):
    input_file = input_file + '.json'
    
waypoint_file = open(os.path.dirname(__file__) + "/" + input_file)
waypoints = json.load(waypoint_file)

# Initialize ROS Functionality
rospy.init_node('waypoint_manager')
pose_pub = rospy.Publisher('/command/pose', PoseStamped, queue_size=1)
rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, gazebo_pose_callback)

# Wait for connections before publishing messsage
while(pose_pub.get_num_connections() < 1):
    pass

# Waypoint settings set to defualts
waypoint_type = "fast" 
passenger_pickup = "no"

for waypoint in waypoints:
    print("Moving to waypoint")
    print(waypoint)
    pose = Pose()
    pose.position.x = waypoint["x"]
    pose.position.y = waypoint["y"]
    pose.position.z = waypoint["z"]

    yaw = 0
    if "yaw" in waypoint:
        yaw = waypoint["yaw"]
    q = quaternion_from_euler(0, 0, yaw/57.29)
    ori = Quaternion()
    ori.w = q[3]
    ori.x = q[0]
    ori.y = q[1]
    ori.z = q[2]
    pose.orientation = ori

    # New parameter for waypoint to sepecify if the drone should move through it quickly or take time to stabilize
    try: 
        waypoint_type = waypoint["type"]
    except:
        pass
    # new parameter to specify if the drone should wait as if its picking up a passenger
    try: 
        passenger_pickup = waypoint["passenger pickup"]
    except:
        pass

    pose_msg = PoseStamped()
    pose_msg.pose = pose

    pose_pub.publish(pose_msg)

    # Wait until waypoint has been reached. Move fast or stabilize depending on waypoint type
    if waypoint_type == "fast":
        while(not arrived_at_waypoint(cur_pose, pose_msg.pose, 6.0)):
            pass
    elif waypoint_type == "stable":
        while(not arrived_at_waypoint(cur_pose, pose_msg.pose, 0.8)):
            pass
        rospy.sleep(0.7)

    print("Arrived at waypoint!")

    # check to see if theres a passenger pickup
    if passenger_pickup == "yes":
        rospy.sleep(4)
    
    # Wait a small period of time before moving to the next waypoint
    rospy.sleep(0.2)