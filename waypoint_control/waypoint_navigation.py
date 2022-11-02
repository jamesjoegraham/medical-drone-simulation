#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
from gazebo_msgs.msg import LinkStates
import json
import os

#cur_pose = Pose()
#def gazebo_pose_callback(msg):
#    cur_pose = msg.pose

# Open Waypoints JSON File
waypoint_file = open(os.path.dirname(__file__)+ '/waypoints_1.json')
waypoints = json.load(waypoint_file)

# Initialize ROS Functionality
rospy.init_node('waypoint_manager')
pose_pub = rospy.Publisher('/command/pose', PoseStamped, queue_size=1)
#rospy.Subscriber("gazebo/link_states", LinkStates, gazebo_pose_callback)

for waypoint in waypoints:
    print("Moving to waypoint")
    print(waypoint)
    pose = Pose()
    pose.position.x = waypoint["x"]
    pose.position.y = waypoint["y"]
    pose.position.z = waypoint["z"]

    pose_msg = PoseStamped()
    pose_msg.pose = pose

    print(pose_msg)

    
    #wait for connections before publishing messsage
    while(pose_pub.get_num_connections() < 1):
        pass
    
    pose_pub.publish(pose_msg)
    rospy.spin()

    # Wait until waypoint has been reached

rospy.spin()