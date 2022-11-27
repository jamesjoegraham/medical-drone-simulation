from __future__ import print_function

import sys
import rospy
import json
import os
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, PoseStamped


# def apply_body_wrench_client(x, y, z):
#     rospy.wait_for_service('apply_body_wrench')
#     try:
#         apply_body_wrench = rospy.ServiceProxy('apply_body_wrench', )

# Initialize ROS Functionality
rospy.init_node('command_pose_listener')
pose_pub = rospy.Subscriber('/command/pose', PoseStamped, queue_size=1)
rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, gazebo_pose_callback)

def applyForce(wrench, wrench_duration): 
    rospy.wait_for_service('/gazebo/apply_body_wrench') 
    force = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)


    
    # You can also define the start time if necessary... 
    success = force(body_name = "quadrotor::base_link",wrench = wrench, duration = rospy.Duration(wrench_duration))
    return success


def get_wrench(wrench_file):
    if not wrench_file:
        wrench_file = "ImpulseTest"
    if not wrench_file.endswith('.json'):
        wrench_file = wrench_file + '.json'
    wrench_json = open(os.path.dirname(__file__) + "/wrench_json/" + wrench_file)
    wrenches = json.load(wrench_json)

    # get wrench from json file
    for wrenchj in wrenches:
        wrench = Wrench()
        wrench.force.x  = wrenchj["force x"]
        wrench.force.y  = wrenchj["force y"]
        wrench.force.z  = wrenchj["force z"]
        wrench.torque.x = wrenchj["torque x"]
        wrench.torque.y = wrenchj["torque y"]
        wrench.torque.z = wrenchj["torque z"]
        wrench_type = wrenchj["type"]
        wrench_duration = wrenchj["duration"]



        success = applyForce(wrench, wrench_duration)

        if success:
            print('Body wrench perturbation applied!')
            # print('\tFrame: ', body_name)
            # print('\tDuration [s]: ', duration)
            # print('\tForce [N]: ', force)
            # print('\tTorque [Nm]: ', torque)
        else:
            print('Failed!')


if __name__ == "__main__":
    # Open Waypoints JSON File
    # wrench_file =  input("Enter .json file name for impulses: ")
    
    wrench_file = rospy.get_param('/apply_body_wrench_script/wrenches_json')
    get_wrench(wrench_file)

    