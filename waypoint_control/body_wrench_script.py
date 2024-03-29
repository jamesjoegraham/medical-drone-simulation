from __future__ import print_function

import sys
import rospy
import json
import math
import time
import os
from std_msgs.msg import Header
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.msg import LinkStates


# Initialize ROS Functionality
# rospy.init_node('command_pose_listener')
# pose_pub = rospy.Subscriber('/command/pose', PoseStamped, queue_size=1)
# rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, gazebo_pose_callback)

def apply_force(wrench, wrench_duration): 
    rospy.wait_for_service('/gazebo/apply_body_wrench') 
    force = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)
    # You can also define the start time if necessary... 
    success = force(body_name = "quadrotor::base_link",wrench = wrench, duration = rospy.Duration(wrench_duration))
    return success

def create_wrench(wrenchj):
    wrench = Wrench()
    if "force x" in wrenchj: wrench.force.x  = wrenchj["force x"]
    if "force y" in wrenchj: wrench.force.y  = wrenchj["force y"]
    if "force z" in wrenchj: wrench.force.z  = wrenchj["force z"]
    if "torque x" in wrenchj: wrench.torque.x = wrenchj["torque x"]
    if "torque y" in wrenchj: wrench.torque.y = wrenchj["torque y"]
    if "torque z" in wrenchj: wrench.torque.z = wrenchj["torque z"]

    return wrench

def print_success(success):
    if success:
        print('Body wrench perturbation applied!')
        # print('\tFrame: ', body_name)
        # print('\tDuration [s]: ', duration)
        # print('\tForce [N]: ', force)
        # print('\tTorque [Nm]: ', torque)
    else:
        print('Failed!')


def sin_func(freq, ampl, t):
    return ampl*math.sin(2*math.pi*freq*t)

def step_func(freq, ampl, t):
    frac, w = math.modf(2*t*freq)
    return ampl if frac < 0.5 else 0

def ramp_func(freq, ampl, t):
    frac, w = math.modf(t*freq)
    return ampl*frac

def get_wrench(wrench_file):
    # open the file
    if not wrench_file:
        wrench_file = "ImpulseTest"
        return
    if not wrench_file.endswith('.json'):
        wrench_file = wrench_file + '.json'
    print('Opening file {0}'.format(wrench_file))
    wrench_json = open(os.path.dirname(__file__) + "/wrench_json/" + wrench_file)
    wrenches = json.load(wrench_json)

    # iterate through the wrenches and run for duration
    for wrenchj in wrenches:

        print(wrenchj)
        wrench_type = wrenchj["type"]

        if wrench_type == "oscillation" or wrench_type == "ramp":
            frequency = wrenchj["frequency"]
            amplitude = wrenchj["amplitude"]
            duration = wrenchj["duration"]
            osc_type = wrenchj["osc_type"]
            dimension = wrenchj["dimension"]

            if osc_type == "sin":
                osc_func = sin_func
            elif osc_type == "step":
                osc_func = step_func
            elif osc_type == "ramp":
                osc_func = ramp_func

            start_time = time.time()
            cur_time = time.time() - start_time

            force_pub = rospy.Publisher("current_force", WrenchStamped, queue_size=10)

            print('Start time={0}'.format(start_time))
            while (cur_time < duration):
                test_s_t = cur_time
                cur_force = osc_func(frequency, amplitude, cur_time)
                wrenchj[dimension] = cur_force
                wrench = create_wrench(wrenchj)
                success = apply_force(wrench, 0.05)
                cur_time = time.time() - start_time
                
                # Publish current force to topic current_force
                w_stamped = WrenchStamped()
                w_header = Header()
                w_header.stamp = rospy.Time.now()
                w_stamped.header = w_header
                w_stamped.wrench = wrench
                force_pub.publish(w_stamped)

                rospy.sleep(0.05)

        elif wrench_type == "impulse":
            wrench_duration = wrenchj["duration"]
            wrench = create_wrench(wrenchj)
            success = apply_force(wrench, wrench_duration)
            print_success(success)
            rospy.sleep(wrench_duration)

if __name__ == "__main__":
    # Open Waypoints JSON File
    # wrench_file =  input("Enter .json file name for impulses: ")
    
    wrench_file = rospy.get_param('/body_wrench_script/wrenches_json')
    rospy.init_node('body_wrench_automation')
    # wrench_file = "oscillation"

    # wait for gazebo to be unpaused before proceeding
    while not rospy.wait_for_message('gazebo/link_states', LinkStates):
        pass


    get_wrench(wrench_file)

    