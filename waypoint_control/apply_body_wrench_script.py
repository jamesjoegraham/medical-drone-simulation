from __future__ import print_function

import sys
import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench


# def apply_body_wrench_client(x, y, z):
#     rospy.wait_for_service('apply_body_wrench')
#     try:
#         apply_body_wrench = rospy.ServiceProxy('apply_body_wrench', )

def applyForce(): 
    rospy.wait_for_service('/gazebo/apply_body_wrench') 
    force = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)


    wrench = Wrench()
    wrench.force.x  = 10
    wrench.force.y  = 0
    wrench.force.z  = 0
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0
    # You can also define the start time if necessary... 
    success = force(body_name = "quadrotor::base_link",wrench = wrench, duration = rospy.Duration(10))
    return success


if __name__ == "__main__":
    success = applyForce()



    if success:
        print('Body wrench perturbation applied!')
        # print('\tFrame: ', body_name)
        # print('\tDuration [s]: ', duration)
        # print('\tForce [N]: ', force)
        # print('\tTorque [Nm]: ', torque)
    else:
        print('Failed!')