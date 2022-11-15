#!/usr/bin/env python3
from __future__ import print_function
import rospy
import sys
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Wrench, Vector3

if __name__ == '__main__':
    print('Apply programmed perturbation to vehicle', rospy.get_namespace())
    rospy.init_node('set_body_wrench')
        

    if rospy.is_shutdown():
        print('ROS master not running!')
        sys.exit(-1)

    starting_time = 0.0
    if rospy.has_param('~starting_time'):
        starting_time = rospy.get_param('~starting_time')

    print('Starting time= {} s'.format(starting_time))

    # duration = 0.0
    duration = 10.0
    if rospy.has_param('~duration'):
        duration = rospy.get_param('~duration')

    if duration == 0.0:
        print('Duration not set, leaving node...')
        sys.exit(-1)

    print('Duration [s]=', ('Inf.' if duration < 0 else duration))

    force = [0, 0, 0]
    if rospy.has_param('~force'):
        force = rospy.get_param('~force')
        print(force)
        if len(force) != 3:
            raise rospy.ROSException('Invalid force vector')

    print('Force [N]=', force)

    torque = [500, 0, 0]
    if rospy.has_param('~torque'):
        torque = rospy.get_param('~torque')
        if len(torque) != 3:
            raise rospy.ROSException('Invalid torque vector')

    print('Torque [N]=', torque)

    try:
        rospy.wait_for_service('/gazebo/apply_body_wrench', timeout=10)
       
    except rospy.ROSException:
        print('Service not available! Closing node...')
        sys.exit(-1)

    try:
        apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    except rospy.ServiceException as e:
        print('Service call failed, error=', e)
        sys.exit(-1)

    ns = rospy.get_namespace().replace('/', '')
    body_name = '%sbase_link' % ns

    # # body_name = '%s/base_link' % ns

    if starting_time >= 0:
        rate = rospy.Rate(100)
        while rospy.get_time() < starting_time:
            rate.sleep()

    wrench = Wrench()
    wrench.force = Vector3(*force)
    wrench.torque = Vector3(*torque)
    success = apply_wrench(
        body_name,
        'world',
        Point(0, 0, 0),
        wrench,
        rospy.Time().now(),
        rospy.Duration(duration))

    if success:
        print('Body wrench perturbation applied!')
        print('\tFrame: ', body_name)
        print('\tDuration [s]: ', duration)
        print('\tForce [N]: ', force)
        print('\tTorque [Nm]: ', torque)
    else:
        print('Failed!')
