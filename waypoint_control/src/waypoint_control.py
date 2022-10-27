#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


#euler_sub = rospy.Subscriber('euler', Twist, euler_callback)

rospy.init_node('up_down')
rate = rospy.Rate(0.5)

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


while True:
    twist_msg = Twist()
    twist_msg.linear.z = 3
    cmd_vel_pub.publish((twist_msg))
    rate.sleep()
    twist_msg.linear.z = -3
    cmd_vel_pub.publish((twist_msg))
    rate.sleep()


# Run the ros node. Everything after this is automatic
rospy.spin()