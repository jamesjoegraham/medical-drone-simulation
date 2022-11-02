

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose

rospy.init_node("waypoint_test")
pose_pub = rospy.Publisher('command/pose', PoseStamped, queue_size=1)

pose_msg = PoseStamped()
pose_msg.pose.position.z = 10
pose_pub.publish((pose_msg))

rospy.spin()