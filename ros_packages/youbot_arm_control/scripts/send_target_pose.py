#!/usr/bin/python

import rospy
import string
import threading
from brics_actuator.msg import JointPositions, JointValue
from std_msgs.msg import Float64MultiArray


def poseCallback(simple_msg):
    hard_msg = JointPositions()
    for i in range(5):
        hard_msg.positions.append(JointValue())
        hard_msg.positions[i].joint_uri = 'arm_joint_' + str(i+1)
        hard_msg.positions[i].unit = 'rad'
        hard_msg.positions[i].value = simple_msg.data[i]
    pose_pub.publish(hard_msg)


if __name__=="__main__":

    rospy.init_node("collect_ident_data_node")

    pose_sub = rospy.Subscriber("simple_pose", Float64MultiArray, poseCallback)
    pose_pub = rospy.Publisher('target_pose', JointPositions, queue_size=10, latch=True)

    rospy.spin()
