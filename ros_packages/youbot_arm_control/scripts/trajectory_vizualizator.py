#!/usr/bin/python

import rospy
import string
import threading
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


def jointTrajectoryCallback(msg):
    global joint_trajectory
    lock.acquire()
    joint_trajectory = msg
    lock.release()


def showTrajectoryCallback(req):
    js_msg = JointState()
    js_msg.name = ['arm_joint_' + str(i+1) for i in range(5)]
    lock.acquire()
    if len(joint_trajectory.points) == 0:
        lock.release()
        return TriggerResponse(False, 'No one or empty trajectory have been received!')
    for i, pose in enumerate(joint_trajectory.points):
        js_msg.position = pose.positions
        js_pub.publish(js_msg)
        if i != len(joint_trajectory.points) - 1:
            dt = joint_trajectory.points[i+1].time_from_start.to_sec() - pose.time_from_start.to_sec()
            rospy.sleep(dt)
        else:
            break
    lock.release()
    return TriggerResponse(True, 'Successfully done!')


if __name__=="__main__":
    rospy.init_node("trajectory_vizualizator_node")

    lock = threading.Lock()
    joint_trajectory = JointTrajectory()
    joint_trajectory.points = []

    jt_sub = rospy.Subscriber("joint_trajectory", JointTrajectory, jointTrajectoryCallback)
    js_pub = rospy.Publisher("poses/joint_states", JointState, queue_size=1, latch=True)
    show_poses = rospy.Service('show_trajectory', Trigger, showTrajectoryCallback)

    rospy.spin()
