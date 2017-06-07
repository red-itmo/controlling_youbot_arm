#!/usr/bin/python

import rospy
import string
import threading
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse, Empty, EmptyResponse


def js_callback(js_msg):
    global cur_angles
    lock.acquire()
    cur_angles = js_msg.position
    lock.release()


def add_pose_callback(rdoeq):
    global counter
    counter += 1

    config_file = open(file_name, "a")
    lock.acquire()
    config_file.write(' '.join(str(angle) for angle in cur_angles[:5]))
    lock.release()
    config_file.write('\n')
    config_file.close()

    return TriggerResponse(True, 'Pose No ' + str(counter) + ' has been saved')


def show_poses_callback(req):

    time_between_poses = 2.0
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'gripper_finger_joint_l', 'gripper_finger_joint_r']
    msg.position = [0.01]*7;

    angles_start = [0.0]*5
    msg.position[:5] = angles_start
    js_pub.publish(msg)
    rospy.sleep(1.0)

    config_file = open(file_name, "r")
    pose = config_file.readline()
    while bool(pose):
        angles_final = [float(angle) for angle in pose.split()]
        start_time = rospy.Time.now()
        cur_time = start_time
        while (cur_time - start_time).to_sec() < time_between_poses:
            relative_time = (cur_time - start_time).to_sec() / time_between_poses
            for i in range(5):
                msg.position[i] = angles_start[i] + (angles_final[i] - angles_start[i]) * relative_time
            js_pub.publish(msg)
            rospy.sleep(0.1)
            cur_time = rospy.Time.now()
        angles_start = angles_final
        pose = config_file.readline()
    config_file.close()

    return EmptyResponse()


if __name__=="__main__":

    rospy.init_node("make_poses_list_node")

    counter = 0
    cur_angles = []
    lock = threading.Lock()

    file_name = rospy.get_param("~file_with_poses", "poses.txt")

    js_sub = rospy.Subscriber("joint_states", JointState, js_callback)
    js_pub = rospy.Publisher("poses/joint_states", JointState, queue_size=10)
    add_pose = rospy.Service('add_pose', Trigger, add_pose_callback)
    show_poses = rospy.Service('show_poses', Empty, show_poses_callback)

    rospy.spin()
