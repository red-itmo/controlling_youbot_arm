#!/usr/bin/python

import rospy
import string
import threading
from sensor_msgs.msg import JointState
from brics_actuator.msg import JointPositions, JointValue
from std_srvs.srv import Empty, EmptyResponse


def js_callback(js_msg):
    global file_with_data, cur_angles
    angles = ' '.join(str(angle) for angle in js_msg.position[:5])
    omegas = ' '.join(str(omega) for omega in js_msg.velocity[:5])
    torques = ' '.join(str(torque) for torque in js_msg.effort[:5])
    time = "%.6f" % (js_msg.header.stamp.to_sec() - start_time)
    lock.acquire()
    cur_angles = js_msg.position[:5]
    file_with_data.write(angles + ' ' + omegas + ' ' + torques + ' ' + time + '\n')
    lock.release()


def make_movements_callback(req):
    global file_with_data

    jp_pub = rospy.Publisher("arm_1/arm_controller/position_command", JointPositions, queue_size=10, latch=True)
    jp_msg = JointPositions()
    for i in range(5):
        jp_msg.positions.append(JointValue())
        jp_msg.positions[i].joint_uri = 'arm_joint_' + str(i+1)
        jp_msg.positions[i].unit = 'rad'

    file_with_poses = open(fwp_name, "r")
    string_of_poses = file_with_poses.readlines()
    file_with_poses.close()

    js_sub = rospy.Subscriber("joint_states", JointState, js_callback)

    r = rospy.Rate(1.0)
    for pose in string_of_poses:
        angles = [float(angle) for angle in pose.split()]
        for i in range(5):
            jp_msg.positions[i].value = angles[i]
        jp_pub.publish(jp_msg)

        achieved = False
        while not achieved:
            achieved = True
            r.sleep()
            lock.acquire()
            for i in range(5):
                if abs(angles[i] - cur_angles[i]) > 0.2:
                    achieved = False
                    break
            lock.release()

    js_sub.unregister()

    # for returning to startup (embryo) position
    for i in range(5):
        jp_msg.positions[i].value = 0.025
    jp_msg.positions[2].value = -0.02
    jp_msg.positions[4].value = 0.12
    jp_pub.publish(jp_msg)

    lock.acquire()
    file_with_data.close()
    lock.release()
    return EmptyResponse()


if __name__=="__main__":

    rospy.init_node("collect_ident_data_node")

    cur_angles = [0.0]*5
    lock = threading.Lock()

    fwp_name = rospy.get_param("~file_with_poses", "poses.txt")
    fwd_name = rospy.get_param("~file_with_data", "data.txt")

    file_with_data = open(fwd_name, "w")

    start_time = rospy.Time.now().to_sec()

    make_movements = rospy.Service("make_movements", Empty, make_movements_callback)

    rospy.spin()
