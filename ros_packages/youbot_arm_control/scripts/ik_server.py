#!/usr/bin/env python
import rospy
import tf
import tf.transformations as tftr

from youbot_arm_control.srv import IK
from sensor_msgs.msg import JointState

from libs.kinematics import Kinematics
from config.config import Config


class IKServer:

    def __init__(self):
        self.pubJointState = rospy.Publisher("joint_states", JointState,
                                             queue_size=1)

        self.ks = Kinematics(Config.DH_A, Config.DH_ALPHA,
                             Config.DH_D, Config.DH_THETA)

        # start sever
        self.ikServer()

    def handle(self, req):
        xyz = req.xyz
        rpy = req.rpy
        qtn = tftr.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

        # set orange brick to given coordinate with given orientation
        br = tf.TransformBroadcaster()
        br.sendTransform(xyz, qtn, rospy.Time.now(), 'test_box', "base_link")

        # solve IK problem
        q = self.ks.inverse(xyz, rpy)
        if type(q) is not int:
            # !select first solve from two
            q = q[0]
            rospy.loginfo("found Qs: %s" % str(q))

            # publish joint positions
            jointState = JointState()
            jointState.header.stamp = rospy.Time.now()
            # jointState.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'finger_joint_l', 'finger_joint_r']
            jointState.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
            jointState.position = q.tolist()
            # add positions for fingers of gripper
            # jointState.position.extend([0.015, 0.015])
            self.pubJointState.publish(jointState)
            return 1
        return 0

    def ikServer(self):
        rospy.init_node('ik_server')
        ik = rospy.Service('ik', IK, self.handle)
        print "Ready to set pos and ori."
        rospy.spin()

if __name__ == "__main__":
    IKServer()
