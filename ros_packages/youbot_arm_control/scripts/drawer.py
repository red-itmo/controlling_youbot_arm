#!/usr/bin/env python
import rospy
import threading
from math import sin, cos, pi

import tf.transformations as tftr

from std_msgs.msg import Header
from youbot_arm_control.msg import DecartTrajectory
from geometry_msgs.msg import PolygonStamped, Pose, PoseArray, Quaternion


class Drawer:

    def __init__(self):
        self.poses = []
        self.points = []
        self.header = Header()
        self.lock = threading.Lock()

        rospy.init_node('drawer_node')

        self.sub = rospy.Subscriber("decart_trajectory", DecartTrajectory, self.decartTrajectoryCallback)
        self.pub_polygon = rospy.Publisher("trajectory_polygon", PolygonStamped, queue_size=1)
        self.pub_pose_array = rospy.Publisher("trajectory_poses", PoseArray, queue_size=1)

        self.loop()


    def decartTrajectoryCallback(self, msg):
        self.lock.acquire()
        self.header = msg.header
        self.points = [msg.poses[i].position for i in range(len(msg.poses))]
        self.poses = msg.poses
        self.lock.release()


    def drawPolygon(self):
        msg = PolygonStamped()
        msg.header = self.header
        msg.polygon.points = self.points
        self.pub_polygon.publish(msg)


    def drawPose(self):
        msg = PoseArray()
        msg.header = self.header
        for i, pose in enumerate(self.poses):
            msg.poses.append(Pose())
            msg.poses[i].position = pose.position
            help_quat = tftr.quaternion_multiply([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], [0.0, sin(-pi/4), 0.0, cos(-pi/4)])
            msg.poses[i].orientation = Quaternion(help_quat[0], help_quat[1], help_quat[2], help_quat[3])
        self.pub_pose_array.publish(msg)


    def loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.lock.acquire()
            self.drawPolygon()
            self.drawPose()
            self.lock.release()
            rate.sleep()


if __name__ == '__main__':
    Drawer()
