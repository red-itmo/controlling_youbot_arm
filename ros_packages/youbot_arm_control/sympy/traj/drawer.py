#!/usr/bin/env python2
import rospy
import math

# from youbot_arm_control.msg import DecartTrajectory
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class Drawer:

    def __init__(self):
        # self.sub = rospy.Subscriber("decart_trajectory", DecartTrajectory, self.drawme)
        self.pubPolygon = rospy.Publisher("my_traj_polygon", PolygonStamped, queue_size=10)
        self.pubPoseArray = rospy.Publisher("my_traj_pose", PoseArray, queue_size=1)

        self.height_test = 0
        self.frame = 'frame_drawer'
        self.points = []
        self.poses = []
        self.loop()

    def getXYZ(self):
        """ only for test"""
        x, y, z = math.sin(self.height_test), \
                  math.cos(self.height_test), \
                  0.042 * self.height_test
        self.height_test += 0.1
        o = math.sin(self.height_test), math.cos(self.height_test), 0, 1
        return x, y, z, o

    def drawPolygon(self, point):
        x, y, z = point
        self.points.append(Point32(x, y, z))
        # fill msg
        msg = PolygonStamped()
        msg.polygon.points = self.points
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        self.pubPolygon.publish(msg)

    def drawPose(self, point, orientation):
        x, y, z = point
        qx, qy, qz, w = orientation
        pose = Pose()
        pose.position = Point(x, y, z)
        pose.orientation = Quaternion(qx, qy, qz, w)
        self.poses.append(pose)
        # fill msg
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame
        msg.poses = self.poses
        self.pubPoseArray.publish(msg)

    def loop(self):
        rospy.init_node('drawer')
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # test draw code
            x, y, z, o = self.getXYZ()
            self.drawPolygon((x, y, z))
            self.drawPose((x, y, z), o)

            rate.sleep()

if __name__ == '__main__':
    Drawer()
