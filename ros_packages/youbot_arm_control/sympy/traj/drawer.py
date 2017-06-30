#!/usr/bin/env python2
import rospy
import math, scipy
# from youbot_arm_control.msg import DecartTrajectory
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


class Drawer:

    def __init__(self):
        # self.sub = rospy.Subscriber("decart_trajectory", DecartTrajectory, self.drawme)
        self.pubPolygon = rospy.Publisher("my_traj_polygon", PolygonStamped, queue_size=1)
        self.pubPoseArray = rospy.Publisher("my_traj_pose", PoseArray, queue_size=1)

        self.h = 0
        self.loop()

    def drawme(self, point):
        polygon = Polygon([point])
        msg = PolygonStamped()
        msg.polygon = polygon
        self.pub.publish(msg)

    def getXYZ(self):
        u, h = 10, 3
        x = u * math.cos(self.h)
        y = u * math.sin(self.h)
        z = h * self.h
        if self.h > 1000000000:
            self.h = 0
        self.h += 0.001
        print(self.h)
        return x, y, z

    def drawPolygon(self):
        x, y, z = self.getXYZ()
        points = []
        cur_point = Point32(x, y, z)
        points.append(cur_point)
        pol = Polygon()
        pol.points = points
        msg = PolygonStamped()
        msg.polygon = pol
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'g'
        self.pubPolygon.publish(msg)

    def drawPose(self):
        x, y, z = self.getXYZ()
        msg = PoseArray()
        msg.header.frame_id = 'pose_test'
        msg.header.stamp = rospy.Time.now()
        msg.poses.append(Point(x, y, z))
        self.pubPoseArray.publish(msg)

    def loop(self):
        rospy.init_node('drawer')
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            self.drawPolygon()
            rate.sleep()

if __name__ == '__main__':
    Drawer()
