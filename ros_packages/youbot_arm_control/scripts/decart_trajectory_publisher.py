#!/usr/bin/python
import tf
import rospy
import threading

import tf.transformations as tftr
from geometry_msgs.msg import Pose, Point, Quaternion
from youbot_arm_control.msg import DecartTrajectory


if __name__ == "__main__":
    rospy.init_node('decart_trajectory_publisher_node')
    traj_pub = rospy.Publisher("decart_trajectory", DecartTrajectory, queue_size=1)

    trajectory = DecartTrajectory()
    trajectory.poses = []
    trajectory.time_from_start = []
    trajectory.header.frame_id = 'ground_link'

    for i in range(81):
        trajectory.poses.append(Pose())
        trajectory.poses[i] = Pose(Point(0.2, -0.2+i*0.005, 0.0), Quaternion(1.0, 0.0, 0.0, 0.0))
        trajectory.time_from_start.append(rospy.Duration(i*0.0625))

    while not rospy.is_shutdown():
        trajectory.header.stamp = rospy.Time.now()
        traj_pub.publish(trajectory)
        rospy.sleep(1.0)
