#!/usr/bin/python
import tf
import rospy
import threading

from config.config import Config
from libs.kinematics import Kinematics

import tf.transformations as tftr
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Trigger, TriggerResponse
from youbot_arm_control.msg import DecartTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


class Dec2JointTranslator:

    def __init__(self):
        self.lock = threading.Lock()
        self.kinematics = Kinematics(Config.DH_A, Config.DH_ALPHA, Config.DH_D, Config.DH_THETA)

        rospy.init_node('dec2joint_translator_node')

        self.points_number = 0
        self.dec_points = PoseArray()
        self.time_from_start = []
        self.joint_trajectory = JointTrajectory()
        self.joint_trajectory.joint_names = ['arm_joint_' + str(i+1) for i in range(5)]
        self.joint_trajectory.header.stamp = rospy.Time.now()

        self.min_q = rospy.get_param("~min_q", [0.1, 0.1, -5.08, 0.1, 0.1])
        self.max_q = rospy.get_param("~max_q", [5.8, 2.6, -0.1, 3.48, 5.75])
        self.file_with_trajectory_name = rospy.get_param("~file_with_trajectory_name", "file_with_trajectory_name.txt")

        self.traj_sub = rospy.Subscriber("decart_trajectory", DecartTrajectory, self.trajectoryCallback)
        self.traj_pub = rospy.Publisher("joint_trajectory", JointTrajectory, queue_size=1)
        rospy.Service('translate_trajectory', Trigger, self.translateCallback)

        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self.lock.acquire()
            self.traj_pub.publish(self.joint_trajectory)
            self.lock.release()
            r.sleep()


    def trajectoryCallback(self, msg):
        self.lock.acquire()
        self.time_from_start = []
        self.dec_points.poses = []
        self.dec_points.header = msg.header
        self.points_number = len(msg.poses)
        for i, pose in enumerate(msg.poses):
            self.dec_points.poses.append(pose)
            self.time_from_start.append(msg.time_from_start[i])
        self.lock.release()


    def translateCallback(self, req):
        last_q = [0.0]*5
        self.lock.acquire()
        self.joint_trajectory.points = []
        for j in range(4):
            trajectory_was_translated = True
            self.file_with_trajectory = open(self.file_with_trajectory_name, "w")
            for i, pose in enumerate(self.dec_points.poses):
                xyz = [pose.position.x, pose.position.y, pose.position.z]
                rpy = tftr.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                qs, solution_was_found = self.kinematics.inverse(xyz, rpy)
                q = qs[j]
                if solution_was_found[j/2] != True:
                    rospy.loginfo("Set of angles No%d for pose No%d can't be found!" % (j+1, i+1))
                    trajectory_was_translated = False
                    self.joint_trajectory.points = []
                    self.file_with_trajectory.close()
                    break
                if not self.inLimits(q):
                    rospy.loginfo("Set of angles No%d for pose No%d is out of range!" % (j+1, i+1))
                    trajectory_was_translated = False
                    self.joint_trajectory.points = []
                    self.file_with_trajectory.close()
                    break
                self.joint_trajectory.points.append(JointTrajectoryPoint())
                self.joint_trajectory.points[i].time_from_start = self.time_from_start[i]
                self.joint_trajectory.points[i].positions = q
                self.calculateVelsAndAccels(i, q, last_q)
                self.writeToFile(i)
                last_q = q
            if trajectory_was_translated:
                rospy.loginfo("Successfully translated trajectory with set of angles No %d" % (j+1))
                self.joint_trajectory.header = self.dec_points.header
                self.lock.release()
                self.file_with_trajectory.close()
                return TriggerResponse(True, "Trajectory was successfully translated!")
        self.lock.release()
        return TriggerResponse(False, "Trajectory in the joint space can't be found!")


    def inLimits(self, q):
        for j in range(5):
            if q[j] < self.min_q[j] or q[j] > self.max_q[j]:
                return False
        return True


    # TODO possibly the principle of calculation first and last vels and accels will be changed
    def calculateVelsAndAccels(self, i, q, last_q):
        if i == 1:
            dt1 = self.joint_trajectory.points[i].time_from_start.to_sec() - self.joint_trajectory.points[i-1].time_from_start.to_sec()
            for j in range(5):
                self.joint_trajectory.points[i-1].velocities.append( (q[j] - last_q[j]) / dt1 )
        elif i != 0:
            dt1 = self.joint_trajectory.points[i].time_from_start.to_sec() - self.joint_trajectory.points[i-1].time_from_start.to_sec()
            dt2 = self.joint_trajectory.points[i-1].time_from_start.to_sec() - self.joint_trajectory.points[i-2].time_from_start.to_sec()
            for j in range(5):
                self.joint_trajectory.points[i-1].velocities.append( (q[j] - last_q[j]) / dt1 )
                vel2 = self.joint_trajectory.points[i-1].velocities[j]
                vel1 = self.joint_trajectory.points[i-2].velocities[j]
                self.joint_trajectory.points[i-2].accelerations.append( (vel2 - vel1) / dt2 )
            if i == self.points_number - 1:
                self.joint_trajectory.points[i].velocities = [0.0]*5
                self.joint_trajectory.points[i].accelerations = [0.0]*5
                for j in range(5):
                    vel1 = self.joint_trajectory.points[i-1].velocities[j]
                    self.joint_trajectory.points[i-1].accelerations.append( (0.0 - vel1) / dt1 )


    def writeToFile(self, i):
        if i > 1:
            self.writeLine(i-2)
        if i == self.points_number - 1:
            self.writeLine(i-1)
            self.writeLine(i)


    def writeLine(self, ind):
        self.file_with_trajectory.write(' '.join(str(angle) for angle in self.joint_trajectory.points[ind].positions))
        self.file_with_trajectory.write(' ')
        self.file_with_trajectory.write(' '.join(str(vel) for vel in self.joint_trajectory.points[ind].velocities))
        self.file_with_trajectory.write(' ')
        self.file_with_trajectory.write(' '.join(str(accel) for accel in self.joint_trajectory.points[ind].accelerations))
        self.file_with_trajectory.write(' ' + str(self.joint_trajectory.points[ind].time_from_start.to_sec()))
        self.file_with_trajectory.write('\n')



if __name__ == "__main__":
    Dec2JointTranslator()
