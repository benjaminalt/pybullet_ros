#!/usr/bin/python2.7
import rospy
import tf


class TFPublisher:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        self.environment_id = kargs["environment_id"]
        self.tf_broadcaster = tf.TransformBroadcaster()

    def execute(self):
        """this function gets called from pybullet ros main update loop"""
        num_joints = self.pb.getNumJoints(self.environment_id)
        # In PyBullet, jointIndex == linkIndex
        link_infos = self.pb.getLinkStates(self.environment_id, range(num_joints))
        time = rospy.Time.now()
        for link_id, link_info in enumerate(link_infos):
            link_name = self.pb.getJointInfo(self.environment_id, link_id)[12]
            position_world = link_info[0]
            orientation_world = link_info[1]
            t = tuple(map(lambda x: round(x, 4), position_world))
            r = tuple(map(lambda x: round(x, 4), orientation_world))
            self.tf_broadcaster.sendTransform(t, r, time, link_name, "world")
