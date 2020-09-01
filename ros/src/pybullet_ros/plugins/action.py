#! /usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult


class ActionServer(object):
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            'pybullet_follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start=False)
        self.action_server.start()
        self.execute_now = False
        self.goal = None

    def set_execute_now(self):
        self.execute_now = True

    def execute_cb(self, goal):
        self.goal = goal
        while self.is_data_available():
            rospy.sleep(1)
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        self.action_server.set_succeeded(result)

    def is_data_available(self):
        return True if self.goal and len(self.goal.trajectory.points) > 0 else False

    def get_next_step(self):
        goal = self.goal.trajectory.points.pop(0)
        return goal.positions


class Trajectory:
    def __init__(self, pybullet, robot, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # get robot from parent class
        self.robot = robot
        # lists to recall last received command (useful when controlling multiple joints)
        self.position_joint_commands = []

        # the max force to apply to the joint, used in velocity control
        self.force_commands = []
        # get joints names and store them in dictionary
        self.joint_index_name_dic = kargs['rev_joints']
        # setup subscribers
        self.pc_subscribers = []
        self.joint_indices = []

        self.server = ActionServer()

        # joint position, velocity and effort control command individual subscribers
        for joint_index in self.joint_index_name_dic:
            self.position_joint_commands.append(0.0)
            # used only in velocity control mode
            joint_name = self.joint_index_name_dic[joint_index]
            # create list of joints for later use in pve_ctrl_cmd(...)
            self.joint_indices.append(joint_index)
            # create position control object

    def is_available(self):
        return self.server.is_data_available()

    def execute(self):
        if self.server.is_data_available():
            goal = self.server.get_next_step()
            self.pb.setJointMotorControlArray(bodyUniqueId=self.robot, jointIndices=self.joint_indices,
                                              controlMode=self.pb.POSITION_CONTROL, targetPositions=goal)
