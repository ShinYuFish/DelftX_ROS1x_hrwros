#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Delft University of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Delft University of Technology nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Authors: the HRWROS mooc instructors

import rospy
import sensor_msgs

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

import math
import sys
import copy
import moveit_commander
import moveit_msgs.msg

from tf.transformations import *

import geometry_msgs
import tf2_ros
import tf2_geometry_msgs
import actionlib

'''
Created on Sep 5 2018

@author: HRWROS mooc instructors

This state provides the joint configuration to grasp the box in the factory simulation of the MOOC "Hello (Real) World with ROS", given the pose of the box as provided by the DetectPartCameraState
'''

class ComputeGraspState(EventState):
	'''
	Computes the joint configuration needed to grasp the part given its pose.

	-- group       string              Name of the group for which to compute the joint values for grasping.
	-- offset			float		Some offset
	-- joint_names		string[]		names of the joints
	-- tool_link		string			e.g. "vacuum_gripper1_suction_cup"

	># pose				PoseStamped		pose of the part to pick
	#> joint_values		float[]			joint values for grasping
	#> joint_names		string[]		names of the joints

	<= continue 			if a grasp configuration has been computed for the pose
	<= failed 				otherwise.
	'''

	def __init__(self, group, offset, joint_names, tool_link):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(ComputeGraspState, self).__init__(outcomes = ['continue', 'failed'], input_keys = ['pose'], output_keys = ['joint_values','joint_names'])

		self._group = group
		self._offset = offset
		self._joint_names = joint_names
		self._tool_link = tool_link

		self._srv_name = '/compute_ik'
		self._ik_srv = ProxyServiceCaller({self._srv_name: GetPositionIK})

		# tf to transfor the object pose
		self._tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

		self._robot1_client = actionlib.SimpleActionClient('execute_trajectory',
			moveit_msgs.msg.ExecuteTrajectoryAction)
		self._robot1_client.wait_for_server()
		rospy.loginfo('Execute Trajectory server is available for robot1')

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		rospy.logwarn(userdata.pose)

		if self._failed == True:
			return 'failed'

		if int(self._srv_result.error_code.val) == 1:
			sol_js = self._srv_result.solution.joint_state

			joint_names = self._joint_names
			#['robot1_shoulder_pan_joint', 'robot1_shoulder_lift_joint', 'robot1_elbow_joint', 'robot1_wrist_1_joint', 'robot1_wrist_2_joint', 'robot1_wrist_3_joint']  # TODO: this needs to be a parameter

			jname_idx = [sol_js.name.index(jname) for jname in joint_names]
			j_angles = map(sol_js.position.__getitem__, jname_idx)
			# solution_dict = dict(zip(joint_names, j_angles))
			userdata.joint_values = copy.deepcopy(j_angles)
			userdata.joint_names = copy.deepcopy(joint_names)
			return 'continue'
		else:
			return 'failed'

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# Get transform between camera and robot1_base
		while True:
			rospy.sleep(0.1)
			try:
				target_pose = self._tf_buffer.transform(userdata.pose, "world")
				break
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				continue

		# the grasp pose is defined as being located on top of the box
		target_pose.pose.position.z += self._offset + 0.0


		# rotate the object pose 180 degrees around
		q_orig = [target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w]
		q_rot = quaternion_from_euler(math.pi, 0, 0)
		res_q = quaternion_multiply(q_rot, q_orig)
		target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*res_q)

		# use ik service to compute joint_values
		self._srv_req = GetPositionIKRequest()
		self._srv_req.ik_request.group_name = self._group
		self._srv_req.ik_request.robot_state.joint_state = rospy.wait_for_message(self._group + '/joint_states', sensor_msgs.msg.JointState)

		self._srv_req.ik_request.ik_link_name = self._tool_link  # TODO: this needs to be a parameter
		self._srv_req.ik_request.pose_stamped = target_pose
		self._srv_req.ik_request.avoid_collisions = True
		self._srv_req.ik_request.attempts = 5

		try:
			self._srv_result = self._ik_srv.call(self._srv_name, self._srv_req)
			self._failed = False

		except Exception as e:
			Logger.logwarn('Could not call IK: ' + str(e))
			self._failed = True

	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.
		pass # Nothing to do

	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.
		pass

	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.
		pass # Nothing to do
