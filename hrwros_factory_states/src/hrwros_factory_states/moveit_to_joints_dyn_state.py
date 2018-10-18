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
import sys
import moveit_commander
import copy
import actionlib
import moveit_msgs
import sensor_msgs
from moveit_python import *
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes, RobotState, ExecuteTrajectoryGoal, ExecuteTrajectoryAction
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest

'''

Created on Sep 5 2018

This is an alternative implementation of the original MoveitToJointsDynState state implementation by Alberto Romayon 10.11.2016

@author: HRWROS mooc instructors

'''

class MoveitToJointsDynState(EventState):
	'''
	Uses MoveIt to plan and move the specified joints to the target configuration.

	-- move_group		string		Name of the move group to be used for planning.
									Specified joint names need to exist in the given group.

	-- offset			float		Some offset
	-- tool_link		string		e.g. "vacuum_gripper1_suction_cup"


	-- action_topic 	string 		Topic on which MoveIt is listening for action calls.

	># joint_names		string[]	Names of the joints to set.
									Does not need to specify all joints.
	># joint_values		float[]		Target configuration of the joints.
									Same order as their corresponding names in joint_names.

	<= reached 						Target joint configuration has been reached.
	<= planning_failed 				Failed to find a plan to the given joint configuration.
	<= control_failed 				Failed to move the arm along the planned trajectory.

	'''


	def __init__(self, move_group, offset, tool_link, action_topic = '/move_group'):
		'''
		Constructor
		'''
		super(MoveitToJointsDynState, self).__init__(
			outcomes=['reached', 'planning_failed', 'control_failed'],
			input_keys=['joint_values', 'joint_names'])

		self._offset = offset
		self._tool_link = tool_link

		self._action_topic = action_topic
		self._fk_srv_topic = '/compute_fk'
		self._cartesian_srv_topic = '/compute_cartesian_path'
		self._client = ProxyActionClient({self._action_topic: MoveGroupAction})
		self._fk_srv = ProxyServiceCaller({self._fk_srv_topic: GetPositionFK})
		self._cartesian_srv = ProxyServiceCaller({self._cartesian_srv_topic: GetCartesianPath})

		self._current_group_name = move_group
		self._joint_names = None

		self._planning_failed = False
		self._control_failed = False
		self._success = False

		self._traj_client = actionlib.SimpleActionClient('execute_trajectory',
			moveit_msgs.msg.ExecuteTrajectoryAction)
		self._traj_client.wait_for_server()


	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._planning_failed:
			return 'planning_failed'
		if self._control_failed:
			return 'control_failed'
		if self._success:
			return 'reached'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)

			if result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
				Logger.logwarn('Control failed for move action of group: %s (error code: %s)' % (self._current_group_name, str(result.error_code)))
				self._control_failed = True
				return 'control_failed'
			elif result.error_code.val != MoveItErrorCodes.SUCCESS:
				Logger.logwarn('Move action failed with result error code: %s' % str(result.error_code))
				self._planning_failed = True
				return 'planning_failed'
			else:
				self._success = True
				return 'reached'


	def on_enter(self, userdata):
		self._planning_failed = False
		self._control_failed = False
		self._success = False

		self._joint_names = userdata.joint_names

		self._initial_state = RobotState()
		self._initial_state.joint_state.position = copy.deepcopy(userdata.joint_values)
		self._initial_state.joint_state.name = copy.deepcopy(self._joint_names)
		#print self._initial_state.joint_state.name
		#print self._initial_state.joint_state.position

		self._srv_req = GetPositionFKRequest()
		self._srv_req.robot_state = self._initial_state
		self._srv_req.header.stamp = rospy.Time.now()
		self._srv_req.header.frame_id = "world"
		self._srv_req.fk_link_names = [self._tool_link]

		try:
			srv_result = self._fk_srv.call(self._fk_srv_topic, self._srv_req)
			self._failed = False

		except Exception as e:
			Logger.logwarn('Could not call FK: ' + str(e))
			self._planning_failed = True
			return


		grasp_pose = srv_result.pose_stamped[0].pose
		grasp_pose_stamped = srv_result.pose_stamped[0]

		# Create a pre-grasp approach pose with an offset of 0.3
		pre_grasp_approach_pose = copy.deepcopy(grasp_pose_stamped)
		pre_grasp_approach_pose.pose.position.z += self._offset + 0.3

		# Create an object to MoveGroupInterface for the current robot.
		self._mgi_active_robot = MoveGroupInterface(self._current_group_name,
			self._current_group_name + '_base_link')	# TODO: clean up in on_exit

		self._mgi_active_robot.moveToPose(pre_grasp_approach_pose, self._tool_link)

		# Use cartesian motions to pick the object.
		cartesian_service_req = GetCartesianPathRequest()
		cartesian_service_req.start_state.joint_state = rospy.wait_for_message(self._current_group_name + '/joint_states', sensor_msgs.msg.JointState)
		cartesian_service_req.header.stamp = rospy.Time.now()
		cartesian_service_req.header.frame_id = "world"
		cartesian_service_req.link_name = self._tool_link
		cartesian_service_req.group_name = self._current_group_name
		cartesian_service_req.max_step = 0.01
		cartesian_service_req.jump_threshold = 0
		cartesian_service_req.avoid_collisions = True
		grasp_pose.position.z += self._offset + 0.16  # this is basically the toolframe (with the box as the tool)
		cartesian_service_req.waypoints.append(grasp_pose)

		try:
			cartesian_srv_result = self._cartesian_srv.call(self._cartesian_srv_topic, cartesian_service_req)
			self._failed = False

		except Exception as e:
			Logger.logwarn('Could not call Cartesian: ' + str(e))
			self._planning_failed = True
			return

		if cartesian_srv_result.fraction < 1.0:
			Logger.logwarn('Cartesian failed. fraction: ' + str(cartesian_srv_result.fraction))
			self._planning_failed = True
			return

		traj_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
		traj_goal.trajectory = cartesian_srv_result.solution

		self._traj_client.send_goal(traj_goal)
		self._traj_client.wait_for_result()


	def on_exit(self, userdata):
		pass


	def on_stop(self):
		try:
			if self._client.is_available(self._action_topic) \
			and not self._client.has_result(self._action_topic):
				self._client.cancel(self._action_topic)
		except:
			# client already closed
			pass

	def on_pause(self):
		self._client.cancel(self._action_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
