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

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from hrwros_gazebo.srv import ConveyorBeltControl, ConveyorBeltControlRequest
from hrwros_gazebo.msg import ConveyorBeltState

'''

Created on Sep 5 2018

@author: HRWROS mooc instructors

'''

class SetConveyorPowerState(EventState):
	'''
	State to update the speed of the conveyor belt through a service call (0 to stop it, 100 max), in the factory simulation of the MOOC "Hello (Real) World with ROS"

	-- stop 		bool 	If 'true' the state instance stops the conveyor belt, ignoring the speed inputkey

	># speed		float	Value to set the speed of the conveyor belt.

	<= succeeded 			Speed of the conveyor belt has been succesfully set.
	<= failed 				There was a problem setting the speed.

	'''

	def __init__(self, stop):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(SetConveyorPowerState, self).__init__(outcomes = ['succeeded', 'failed'], input_keys = ['speed'])

		# Store state parameter for later use.
		self._stop = stop

		# initialize service proxy
		self._srv_name = '/hrwros/conveyor/control'
		self._srv = ProxyServiceCaller({self._srv_name: ConveyorBeltControl})


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		if self._failed:
			return 'failed'

		if self._srv_result.success is True:
			return 'succeeded'
		else:
			return 'failed'


	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.
		self.speed = float(userdata.speed)

		# create service request depending on activation parameter and userdata
		self._srv_req = ConveyorBeltControlRequest()
		if self._stop:
			self._srv_req.state.power = 0.0
		else:
			self._srv_req.state.power = self.speed
		try:
			self._srv_result = self._srv.call(self._srv_name, self._srv_req)
			self._failed = False

		except Exception as e:
			Logger.logwarn('Could not update conveyor belt speed')
			rospy.logwarn(str(e))
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
