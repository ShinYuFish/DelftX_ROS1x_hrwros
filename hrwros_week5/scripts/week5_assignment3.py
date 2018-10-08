#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Delft University of Technology
# TU Delft Robotics Institute.
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
# Author: Mukunda Bharatheesha
##
import rospy
from hrwros_gazebo.msg import LogicalCameraImage
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs

# Declare a TF buffer globally.
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def logical_camera2_callback(data):
  # Check if the logical camera has seen our box which has the name 'object'.
  if (data.models[-1].type == 'object'):
    # Create a pose stamped message type from the camera image topic.
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.header.stamp = rospy.Time.now()
    object_pose.header.frame_id = "logical_camera_2_frame"
    object_pose.pose.position.x = data.models[-1].pose.position.x
    object_pose.pose.position.y = data.models[-1].pose.position.y
    object_pose.pose.position.z = data.models[-1].pose.position.z
    object_pose.pose.orientation.x = data.models[-1].pose.orientation.x
    object_pose.pose.orientation.y = data.models[-1].pose.orientation.y
    object_pose.pose.orientation.z = data.models[-1].pose.orientation.z
    object_pose.pose.orientation.w = data.models[-1].pose.orientation.w
    while True:
      try:
        object_world_pose = tf_buffer.transform(object_pose, "vacuum_gripper2_suction_cup")
        break
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        continue
    rospy.loginfo('Pose of the object in the vacuum_gripper2_suction_cup reference frame is: %s', object_world_pose)
    rospy.loginfo('Pose of the object in the reference framecamera of logical camera2 is: %s', object_pose)
    # Gracefully terminate the ROS node after transforming the pose.
    rospy.signal_shutdown('Successfully transformed pose.')
  else:
    # Do nothing.
    print('')

if __name__== '__main__':
  # Initialize ROS node to transform object pose.
  rospy.init_node('week5_assignment3',
                    anonymous=True)

  # Subscribe to the logical camera topic.
  rospy.Subscriber('hrwros/logical_camera_2', LogicalCameraImage, logical_camera2_callback)

  rospy.spin()
