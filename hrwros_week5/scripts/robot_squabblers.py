#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Delft University of Technology,
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
#  * Neither the name of SRI International nor the names of its
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
import tf2_ros

# String list for robot conversation
_robot1_strings = ['Turtlebot is at: ', 'Yes, I am sure. Turtlebot is at: ', 'Now, Robot2, get on with it. Turtlebot IS AT: ']
_robot2_strings = ['Are you sure? Turtlebot is actually at: ', 'You got to be kidding me!! I can guarantee that the Turtlebot is at: ', 'You hold back right there Robot1! Turtlebot is not where you say it is! Turtlebot is at: ']

# The robot_squabblers() function reporting live about the dispute between robot arms.
def robot_squabblers():
    # Initialize ROS node.
    rospy.init_node('robot_squabblers', anonymous = True)

    # Define TF buffer and associate it to a TF listener.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Helper variables.
    rate_pub = rospy.Rate(0.5)
    rate_listener = rospy.Rate(10)
    string_idx = 0

    # Continuously display the transform between the base links of the two robots and the turtlebot.
    while not rospy.is_shutdown():
        # This try-except block makes sure any errors we might run into due to non-availability of TF frames
        # is correctly handled via exception handling.
        try:
            # Lookup translation information between robot1_base_link and turtlebot base_link.
            trans_r1 = tfBuffer.lookup_transform('robot1_base_link', 'base_link', rospy.Time())
            # Update and print string for Robot1.
            robot1_words = 'Robot1:' + _robot1_strings[string_idx]
            print ('%s (x=%4.2f, y=%4.2f).'%(robot1_words, trans_r1.transform.translation.x, trans_r1.transform.translation.y))
            # Lookup translation information between robot2_base_link and turtlebot base_link.
            trans_r2 = tfBuffer.lookup_transform('robot2_base_link', 'base_link', rospy.Time())
            # Update and print string for Robot1.
            robot2_words = 'Robot2:' + _robot2_strings[string_idx]
            print ('%s (x=%4.2f, y=%4.2f).'%(robot2_words, trans_r2.transform.translation.x, trans_r2.transform.translation.y))
            print '-------------------'
            string_idx +=1
            if(string_idx == 3):
                string_idx = 0
            rate_pub.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # In case the requested TF is not available, try again after a short duration.
            rate_listener.sleep()
            continue

if __name__ == '__main__':
    try:
        # Call the robot_squabblers() function.
        robot_squabblers()
    except rospy.ROSInterruptException:
        pass
