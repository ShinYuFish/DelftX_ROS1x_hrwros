#! /usr/bin/env python

# Assignment 2 for Week1: In this assignment you will subscribe to the topic that
# publishes information on the box height in metres and use the metres_to_feet
# service to convert this height in metres to height in feet.

import rospy
from hrwros_msgs.msg import BoxHeightInformation
from hrwros_msgs.srv import ConvertMetresToFeet, ConvertMetresToFeetRequest, ConvertMetresToFeetResponse

def box_height_info_callback(data):
    try:
        # Create a proxy for the service to convert metres to feet.
        metres_to_feet = rospy.ServiceProxy(<update the correct details here>)

        # Call the service here.
        service_response = <write your code here>

        # Write a log message here to print the height of this box in feet.
        <write your code here>

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    # Initialize the ROS node here.
    rospy.init_node('box_height_in_feet', anonymous = False)

    # First wait for the service to become available.
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service('<use the correct service name here>')
    rospy.loginfo("Service %s is now available", '<use the correct service name here>')

    # Create a subscriber to the box height topic.
    rospy.Subscriber(<use the correct topic name here>, <use the correct message type here>, <use the correct callback name here>)

    # Prevent this ROS node from terminating until Ctrl+C is pressed.
    rospy.spin()
