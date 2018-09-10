#!/usr/bin/env python
# This code has been adapted from the ROS Wiki ROS Service tutorials to the context
# of this course.
# (http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

import sys
import rospy
from hrwros_msgs.srv import ConvertMetresToFeet, ConvertMetresToFeetRequest, ConvertMetresToFeetResponse

def metres_to_feet_client(x):
    # First wait for the service to become available.
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service('metres_to_feet')
    try:
        # Create a service proxy.
        metres_to_feet = rospy.ServiceProxy('metres_to_feet', ConvertMetresToFeet)

        # Call the service here.
        service_response = metres_to_feet(x)

        print("I only got here AFTER the service call was completed!")

        # Return the response to the calling function.
        return service_response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    # Initialize the client ROS node.
    rospy.init_node("metres_to_feet_client", anonymous = False)

    # The distance to be converted to feet.
    dist_metres = 0.25

    rospy.loginfo("Requesting conversion of %4.2f m to feet"%(dist_metres))

    # Call the service client function.
    service_response = metres_to_feet_client(dist_metres)

    # Process the service response and display log messages accordingly.
    if(not service_response.success):
        rospy.logerr("Conversion unsuccessful! Requested distance in metres should be a positive real number.")
    else:
        rospy.loginfo("%4.2f(m) = %4.2f feet"%(dist_metres, service_response.distance_feet))
        rospy.loginfo("Conversion successful!")
