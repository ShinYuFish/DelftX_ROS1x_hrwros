#!/usr/bin/env python
# This code has been adapted from the ROS Wiki ROS Service tutorials to the context
# of this course.
# (http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)

from hrwros_msgs.srv import ConvertMetresToFeet, ConvertMetresToFeetRequest, ConvertMetresToFeetResponse
import rospy
import numpy as np

_CONVERSION_FACTOR_METRES_TO_FEET = 3.28 # Metres -> Feet conversion factor.

# Service callback function.
def process_service_request(req):

    # Instantiate the response message object.
    res = ConvertMetresToFeetResponse()

    # Perform sanity check. Allow only positive real numbers.
    # Compose the response message accordingly.
    if(req.distance_metres < 0):
        res.success = False
        res.distance_feet = -np.Inf # Default error value.
    else:
        res.distance_feet = _CONVERSION_FACTOR_METRES_TO_FEET * req.distance_metres
        res.success = True

    #Return the response message.
    return res

def metres_to_feet_server():
    # ROS node for the service server.
    rospy.init_node('metres_to_feet_server', anonymous = False)

    # Create a ROS service type.
    service = rospy.Service('metres_to_feet', ConvertMetresToFeet, process_service_request)

    # Log message about service availability.
    rospy.loginfo('Convert metres to feet service is now available.')
    rospy.spin()

if __name__ == "__main__":
    metres_to_feet_server()
