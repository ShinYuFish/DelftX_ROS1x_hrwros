#! /usr/bin/env python
from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the CounterWithDelay action, including the
# goal message and the result message.
from hrwros_msgs.msg import CounterWithDelayAction, CounterWithDelayGoal, CounterWithDelayResult

def counter_with_delay_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (CounterWithDelayAction) to the constructor.
    client = actionlib.SimpleActionClient('counter_with_delay', CounterWithDelayAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for action server to come up...")
    client.wait_for_server()

    num_counts = 10

    # Creates a goal to send to the action server.
    goal = CounterWithDelayGoal(num_counts)

    # Sends the goal to the action server.
    client.send_goal(goal)

    rospy.loginfo("Goal has been sent to the action server.")

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A CounterWithDelayResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('counter_with_delay_ac')
        result = counter_with_delay_client()
        rospy.loginfo(result.result_message)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
