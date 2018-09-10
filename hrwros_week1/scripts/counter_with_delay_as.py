#! /usr/bin/env python

# This code has been adapted from the ROS Wiki actionlib tutorials to the context
# of this course.
# (http://wiki.ros.org/hrwros_msgs/Tutorials)

import rospy

import actionlib

from hrwros_msgs.msg import CounterWithDelayAction, CounterWithDelayFeedback, CounterWithDelayResult

class CounterWithDelayActionClass(object):
    # create messages that are used to publish feedback/result
    _feedback = CounterWithDelayFeedback()
    _result = CounterWithDelayResult()

    def __init__(self, name):
        # This will be the name of the action server which clients can use to connect to.
        self._action_name = name

        # Create a simple action server of the newly defined action type and
        # specify the execute callback where the goal will be processed.
        self._as = actionlib.SimpleActionServer(self._action_name, CounterWithDelayAction, execute_cb=self.execute_cb, auto_start = False)

        # Start the action server.
        self._as.start()
        rospy.loginfo("Action server started...")

    def execute_cb(self, goal):
        # OPTIONAL: NOT GRADED Check if the parameter for the counter delay is available on the parameter server.
        # if not rospy.has_param("<write your code here>"):
        #     rospy.loginfo("Parameter %s not found on the parameter server. Using default value of 1.0s for counter delay.","counter_delay")
        #     counter_delay = 1.0
        # else:
        # Get the parameter for delay between counts.
        counter_delay_value = rospy.get_param("<write your code here>")
        rospy.loginfo("Parameter %s was found on the parameter server. Using %fs for counter delay."%("counter_delay", counter_delay_value))

        # Variable to decide the final state of the action server.
        success = True

        # publish info to the console for the user
        rospy.loginfo('%s action server is counting up to  %i with %fs delay between each count' % (self._action_name, goal.num_counts, counter_delay_value))

        # start executing the action
        for counter_idx in range(0, goal.num_counts):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            # publish the feedback
            self._feedback.counts_elapsed = counter_idx
            self._as.publish_feedback(self._feedback)
            # Wait for counter_delay s before incrementing the counter.
            <write your code here>

        if success:
            self._result.result_message = "Successfully completed counting."
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    # Initialize a ROS node for this action server.
    rospy.init_node('counter_with_delay')

    # Create an instance of the action server here.
    server = CounterWithDelayActionClass(rospy.get_name())
    rospy.spin()
