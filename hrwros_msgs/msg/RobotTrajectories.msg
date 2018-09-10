std_msgs/Header header
moveit_msgs/RobotTrajectory cur_to_approach       # Array of trajectories to be executed by the robot from current pose to approach pose
moveit_msgs/RobotTrajectory approach_to_pick  # Array of trajectories to be executed by the robot from approach pose to pick pose
moveit_msgs/RobotTrajectory pick_to_retreat   # Array of trajectories to be executed by the robot from pick pose to retreat pose
moveit_msgs/RobotTrajectory retreat_to_place  # Array of trajectories to be executed by the robot from retreat pose to place pose
duration[] trajectory_delays                  # Time delays before each trajectory.  Use 0 for no delay
duration[] execution_duration                 # Execution duration for each state of robot trajectories
time pick_deadline                            # The deadline to reach pick pose
