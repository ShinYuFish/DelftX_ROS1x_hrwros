std_msgs/Header header
geometry_msgs/PoseStamped pick_approach  # Robot tool pose right before picking
geometry_msgs/PoseStamped pick_pose      # Robot tool makes contact with the object and closes its gripper
geometry_msgs/PoseStamped pick_retreat   # Robot tool moves the object away from its resting surface
geometry_msgs/PoseStamped place_pose     # Robot tool moves object to this location and opens gripper
