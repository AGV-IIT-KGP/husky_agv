rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS xy_goal_tolerance 0.3
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS yaw_goal_tolerance 0.3 #18 degrees on either side

rosrun dynamic_reconfigure dynparam set /master use_ramp true
