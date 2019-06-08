rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS xy_goal_tolerance 0.5
rosrun dynamic_reconfigure dynparam set /move_base/TebLocalPlannerROS yaw_goal_tolerance 0.7853981634 #45 degrees on either side

rosrun dynamic_reconfigure dynparam set /master use_ramp false
