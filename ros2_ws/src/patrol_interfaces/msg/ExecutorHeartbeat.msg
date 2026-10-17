# robot_patrol -> patrol_guard liveness and active-run lease snapshot.
string instance_id
string owner_session_id
string run_id
uint8 state
bool active
bool dispatch_allowed
bool stop_unconfirmed
builtin_interfaces/Time stamp
# Real ROS2 NavigateToPose goal UUID currently owned by this executor.
# Empty while idle, validating, dwelling, or paused with no active child goal.
string nav_goal_id
