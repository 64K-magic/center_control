std_msgs/Header header

# Whether Nav2 action servers are currently reachable in the graph
bool nav2_available

# Navigation state: unavailable, idle, accepted, executing, succeeded, canceled, aborted, canceling
string navigation_state

# Current navigation task (primary action)
# task_source: none, follow_waypoints, navigate_to_pose, navigate_through_poses
string task_source
# task_status: unknown, accepted, executing, canceling, succeeded, canceled, aborted
string task_status

# Inner navigation task while follow_waypoints is active (empty when not applicable)
string subtask_source
string subtask_status

# Waypoint progress (-1 if not applicable)
int32 current_waypoint

# Recovery state
bool in_recovery
uint16 recovery_count
string failure_category
string failure_detail
string failed_bt_node
# 激光障碍与 FollowPath 失败独立标记（可同时为 true；failure_detail 仍为最近一次主失败）
bool obstacle_ahead_active
bool follow_path_failed_active
string[] active_behaviors
