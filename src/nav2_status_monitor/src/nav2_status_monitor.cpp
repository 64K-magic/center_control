#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav2_status_monitor/msg/nav2_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav2_status_monitor
{

using NavigateToPoseFeedback = nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage;
using NavigateThroughPosesFeedback = nav2_msgs::action::NavigateThroughPoses::Impl::FeedbackMessage;
using FollowWaypointsFeedback = nav2_msgs::action::FollowWaypoints::Impl::FeedbackMessage;
using Nav2StatusMsg = nav2_status_monitor::msg::Nav2Status;

class Nav2StatusMonitor : public rclcpp::Node
{
public:
  Nav2StatusMonitor()
  : Node("nav2_status_monitor")
  {
    declare_parameter<std::string>("status_topic", "/nav2/status");
    declare_parameter<double>("publish_period_sec", 2.0);
    declare_parameter<std::string>(
      "navigate_to_pose_status_topic", "/navigate_to_pose/_action/status");
    declare_parameter<std::string>(
      "navigate_to_pose_feedback_topic", "/navigate_to_pose/_action/feedback");
    declare_parameter<std::string>(
      "navigate_through_poses_status_topic", "/navigate_through_poses/_action/status");
    declare_parameter<std::string>(
      "navigate_through_poses_feedback_topic", "/navigate_through_poses/_action/feedback");
    declare_parameter<std::string>(
      "follow_waypoints_status_topic", "/follow_waypoints/_action/status");
    declare_parameter<std::string>(
      "follow_waypoints_feedback_topic", "/follow_waypoints/_action/feedback");
    declare_parameter<std::string>("behavior_tree_log_topic", "/behavior_tree_log");
    declare_parameter<std::string>(
      "nav2_availability_topic", "/navigate_to_pose/_action/status");
    declare_parameter<bool>("early_obstacle_detection", true);
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<double>("obstacle_ahead_distance", 2.5);
    declare_parameter<double>("obstacle_ahead_clear_distance", 3.5);
    declare_parameter<double>("obstacle_ahead_half_angle_deg", 35.0);
    declare_parameter<int>("obstacle_ahead_confirm_scans", 40);
    declare_parameter<int>("obstacle_ahead_clear_confirm_scans", 2);
    declare_parameter<int>("obstacle_ahead_min_points", 8);
    declare_parameter<std::string>("obstacle_scan_topic", "/obstacle/scan");

    const auto status_topic = get_parameter("status_topic").as_string();
    publish_period_sec_ = get_parameter("publish_period_sec").as_double();
    nav2_availability_topic_ = get_parameter("nav2_availability_topic").as_string();
    early_obstacle_detection_ = get_parameter("early_obstacle_detection").as_bool();
    obstacle_ahead_distance_ = get_parameter("obstacle_ahead_distance").as_double();
    obstacle_ahead_clear_distance_ = get_parameter("obstacle_ahead_clear_distance").as_double();
    obstacle_ahead_half_angle_rad_ =
      get_parameter("obstacle_ahead_half_angle_deg").as_double() * M_PI / 180.0;
    obstacle_ahead_confirm_scans_ =
      std::max(1, static_cast<int>(get_parameter("obstacle_ahead_confirm_scans").as_int()));
    obstacle_ahead_clear_confirm_scans_ =
      std::max(1, static_cast<int>(get_parameter("obstacle_ahead_clear_confirm_scans").as_int()));
    obstacle_ahead_min_points_ =
      std::max(1, static_cast<int>(get_parameter("obstacle_ahead_min_points").as_int()));

    status_pub_ = create_publisher<Nav2StatusMsg>(status_topic, 10);

    // Action status topics use transient_local durability; volatile subscribers miss updates.
    const auto action_status_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    const auto feedback_qos = rclcpp::QoS(10);

    navigate_to_pose_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      get_parameter("navigate_to_pose_status_topic").as_string(),
      action_status_qos,
      std::bind(&Nav2StatusMonitor::navigateToPoseStatusCb, this, std::placeholders::_1));

    navigate_to_pose_feedback_sub_ = create_subscription<NavigateToPoseFeedback>(
      get_parameter("navigate_to_pose_feedback_topic").as_string(),
      feedback_qos,
      std::bind(&Nav2StatusMonitor::navigateToPoseFeedbackCb, this, std::placeholders::_1));

    navigate_through_poses_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      get_parameter("navigate_through_poses_status_topic").as_string(),
      action_status_qos,
      std::bind(
        &Nav2StatusMonitor::navigateThroughPosesStatusCb, this, std::placeholders::_1));

    navigate_through_poses_feedback_sub_ = create_subscription<NavigateThroughPosesFeedback>(
      get_parameter("navigate_through_poses_feedback_topic").as_string(),
      feedback_qos,
      std::bind(
        &Nav2StatusMonitor::navigateThroughPosesFeedbackCb, this, std::placeholders::_1));

    follow_waypoints_status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      get_parameter("follow_waypoints_status_topic").as_string(),
      action_status_qos,
      std::bind(&Nav2StatusMonitor::followWaypointsStatusCb, this, std::placeholders::_1));

    follow_waypoints_feedback_sub_ = create_subscription<FollowWaypointsFeedback>(
      get_parameter("follow_waypoints_feedback_topic").as_string(),
      feedback_qos,
      std::bind(&Nav2StatusMonitor::followWaypointsFeedbackCb, this, std::placeholders::_1));

    behavior_tree_log_sub_ = create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      get_parameter("behavior_tree_log_topic").as_string(),
      10,
      std::bind(&Nav2StatusMonitor::behaviorTreeLogCb, this, std::placeholders::_1));

    const std::vector<std::pair<std::string, std::string>> behavior_actions = {
      {"spin", "/spin/_action/status"},
      {"backup", "/backup/_action/status"},
      {"wait", "/wait/_action/status"},
      {"assisted_teleop", "/assisted_teleop/_action/status"},
    };

    for (const auto & entry : behavior_actions) {
      const auto & behavior = entry.first;
      const auto & topic = entry.second;
      behavior_status_subs_.push_back(create_subscription<action_msgs::msg::GoalStatusArray>(
        topic,
        action_status_qos,
        [this, behavior](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
          behaviorStatusCb(msg, behavior);
        }));
    }

    if (early_obstacle_detection_) {
      const auto obstacle_scan_topic = get_parameter("obstacle_scan_topic").as_string();
      obstacle_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
        obstacle_scan_topic, rclcpp::SensorDataQoS());
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        get_parameter("scan_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&Nav2StatusMonitor::scanCb, this, std::placeholders::_1));
    }

    if (publish_period_sec_ > 0.0) {
      summary_timer_ = create_wall_timer(
        std::chrono::duration<double>(publish_period_sec_),
        std::bind(&Nav2StatusMonitor::publishSummary, this));
    }

    nav2_available_ = queryNav2Available();
    if (!nav2_available_) {
      resetToUnavailableState();
    }

    RCLCPP_INFO(get_logger(), "Nav2 status monitor started, publishing to %s", status_topic.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Subscribed action status topics: %s, %s, %s",
      get_parameter("navigate_to_pose_status_topic").as_string().c_str(),
      get_parameter("navigate_through_poses_status_topic").as_string().c_str(),
      get_parameter("follow_waypoints_status_topic").as_string().c_str());
    if (early_obstacle_detection_) {
      RCLCPP_INFO(
        get_logger(),
        "Early obstacle detection enabled on %s (%.1fm ahead, %.1fm clear, %.1f deg half-angle, confirm=%d clear_confirm=%d min_points=%d)",
        get_parameter("scan_topic").as_string().c_str(),
        obstacle_ahead_distance_,
        obstacle_ahead_clear_distance_,
        obstacle_ahead_half_angle_rad_ * 180.0 / M_PI,
        obstacle_ahead_confirm_scans_,
        obstacle_ahead_clear_confirm_scans_,
        obstacle_ahead_min_points_);
      RCLCPP_INFO(
        get_logger(),
        "Obstacle scan publisher: %s",
        get_parameter("obstacle_scan_topic").as_string().c_str());
    }

    publishStatus(true);
  }

private:
  struct BtFailureRecord
  {
    std::string node_name;
  };

  static const std::vector<std::string> kTaskSources;
  static const std::vector<std::string> kSubtaskSources;
  static const std::unordered_set<std::string> kPlanningNodes;
  static const std::unordered_set<std::string> kControlNodes;
  static const std::unordered_set<std::string> kPlanningRecoveryNodes;
  static const std::unordered_set<std::string> kControlRecoveryNodes;

  static std::string goalStatusToString(int8_t status)
  {
    using action_msgs::msg::GoalStatus;
    switch (status) {
      case GoalStatus::STATUS_UNKNOWN:
        return "unknown";
      case GoalStatus::STATUS_ACCEPTED:
        return "accepted";
      case GoalStatus::STATUS_EXECUTING:
        return "executing";
      case GoalStatus::STATUS_CANCELING:
        return "canceling";
      case GoalStatus::STATUS_SUCCEEDED:
        return "succeeded";
      case GoalStatus::STATUS_CANCELED:
        return "canceled";
      case GoalStatus::STATUS_ABORTED:
        return "aborted";
      default:
        return "invalid";
    }
  }

  static bool isActiveTaskStatus(int8_t status)
  {
    using action_msgs::msg::GoalStatus;
    return status == GoalStatus::STATUS_ACCEPTED ||
           status == GoalStatus::STATUS_EXECUTING ||
           status == GoalStatus::STATUS_CANCELING;
  }

  static bool isTerminalTaskStatus(int8_t status)
  {
    using action_msgs::msg::GoalStatus;
    return status == GoalStatus::STATUS_SUCCEEDED ||
           status == GoalStatus::STATUS_CANCELED ||
           status == GoalStatus::STATUS_ABORTED;
  }

  int8_t taskStatus(const std::string & source) const
  {
    const auto it = task_states_.find(source);
    if (it == task_states_.end()) {
      return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
    }
    return it->second;
  }

  bool isFollowWaypointsActive() const
  {
    return isActiveTaskStatus(taskStatus("follow_waypoints"));
  }

  bool isNavigationActive() const
  {
    for (const auto & source : kTaskSources) {
      if (isActiveTaskStatus(taskStatus(source))) {
        return true;
      }
    }
    return false;
  }

  void setObstacleAheadFailure()
  {
    if (!isNavigationActive()) {
      return;
    }

    const bool was_active = obstacle_ahead_active_;
    obstacle_ahead_active_ = true;
    const bool failure_changed = !follow_path_failed_active_ && (
      last_failure_category_ != "control" ||
      last_failure_detail_ != "obstacle_ahead" ||
      last_failed_bt_node_ != "LaserScan");
    if (!follow_path_failed_active_) {
      last_failure_category_ = "control";
      last_failure_detail_ = "obstacle_ahead";
      last_failed_bt_node_ = "LaserScan";
    }
    if (!was_active || failure_changed) {
      publishStatus();
    }
  }

  void clearObstacleAheadOnly()
  {
    if (!obstacle_ahead_active_) {
      return;
    }

    obstacle_ahead_active_ = false;
    blocked_scan_count_ = 0;
    clear_scan_count_ = 0;
    if (follow_path_failed_active_ || in_recovery_) {
      publishStatus();
      return;
    }

    if (last_failure_detail_ == "obstacle_ahead") {
      resetFailureTracking();
      publishStatus();
    }
  }

  void clearObstacleAheadIfPathOpen(double min_forward_range)
  {
    if (!obstacle_ahead_active_) {
      return;
    }
    if (min_forward_range <= obstacle_ahead_clear_distance_) {
      return;
    }
    clearObstacleAheadOnly();
  }

  struct ForwardSectorStats
  {
    double min_range{std::numeric_limits<double>::infinity()};
    int close_points{0};
    int near_points{0};
  };

  ForwardSectorStats analyzeForwardSector(const sensor_msgs::msg::LaserScan & msg) const
  {
    ForwardSectorStats stats;
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      const double angle = msg.angle_min + static_cast<double>(i) * msg.angle_increment;
      if (std::abs(angle) > obstacle_ahead_half_angle_rad_) {
        continue;
      }

      const double range = static_cast<double>(msg.ranges[i]);
      if (!std::isfinite(range) || range < msg.range_min || range > msg.range_max) {
        continue;
      }

      stats.min_range = std::min(stats.min_range, range);
      if (range <= obstacle_ahead_distance_) {
        stats.close_points++;
      }
      if (range <= obstacle_ahead_clear_distance_) {
        stats.near_points++;
      }
    }
    return stats;
  }

  bool hasSolidForwardObstacle(const ForwardSectorStats & stats) const
  {
    return stats.close_points >= obstacle_ahead_min_points_ &&
           stats.min_range <= obstacle_ahead_distance_;
  }

  bool inForwardObstacleZone(const ForwardSectorStats & stats) const
  {
    return stats.near_points >= obstacle_ahead_min_points_ &&
           stats.min_range <= obstacle_ahead_clear_distance_;
  }

  sensor_msgs::msg::LaserScan buildObstacleScan(
    const sensor_msgs::msg::LaserScan & src,
    const ForwardSectorStats & stats) const
  {
    sensor_msgs::msg::LaserScan out = src;
    const bool publish_hits = hasSolidForwardObstacle(stats) || inForwardObstacleZone(stats);
    for (size_t i = 0; i < out.ranges.size(); ++i) {
      out.ranges[i] = std::numeric_limits<float>::infinity();
      if (!publish_hits) {
        continue;
      }
      const double angle = out.angle_min + static_cast<double>(i) * out.angle_increment;
      const double range = static_cast<double>(src.ranges[i]);
      const bool in_sector = std::abs(angle) <= obstacle_ahead_half_angle_rad_;
      const bool valid = std::isfinite(range) &&
        range >= static_cast<double>(out.range_min) &&
        range <= static_cast<double>(out.range_max);
      const bool in_distance = valid && range <= obstacle_ahead_clear_distance_;
      if (in_sector && in_distance) {
        out.ranges[i] = src.ranges[i];
      }
    }
    return out;
  }

  void publishObstacleScan(
    const sensor_msgs::msg::LaserScan & src,
    const ForwardSectorStats & stats)
  {
    if (!obstacle_scan_pub_) {
      return;
    }
    auto out = buildObstacleScan(src, stats);
    out.header.stamp = get_clock()->now();
    obstacle_scan_pub_->publish(out);
  }

  void handleFollowPathSuccess()
  {
    if (!follow_path_failed_active_) {
      return;
    }

    follow_path_failed_active_ = false;
    if (obstacle_ahead_active_) {
      last_failure_category_ = "control";
      last_failure_detail_ = "obstacle_ahead";
      last_failed_bt_node_ = "LaserScan";
    } else if (last_failure_detail_ == "follow_path_failed") {
      if (in_recovery_) {
        last_failure_category_ = "control";
        last_failure_detail_ = "control_recovery";
        last_failed_bt_node_ = "FollowPath";
      } else {
        last_failure_category_ = "unknown";
        last_failure_detail_ = "unknown";
        last_failed_bt_node_.clear();
      }
    }
    publishStatus();
  }

  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!ensureNav2AvailableForUpdate()) {
      return;
    }

    if (!isNavigationActive()) {
      if (obstacle_ahead_active_) {
        blocked_scan_count_ = 0;
        clear_scan_count_ = 0;
        obstacle_ahead_active_ = false;
        if (!follow_path_failed_active_ && !in_recovery_) {
          if (last_failure_detail_ == "obstacle_ahead") {
            resetFailureTracking();
            publishStatus();
          }
        } else {
          publishStatus();
        }
      }
      return;
    }

    const ForwardSectorStats sector = analyzeForwardSector(*msg);
    if (!std::isfinite(sector.min_range)) {
      return;
    }

    if (hasSolidForwardObstacle(sector)) {
      blocked_scan_count_++;
      clear_scan_count_ = 0;
      if (blocked_scan_count_ >= obstacle_ahead_confirm_scans_) {
        setObstacleAheadFailure();
      }
      if (obstacle_ahead_active_) {
        publishObstacleScan(*msg, sector);
      }
      return;
    }

    // 滞回区：点数仍足够才维持；孤立残点（绕过障碍后的噪点/反射）不计入
    if (obstacle_ahead_active_ && inForwardObstacleZone(sector)) {
      clear_scan_count_ = 0;
      publishObstacleScan(*msg, sector);
      return;
    }

    blocked_scan_count_ = 0;
    clear_scan_count_++;
    if (clear_scan_count_ >= obstacle_ahead_clear_confirm_scans_) {
      clearObstacleAheadIfPathOpen(sector.min_range);
    }
  }

  Nav2StatusMsg makeStatusMsg() const
  {
    Nav2StatusMsg msg;
    msg.nav2_available = nav2_available_;
    msg.navigation_state = navigation_state_;
    msg.task_source = task_source_;
    msg.task_status = task_status_;
    msg.subtask_source = subtask_source_;
    msg.subtask_status = subtask_status_;
    msg.current_waypoint = current_waypoint_;
    msg.in_recovery = in_recovery_;
    msg.recovery_count = last_recovery_count_;
    msg.failure_category = last_failure_category_;
    msg.failure_detail = last_failure_detail_;
    msg.failed_bt_node = last_failed_bt_node_;
    msg.obstacle_ahead_active = obstacle_ahead_active_;
    msg.follow_path_failed_active = follow_path_failed_active_;
    msg.active_behaviors.assign(active_behaviors_.begin(), active_behaviors_.end());
    return msg;
  }

  bool statusChanged(const Nav2StatusMsg & msg) const
  {
    return msg.nav2_available != last_published_.nav2_available ||
           msg.navigation_state != last_published_.navigation_state ||
           msg.task_source != last_published_.task_source ||
           msg.task_status != last_published_.task_status ||
           msg.subtask_source != last_published_.subtask_source ||
           msg.subtask_status != last_published_.subtask_status ||
           msg.current_waypoint != last_published_.current_waypoint ||
           msg.in_recovery != last_published_.in_recovery ||
           msg.recovery_count != last_published_.recovery_count ||
           msg.failure_category != last_published_.failure_category ||
           msg.failure_detail != last_published_.failure_detail ||
           msg.failed_bt_node != last_published_.failed_bt_node ||
           msg.obstacle_ahead_active != last_published_.obstacle_ahead_active ||
           msg.follow_path_failed_active != last_published_.follow_path_failed_active ||
           msg.active_behaviors != last_published_.active_behaviors;
  }

  void publishStatus(bool force = false)
  {
    auto msg = makeStatusMsg();
    if (!force && !statusChanged(msg)) {
      return;
    }

    msg.header.stamp = get_clock()->now();
    status_pub_->publish(msg);
    last_published_ = msg;

    if (force) {
      RCLCPP_DEBUG(
        get_logger(),
        "Nav2 status heartbeat: state=%s, task=%s/%s",
        msg.navigation_state.c_str(),
        msg.task_source.c_str(),
        msg.task_status.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Nav2 status: state=%s, task=%s/%s, subtask=%s/%s, waypoint=%d, recovery=%u, category=%s",
        msg.navigation_state.c_str(),
        msg.task_source.c_str(),
        msg.task_status.c_str(),
        msg.subtask_source.c_str(),
        msg.subtask_status.c_str(),
        msg.current_waypoint,
        msg.recovery_count,
        msg.failure_category.c_str());
    }
  }

  void publishSummary()
  {
    updateNav2Availability();
    clearStaleTerminalStates();
    recomputeTaskView();
    publishStatus(true);
  }

  void clearStaleTerminalStates()
  {
    bool has_active = false;
    for (const auto & source : kTaskSources) {
      if (isActiveTaskStatus(taskStatus(source))) {
        has_active = true;
        break;
      }
    }
    if (has_active) {
      return;
    }

    for (auto it = task_states_.begin(); it != task_states_.end(); ) {
      if (isTerminalTaskStatus(it->second)) {
        it = task_states_.erase(it);
      } else {
        ++it;
      }
    }
  }

  bool queryNav2Available() const
  {
    return count_publishers(nav2_availability_topic_) > 0;
  }

  void resetToIdleState()
  {
    task_states_.clear();
    navigation_state_ = "idle";
    task_source_ = "none";
    task_status_ = "unknown";
    subtask_source_.clear();
    subtask_status_ = "unknown";
    current_waypoint_ = -1;
    last_recovery_count_ = 0;
    in_recovery_ = false;
    active_behaviors_.clear();
    resetFailureTracking();
  }

  void resetToUnavailableState()
  {
    resetToIdleState();
    navigation_state_ = "unavailable";
    nav2_available_ = false;
  }

  void updateNav2Availability()
  {
    const bool available = queryNav2Available();
    if (available == nav2_available_) {
      return;
    }

    if (!available) {
      RCLCPP_WARN(get_logger(), "Nav2 is no longer available, clearing stale navigation state");
      resetToUnavailableState();
      return;
    }

    RCLCPP_INFO(get_logger(), "Nav2 is available again, waiting for navigation updates");
    nav2_available_ = true;
    resetToIdleState();
    publishStatus();
  }

  bool ensureNav2AvailableForUpdate()
  {
    if (nav2_available_) {
      return true;
    }

    if (!queryNav2Available()) {
      return false;
    }

    nav2_available_ = true;
    resetToIdleState();
    return true;
  }

  void recomputeTaskView()
  {
    std::string primary_source = "none";
    int8_t primary_status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;

    for (const auto & source : kTaskSources) {
      const auto status = taskStatus(source);
      if (isActiveTaskStatus(status)) {
        primary_source = source;
        primary_status = status;
        break;
      }
    }

    if (primary_source == "none") {
      for (const auto & source : kTaskSources) {
        const auto status = taskStatus(source);
        if (isTerminalTaskStatus(status)) {
          primary_source = source;
          primary_status = status;
          break;
        }
      }
    }

    if (primary_source == "none") {
      navigation_state_ = "idle";
      task_source_ = "none";
      task_status_ = "unknown";
      subtask_source_.clear();
      subtask_status_ = "unknown";
      return;
    }

    task_source_ = primary_source;
    task_status_ = goalStatusToString(primary_status);

    subtask_source_.clear();
    subtask_status_ = "unknown";
    if (primary_source == "follow_waypoints") {
      for (const auto & source : kSubtaskSources) {
        const auto status = taskStatus(source);
        if (isActiveTaskStatus(status)) {
          subtask_source_ = source;
          subtask_status_ = goalStatusToString(status);
          break;
        }
      }
    }

    navigation_state_ = task_status_;
  }

  void updateTaskStatusFromArray(
    const action_msgs::msg::GoalStatusArray & msg,
    const std::string & source)
  {
    if (!ensureNav2AvailableForUpdate()) {
      return;
    }

    if (msg.status_list.empty()) {
      if (!task_states_.empty()) {
        task_states_.clear();
        recomputeTaskView();
        publishStatus();
      }
      return;
    }

    int8_t latest_status = action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
    for (const auto & status : msg.status_list) {
      latest_status = status.status;
    }

    if (task_states_[source] == latest_status) {
      return;
    }

    if (isActiveTaskStatus(latest_status)) {
      for (const auto & other : kTaskSources) {
        if (other != source && isTerminalTaskStatus(taskStatus(other))) {
          task_states_.erase(other);
        }
      }
      if (source == "follow_waypoints") {
        task_states_.erase("navigate_to_pose");
        task_states_.erase("navigate_through_poses");
      }
    }

    task_states_[source] = latest_status;
    handleTaskStatusTransition(source, latest_status);
    recomputeTaskView();
    publishStatus();
  }

  void handleTaskStatusTransition(const std::string & source, int8_t status)
  {
    using action_msgs::msg::GoalStatus;

    if (source == "follow_waypoints" && isTerminalTaskStatus(status)) {
      task_states_.erase("navigate_to_pose");
      task_states_.erase("navigate_through_poses");
      subtask_source_.clear();
      subtask_status_ = "unknown";
      current_waypoint_ = -1;
      last_recovery_count_ = 0;
      in_recovery_ = false;
      active_behaviors_.clear();
      return;
    }

    if (source != "navigate_to_pose" && source != "navigate_through_poses") {
      return;
    }

    if (status == GoalStatus::STATUS_ACCEPTED) {
      clearFailureAndRecovery();
      last_recovery_count_ = 0;
      active_behaviors_.clear();
      return;
    }

    if (!isTerminalTaskStatus(status)) {
      return;
    }

    if (isFollowWaypointsActive()) {
      if (status == GoalStatus::STATUS_SUCCEEDED) {
        clearFailureAndRecovery();
        active_behaviors_.clear();
        last_recovery_count_ = 0;
      } else if (
        status == GoalStatus::STATUS_ABORTED &&
        last_failure_category_ == "planning")
      {
        last_failure_category_ = "unreachable_waypoint";
        last_failure_detail_ = "navigation_aborted_after_planning_failure";
      }
      return;
    }

    if (status == GoalStatus::STATUS_ABORTED && last_failure_category_ == "planning") {
      last_failure_category_ = "unreachable_waypoint";
      last_failure_detail_ = "navigation_aborted_after_planning_failure";
    }

    last_recovery_count_ = 0;
    in_recovery_ = false;
    active_behaviors_.clear();
    current_waypoint_ = -1;
    resetFailureTracking();
  }

  void resetFailureTracking()
  {
    bt_failures_.clear();
    obstacle_ahead_active_ = false;
    follow_path_failed_active_ = false;
    blocked_scan_count_ = 0;
    clear_scan_count_ = 0;
    last_failure_category_ = "unknown";
    last_failure_detail_ = "unknown";
    last_failed_bt_node_.clear();
  }

  bool hasFailureInfo() const
  {
    return last_failure_category_ != "unknown" ||
           last_failure_detail_ != "unknown" ||
           !last_failed_bt_node_.empty();
  }

  void clearFailureAndRecovery()
  {
    if (!in_recovery_ &&
      last_failure_category_ == "unknown" &&
      last_failure_detail_ == "unknown" &&
      last_failed_bt_node_.empty())
    {
      return;
    }
    in_recovery_ = false;
    resetFailureTracking();
    publishStatus();
  }

  void navigateToPoseStatusCb(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    updateTaskStatusFromArray(*msg, "navigate_to_pose");
  }

  void navigateThroughPosesStatusCb(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    updateTaskStatusFromArray(*msg, "navigate_through_poses");
  }

  void followWaypointsStatusCb(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    updateTaskStatusFromArray(*msg, "follow_waypoints");
  }

  void navigateToPoseFeedbackCb(const NavigateToPoseFeedback::SharedPtr msg)
  {
    if (!ensureNav2AvailableForUpdate()) {
      return;
    }
    handleRecoveryCount(msg->feedback.number_of_recoveries);
  }

  void navigateThroughPosesFeedbackCb(const NavigateThroughPosesFeedback::SharedPtr msg)
  {
    if (!ensureNav2AvailableForUpdate()) {
      return;
    }
    handleRecoveryCount(msg->feedback.number_of_recoveries);
  }

  void followWaypointsFeedbackCb(const FollowWaypointsFeedback::SharedPtr msg)
  {
    if (!ensureNav2AvailableForUpdate()) {
      return;
    }
    const auto waypoint = static_cast<int32_t>(msg->feedback.current_waypoint);
    if (waypoint == current_waypoint_) {
      return;
    }
    current_waypoint_ = waypoint;
    clearFailureAndRecovery();
  }

  void behaviorTreeLogCb(const nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg)
  {
    if (!ensureNav2AvailableForUpdate()) {
      return;
    }

    for (const auto & event : msg->event_log) {
      if (kControlNodes.count(event.node_name) > 0 && event.current_status == "SUCCESS") {
        handleFollowPathSuccess();
        continue;
      }

      if (event.current_status != "FAILURE") {
        continue;
      }

      bt_failures_.push_back(BtFailureRecord{event.node_name});
      if (bt_failures_.size() > 32) {
        bt_failures_.pop_front();
      }

      const std::string previous_category = last_failure_category_;
      const std::string previous_detail = last_failure_detail_;
      const std::string previous_node = last_failed_bt_node_;
      const bool previous_follow_path_active = follow_path_failed_active_;

      if (kPlanningNodes.count(event.node_name) > 0) {
        last_failure_category_ = "planning";
        last_failure_detail_ = "no_path_to_goal";
        last_failed_bt_node_ = event.node_name;
      } else if (kControlNodes.count(event.node_name) > 0) {
        follow_path_failed_active_ = true;
        last_failure_category_ = "control";
        last_failure_detail_ = "follow_path_failed";
        last_failed_bt_node_ = event.node_name;
      } else if (kPlanningRecoveryNodes.count(event.node_name) > 0) {
        if (last_failure_category_ == "unknown") {
          last_failure_category_ = "planning";
          last_failure_detail_ = "planning_recovery";
          last_failed_bt_node_ = event.node_name;
        }
      } else if (kControlRecoveryNodes.count(event.node_name) > 0) {
        if (last_failure_category_ == "unknown") {
          last_failure_category_ = "control";
          last_failure_detail_ = "control_recovery";
          last_failed_bt_node_ = event.node_name;
        }
      }

      if (last_failure_category_ != previous_category ||
        last_failure_detail_ != previous_detail ||
        last_failed_bt_node_ != previous_node ||
        follow_path_failed_active_ != previous_follow_path_active)
      {
        publishStatus();
      }
    }
  }

  void handleRecoveryCount(uint16_t recovery_count)
  {
    if (recovery_count == 0 && last_recovery_count_ > 0) {
      last_recovery_count_ = 0;
      in_recovery_ = false;
      active_behaviors_.clear();
      resetFailureTracking();
      publishStatus();
      return;
    }

    if (recovery_count <= last_recovery_count_) {
      return;
    }

    last_recovery_count_ = recovery_count;
    in_recovery_ = true;

    std::string category;
    std::string detail;
    std::string bt_node;
    classifyRecoveryCause(category, detail, bt_node);
    last_failure_category_ = category;
    last_failure_detail_ = detail;
    last_failed_bt_node_ = bt_node;
    if (detail == "follow_path_failed") {
      follow_path_failed_active_ = true;
    }

    publishStatus();
  }

  void classifyRecoveryCause(
    std::string & category,
    std::string & detail,
    std::string & bt_node) const
  {
    for (auto it = bt_failures_.rbegin(); it != bt_failures_.rend(); ++it) {
      if (kPlanningNodes.count(it->node_name) > 0) {
        category = "planning";
        detail = "no_path_to_goal";
        bt_node = it->node_name;
        return;
      }
      if (kControlNodes.count(it->node_name) > 0) {
        category = "control";
        detail = "follow_path_failed";
        bt_node = it->node_name;
        return;
      }
    }

    category = last_failure_category_;
    detail = last_failure_detail_;
    bt_node = last_failed_bt_node_;
  }

  void behaviorStatusCb(
    const action_msgs::msg::GoalStatusArray::SharedPtr msg,
    const std::string & behavior)
  {
    if (!ensureNav2AvailableForUpdate()) {
      return;
    }

    const bool active = std::any_of(
      msg->status_list.begin(),
      msg->status_list.end(),
      [](const action_msgs::msg::GoalStatus & status) {
        return status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
      });

    if (active) {
      if (active_behaviors_.count(behavior) == 0) {
        active_behaviors_.insert(behavior);
        publishStatus();
      }
      return;
    }

    if (active_behaviors_.count(behavior) > 0) {
      active_behaviors_.erase(behavior);
      if (active_behaviors_.empty() && (in_recovery_ || hasFailureInfo())) {
        clearFailureAndRecovery();
      } else {
        publishStatus();
      }
    }
  }

  rclcpp::Publisher<Nav2StatusMsg>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr obstacle_scan_pub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;

  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigate_to_pose_status_sub_;
  rclcpp::Subscription<NavigateToPoseFeedback>::SharedPtr navigate_to_pose_feedback_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr navigate_through_poses_status_sub_;
  rclcpp::Subscription<NavigateThroughPosesFeedback>::SharedPtr navigate_through_poses_feedback_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr follow_waypoints_status_sub_;
  rclcpp::Subscription<FollowWaypointsFeedback>::SharedPtr follow_waypoints_feedback_sub_;
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr behavior_tree_log_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::vector<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr>
  behavior_status_subs_;

  double publish_period_sec_{2.0};
  std::string nav2_availability_topic_;
  bool early_obstacle_detection_{true};
  double obstacle_ahead_distance_{2.5};
  double obstacle_ahead_clear_distance_{3.5};
  double obstacle_ahead_half_angle_rad_{0.61};
  int obstacle_ahead_confirm_scans_{20};
  int obstacle_ahead_clear_confirm_scans_{2};
  int obstacle_ahead_min_points_{8};
  int blocked_scan_count_{0};
  int clear_scan_count_{0};

  bool nav2_available_{false};
  std::unordered_map<std::string, int8_t> task_states_;
  std::string navigation_state_{"idle"};
  std::string task_source_{"none"};
  std::string task_status_{"unknown"};
  std::string subtask_source_;
  std::string subtask_status_{"unknown"};

  uint16_t last_recovery_count_{0};
  bool in_recovery_{false};
  std::set<std::string> active_behaviors_;

  int32_t current_waypoint_{-1};

  std::deque<BtFailureRecord> bt_failures_;
  std::string last_failure_category_{"unknown"};
  std::string last_failure_detail_{"unknown"};
  std::string last_failed_bt_node_;
  bool obstacle_ahead_active_{false};
  bool follow_path_failed_active_{false};

  Nav2StatusMsg last_published_;
};

const std::vector<std::string> Nav2StatusMonitor::kTaskSources = {
  "follow_waypoints",
  "navigate_through_poses",
  "navigate_to_pose",
};

const std::vector<std::string> Nav2StatusMonitor::kSubtaskSources = {
  "navigate_to_pose",
  "navigate_through_poses",
};

const std::unordered_set<std::string> Nav2StatusMonitor::kPlanningNodes = {
  "ComputePathToPose",
  "ComputePathThroughPoses",
};

const std::unordered_set<std::string> Nav2StatusMonitor::kControlNodes = {
  "FollowPath",
};

const std::unordered_set<std::string> Nav2StatusMonitor::kPlanningRecoveryNodes = {
  "ClearGlobalCostmap-Context",
  "ClearGlobalCostmap-Subtree",
};

const std::unordered_set<std::string> Nav2StatusMonitor::kControlRecoveryNodes = {
  "ClearLocalCostmap-Context",
  "ClearLocalCostmap-Subtree",
};

}  // namespace nav2_status_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_status_monitor::Nav2StatusMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
