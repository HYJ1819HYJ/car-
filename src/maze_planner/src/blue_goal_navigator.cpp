#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <stm32_bridge/BlueTargetObservation.h>
#include <stm32_bridge/BlackLineObservation.h>

#include <maze_planner/wall_parking.h>
#include <maze_planner/visual_tracking.h>
#include <maze_planner/search_waypoint_order.h>
#include <XmlRpcException.h>
#include <XmlRpcValue.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace {

double clampValue(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion) {
  tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::Quaternion quaternionFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::Quaternion result;
  result.x = q.x();
  result.y = q.y();
  result.z = q.z();
  result.w = q.w();
  return result;
}

}  // namespace

class BlueGoalNavigator {
 public:
  using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  BlueGoalNavigator()
      : nh_(), pnh_("~"), move_base_("move_base", true),
        tf_listener_(tf_buffer_) {
    pnh_.param<bool>("auto_start", auto_start_, false);
    pnh_.param<double>("start_delay", start_delay_, 5.0);
    pnh_.param<bool>("search_enabled", search_enabled_, true);
    pnh_.param<bool>("acquire_target_in_navigation", acquire_target_in_navigation_, true);
    pnh_.param<bool>("hold_at_course_end", hold_at_course_end_, false);
    pnh_.param<bool>("require_localization", require_localization_, true);
    pnh_.param<bool>("require_gimbal", require_gimbal_, false);
    pnh_.param<bool>("require_camera_info", require_camera_info_, false);
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    pnh_.param<double>("camera_x", camera_x_, 0.145);
    pnh_.param<double>("camera_y", camera_y_, 0.0);
    pnh_.param<double>("paper_height", paper_height_, 0.297);
    pnh_.param<double>("fallback_hfov_deg", fallback_hfov_deg_, 70.0);
    pnh_.param<double>("min_pixel_height", min_pixel_height_, 18.0);
    pnh_.param<double>("min_target_range", min_target_range_, 0.45);
    pnh_.param<double>("max_target_range", max_target_range_, 8.0);
    pnh_.param<int>("target_sample_count", target_sample_count_, 10);
    pnh_.param<double>("target_sample_max_age", target_sample_max_age_, 1.5);
    pnh_.param<double>("target_max_stddev", target_max_stddev_, 0.30);
    pnh_.param<double>("target_lost_timeout", target_lost_timeout_, 1.5);
    pnh_.param<double>("approach_standoff", approach_standoff_, 0.65);
    pnh_.param<double>("approach_clearance", approach_clearance_, 0.46);
    pnh_.param<double>("goal_update_distance", goal_update_distance_, 0.25);
    pnh_.param<double>("goal_update_period", goal_update_period_, 2.0);
    pnh_.param<double>("search_spacing", search_spacing_, 1.5);
    pnh_.param<double>("search_clearance", search_clearance_, 0.50);
    pnh_.param<double>("search_goal_timeout", search_goal_timeout_, 30.0);
    pnh_.param<int>("max_search_waypoints", max_search_waypoints_, 250);
    pnh_.param<bool>("prefer_forward_search", prefer_forward_search_, false);
    pnh_.param<bool>("line_guidance_enabled", line_guidance_enabled_, true);
    pnh_.param<double>("waypoint_observation_time", waypoint_observation_time_, 3.0);
    pnh_.param<double>("line_scan_hold_time", line_scan_hold_time_, 10.0);
    pnh_.param<double>("line_retrigger_cooldown", line_retrigger_cooldown_, 12.0);
    pnh_.param<double>("black_line_timeout", black_line_timeout_, 0.6);
    pnh_.param<bool>("wall_parking_enabled", wall_parking_enabled_, false);
    pnh_.param<double>("wall_parking_standoff", wall_parking_standoff_, 0.56);
    pnh_.param<double>("wall_parking_xy_tolerance", wall_parking_xy_tolerance_, 0.04);
    pnh_.param<double>("wall_parking_yaw_tolerance", wall_parking_yaw_tolerance_, 0.08);
    pnh_.param<bool>("visual_tracking_parking_enabled", visual_tracking_parking_enabled_, false);
    pnh_.param<double>("visual_stop_area_ratio", visual_stop_area_ratio_, 0.16);
    pnh_.param<double>("visual_facing_tolerance_deg", visual_facing_tolerance_deg_, 5.0);
    pnh_.param<double>("visual_approach_standoff", visual_approach_standoff_, 0.48);
    pnh_.param<double>("visual_approach_step", visual_approach_step_, 0.05);
    pnh_.param<double>("visual_goal_xy_tolerance", visual_goal_xy_tolerance_, 0.04);
    pnh_.param<double>("visual_goal_yaw_tolerance", visual_goal_yaw_tolerance_, 0.08);
    pnh_.param<double>("visual_goal_update_period", visual_goal_update_period_, 1.0);
    pnh_.param<double>("max_localization_position_stddev",
                        max_localization_position_stddev_, 0.50);
    pnh_.param<double>("max_localization_yaw_stddev",
                        max_localization_yaw_stddev_, 0.50);

    target_sample_count_ = std::max(3, target_sample_count_);
    max_search_waypoints_ = std::max(1, max_search_waypoints_);
    paper_height_ = std::max(0.05, paper_height_);
    approach_standoff_ = std::max(0.20, approach_standoff_);
    search_spacing_ = std::max(0.30, search_spacing_);
    waypoint_observation_time_ = std::max(0.5, waypoint_observation_time_);
    line_scan_hold_time_ = std::max(1.0, line_scan_hold_time_);
    line_retrigger_cooldown_ = std::max(1.0, line_retrigger_cooldown_);
    black_line_timeout_ = std::max(0.1, black_line_timeout_);
    visual_stop_area_ratio_ = clampValue(visual_stop_area_ratio_, 0.0, 1.0);
    visual_facing_tolerance_deg_ = clampValue(visual_facing_tolerance_deg_, 0.0, 180.0);
    visual_approach_standoff_ = std::max(0.0, visual_approach_standoff_);
    visual_approach_step_ = std::max(0.01, visual_approach_step_);
    visual_goal_xy_tolerance_ = std::max(0.001, visual_goal_xy_tolerance_);
    visual_goal_yaw_tolerance_ = std::max(0.001, visual_goal_yaw_tolerance_);
    current_visual_standoff_ = visual_approach_standoff_;
    visual_goal_update_period_ = std::max(0.1, visual_goal_update_period_);
    if (wall_parking_enabled_ || visual_tracking_parking_enabled_)
      min_target_range_ = std::min(min_target_range_, 0.25);

    observation_sub_ = nh_.subscribe("/blue_target/observation", 10,
                                     &BlueGoalNavigator::observationCb, this);
    black_line_sub_ = nh_.subscribe("/black_line/observation", 10,
                                    &BlueGoalNavigator::blackLineCb, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &BlueGoalNavigator::scanCb, this);
    camera_info_sub_ = nh_.subscribe("/usb_cam/camera_info", 2,
                                     &BlueGoalNavigator::cameraInfoCb, this);
    map_sub_ = nh_.subscribe("/map", 1, &BlueGoalNavigator::mapCb, this);
    amcl_sub_ = nh_.subscribe("/amcl_pose", 2, &BlueGoalNavigator::amclCb, this);
    joint_sub_ = nh_.subscribe("/gimbal/joint_states", 5,
                               &BlueGoalNavigator::jointCb, this);
    gimbal_connected_sub_ = nh_.subscribe("/gimbal/connected", 2,
                                          &BlueGoalNavigator::gimbalConnectedCb, this);
    gimbal_ready_sub_ = nh_.subscribe("/gimbal/servo_ready", 2,
                                      &BlueGoalNavigator::gimbalReadyCb, this);
    gimbal_locked_sub_ = nh_.subscribe("/gimbal/target_locked", 2,
                                       &BlueGoalNavigator::gimbalLockedCb, this);
    enable_sub_ = nh_.subscribe("/blue_autonomy/enable", 2,
                                &BlueGoalNavigator::enableCb, this);
    if (hold_at_course_end_ && !acquire_target_in_navigation_) {
      finish_observation_sub_ = nh_.subscribe("/handoff/finish_observation_goal", 1,
          &BlueGoalNavigator::finishObservationCb, this);
    }
    tolerance_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(
        "/move_base/TebLocalPlannerROS/set_parameters");
    loadFootprint();
    loadCourseWaypoints();

    state_pub_ = nh_.advertise<std_msgs::String>("/blue_autonomy/state", 1, true);
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/blue_target/map_pose", 10);
    approach_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/blue_target/approach_goal", 10, true);
    goal_reached_pub_ = nh_.advertise<std_msgs::Bool>("/goal_reached", 1, true);

    node_start_time_ = ros::Time::now();
    enabled_ = auto_start_;
    mission_enable_time_ = node_start_time_;
    publishGoalReached(false);
    setState(enabled_ ? State::WAIT_SYSTEM : State::IDLE,
             enabled_ ? "waiting for navigation, localization and camera" :
                        "disabled; publish true on /blue_autonomy/enable");
    timer_ = nh_.createTimer(ros::Duration(0.10), &BlueGoalNavigator::timerCb, this);
  }

 private:
  enum class State {
    IDLE, WAIT_SYSTEM, SEARCH, OBSERVE, LINE_SCAN, ACQUIRE, APPROACH, COMPLETE, FAILED
  };
  enum class GoalKind { NONE, SEARCH, APPROACH };

  struct TargetEstimate {
    double x{0.0};
    double y{0.0};
    double range{0.0};
    ros::Time stamp;
  };

  struct Point2D {
    double x{0.0};
    double y{0.0};
  };

  void loadCourseWaypoints() {
    XmlRpc::XmlRpcValue values;
    if (!pnh_.getParam("course_waypoints", values)) return;
    if (values.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR("course_waypoints must be an array of [x, y] points");
      return;
    }
    for (int i = 0; i < values.size(); ++i) {
      if (values[i].getType() != XmlRpc::XmlRpcValue::TypeArray ||
          values[i].size() != 2) {
        ROS_ERROR("Invalid course waypoint at index %d", i);
        course_waypoints_.clear();
        return;
      }
      double coordinates[2];
      for (int j = 0; j < 2; ++j) {
        const auto type = values[i][j].getType();
        if (type == XmlRpc::XmlRpcValue::TypeInt)
          coordinates[j] = static_cast<int>(values[i][j]);
        else if (type == XmlRpc::XmlRpcValue::TypeDouble)
          coordinates[j] = static_cast<double>(values[i][j]);
        else {
          ROS_ERROR("Invalid course waypoint coordinate at index %d", i);
          course_waypoints_.clear();
          return;
        }
        if (!std::isfinite(coordinates[j])) {
          ROS_ERROR("Non-finite course waypoint at index %d", i);
          course_waypoints_.clear();
          return;
        }
      }
      course_waypoints_.push_back(Point2D{coordinates[0], coordinates[1]});
    }
    ROS_INFO("Loaded %zu mapped-course waypoints", course_waypoints_.size());
  }

  void loadFootprint() {
    XmlRpc::XmlRpcValue value;
    if (nh_.getParam("/robot_geometry/footprint", value) &&
        value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < value.size(); ++i) {
        if (value[i].getType() != XmlRpc::XmlRpcValue::TypeArray ||
            value[i].size() != 2) continue;
        try {
          const double x = value[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt
              ? static_cast<int>(value[i][0]) : static_cast<double>(value[i][0]);
          const double y = value[i][1].getType() == XmlRpc::XmlRpcValue::TypeInt
              ? static_cast<int>(value[i][1]) : static_cast<double>(value[i][1]);
          footprint_.emplace_back(x, y);
        } catch (const XmlRpc::XmlRpcException&) {
          footprint_.clear();
          break;
        }
      }
    }
    if (footprint_.size() < 3) {
      ROS_WARN("STL footprint unavailable; using configured rectangular extent for finish check");
      footprint_ = {maze_planner::Point2(0.3483, 0.245),
                    maze_planner::Point2(0.3483, -0.245),
                    maze_planner::Point2(-0.1517, -0.245),
                    maze_planner::Point2(-0.1517, 0.245)};
    }
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
    latest_scan_ = *msg;
    last_scan_time_ = ros::Time::now();
    have_scan_ = true;
  }

  bool setParkingTolerance(bool parking) {
    if ((!wall_parking_enabled_ && !visual_tracking_parking_enabled_) ||
        parking == parking_tolerance_active_) return true;
    const std::string prefix = "/move_base/TebLocalPlannerROS/";
    if (parking) {
      nh_.param<double>(prefix + "xy_goal_tolerance", previous_xy_tolerance_, 0.12);
      nh_.param<double>(prefix + "yaw_goal_tolerance", previous_yaw_tolerance_, 0.12);
    }
    dynamic_reconfigure::Reconfigure request;
    dynamic_reconfigure::DoubleParameter xy;
    xy.name = "xy_goal_tolerance";
    xy.value = parking ? (visual_tracking_parking_enabled_ ? visual_goal_xy_tolerance_
                                                          : wall_parking_xy_tolerance_)
                       : previous_xy_tolerance_;
    dynamic_reconfigure::DoubleParameter yaw;
    yaw.name = "yaw_goal_tolerance";
    yaw.value = parking ? (visual_tracking_parking_enabled_ ? visual_goal_yaw_tolerance_
                                                           : wall_parking_yaw_tolerance_)
                        : previous_yaw_tolerance_;
    request.request.config.doubles.push_back(xy);
    request.request.config.doubles.push_back(yaw);
    if (!tolerance_client_.call(request)) {
      ROS_WARN_THROTTLE(5.0, "Unable to update TEB goal tolerance for wall parking");
      return false;
    }
    parking_tolerance_active_ = parking;
    ROS_INFO("TEB goal tolerances set to xy=%.3f yaw=%.3f", xy.value, yaw.value);
    return true;
  }

  bool targetBearingInBase(double* bearing) {
    if (!bearing || !have_target_observation_ || !target_visible_ ||
        (ros::Time::now() - last_target_seen_time_).toSec() > 0.5 ||
        (require_gimbal_ && (ros::Time::now() - last_joint_time_).toSec() > 1.5))
      return false;

    const stm32_bridge::BlueTargetObservation& observation = latest_observation_;
    if (!observation.image_width || !observation.image_height) return false;
    double fx = 0.0, fy = 0.0;
    double cx = 0.5 * observation.image_width;
    double cy = 0.5 * observation.image_height;
    if (have_camera_info_ && camera_info_.width > 0 && camera_info_.height > 0) {
      const double sx = static_cast<double>(observation.image_width) / camera_info_.width;
      const double sy = static_cast<double>(observation.image_height) / camera_info_.height;
      fx = camera_info_.K[0] * sx;
      fy = camera_info_.K[4] * sy;
      cx = camera_info_.K[2] * sx;
      cy = camera_info_.K[5] * sy;
    } else {
      const double hfov = clampValue(fallback_hfov_deg_, 20.0, 150.0) * M_PI / 180.0;
      fx = observation.image_width / (2.0 * std::tan(0.5 * hfov));
      fy = fx;
    }
    if (fx <= 1.0 || fy <= 1.0) return false;
    const bool fresh_gimbal = (ros::Time::now() - last_joint_time_).toSec() <= 1.5;
    const double pan = fresh_gimbal ? pan_angle_ : 0.0;
    const double tilt = fresh_gimbal ? tilt_angle_ : 0.0;
    return maze_planner::visualBearing(pan, tilt, observation.center_x,
                                      observation.center_y, cx, cy, fx, fy, bearing);
  }

  bool visualParkingReached() {
    double bearing = 0.0;
    return targetBearingInBase(&bearing) &&
           maze_planner::visualStopReached(target_visible_, latest_observation_.contour_ratio,
                                            bearing, visual_stop_area_ratio_,
                                            visual_facing_tolerance_deg_ * M_PI / 180.0);
  }

  bool targetWallInMap(maze_planner::WallObservation* result) {
    double bearing = 0.0;
    if (!result || !have_scan_ || (ros::Time::now() - last_scan_time_).toSec() > 0.5 ||
        !targetBearingInBase(&bearing)) return false;

    geometry_msgs::TransformStamped laser_to_base;
    try {
      laser_to_base = tf_buffer_.lookupTransform(base_frame_, latest_scan_.header.frame_id,
                                                 ros::Time(0), ros::Duration(0.05));
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(3.0, "Wall fitting cannot transform laser scan: %s", error.what());
      return false;
    }
    const double laser_yaw = yawFromQuaternion(laser_to_base.transform.rotation);
    const double lx = laser_to_base.transform.translation.x;
    const double ly = laser_to_base.transform.translation.y;
    std::vector<maze_planner::Point2> scan;
    scan.reserve(latest_scan_.ranges.size());
    for (std::size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
      const double range = latest_scan_.ranges[i];
      if (!std::isfinite(range) || range <= latest_scan_.range_min ||
          range >= latest_scan_.range_max || range > 4.0) continue;
      const double angle = laser_yaw + latest_scan_.angle_min + i * latest_scan_.angle_increment;
      scan.emplace_back(lx + range * std::cos(angle), ly + range * std::sin(angle));
    }
    maze_planner::WallObservation wall;
    if (!maze_planner::fitTargetWall(scan, maze_planner::Point2(camera_x_, camera_y_),
                                    bearing, &wall)) return false;
    double x = 0.0, y = 0.0, robot_yaw = 0.0;
    if (!robotPose(&x, &y, &robot_yaw)) return false;
    result->marker = maze_planner::Point2(x, y) + maze_planner::rotate(wall.marker, robot_yaw);
    result->outward = maze_planner::rotate(wall.outward, robot_yaw);
    result->inliers = wall.inliers;
    result->span = wall.span;
    return true;
  }

  const char* stateName(State state) const {
    switch (state) {
      case State::IDLE: return "IDLE";
      case State::WAIT_SYSTEM: return "WAIT_SYSTEM";
      case State::SEARCH: return "SEARCH";
      case State::OBSERVE: return "OBSERVE";
      case State::LINE_SCAN: return "LINE_SCAN";
      case State::ACQUIRE: return "ACQUIRE";
      case State::APPROACH: return "APPROACH";
      case State::COMPLETE: return "COMPLETE";
      case State::FAILED: return "FAILED";
    }
    return "UNKNOWN";
  }

  void setState(State state, const std::string& reason) {
    if (state_ == state && reason == state_reason_) return;
    state_ = state;
    state_reason_ = reason;
    std_msgs::String message;
    message.data = std::string(stateName(state_)) + ": " + reason;
    state_pub_.publish(message);
    ROS_INFO("Blue autonomy state -> %s", message.data.c_str());
  }

  void publishGoalReached(bool reached) {
    std_msgs::Bool message;
    message.data = reached;
    goal_reached_pub_.publish(message);
  }

  void enableCb(const std_msgs::Bool::ConstPtr& msg) {
    if (!msg->data) {
      enabled_ = false;
      if (goal_active_) move_base_.cancelGoal();
      goal_active_ = false;
      goal_kind_ = GoalKind::NONE;
      estimates_.clear();
      setParkingTolerance(false);
      publishGoalReached(false);
      setState(State::IDLE, "disabled by /blue_autonomy/enable");
      return;
    }

    if (!enabled_ || state_ == State::COMPLETE || state_ == State::FAILED) {
      enabled_ = true;
      mission_enable_time_ = ros::Time::now();
      goal_active_ = false;
      goal_kind_ = GoalKind::NONE;
      search_index_ = 0;
      course_index_ = 0;
      finish_observation_attempted_ = false;
      current_goal_is_course_ = false;
      estimates_.clear();
      current_visual_standoff_ = visual_approach_standoff_;
      setParkingTolerance(false);
      black_line_visible_ = false;
      publishGoalReached(false);
      setState(State::WAIT_SYSTEM, "mission enabled");
    }
  }

  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    const bool geometry_changed = !have_map_ ||
        map_.info.width != msg->info.width || map_.info.height != msg->info.height ||
        std::fabs(map_.info.resolution - msg->info.resolution) > 1e-9 ||
        std::fabs(map_.info.origin.position.x - msg->info.origin.position.x) > 1e-6 ||
        std::fabs(map_.info.origin.position.y - msg->info.origin.position.y) > 1e-6;
    map_ = *msg;
    have_map_ = true;
    if (geometry_changed) {
      search_waypoints_.clear();
      search_index_ = 0;
    }
  }

  void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    camera_info_ = *msg;
    have_camera_info_ = std::isfinite(msg->K[0]) && std::isfinite(msg->K[4]) &&
                        msg->K[0] > 1.0 && msg->K[4] > 1.0;
    if (!have_camera_info_) {
      ROS_WARN_THROTTLE(10.0, "CameraInfo has no valid calibration; using HFOV fallback");
    }
  }

  void amclCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    have_amcl_ = true;
    const double sx = std::sqrt(std::max(0.0, msg->pose.covariance[0]));
    const double sy = std::sqrt(std::max(0.0, msg->pose.covariance[7]));
    const double syaw = std::sqrt(std::max(0.0, msg->pose.covariance[35]));
    localization_good_ = std::max(sx, sy) <= max_localization_position_stddev_ &&
                         syaw <= max_localization_yaw_stddev_;
    localization_position_stddev_ = std::max(sx, sy);
    localization_yaw_stddev_ = syaw;
  }

  void jointCb(const sensor_msgs::JointState::ConstPtr& msg) {
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      if (msg->name[i] == "camera_pan_joint") pan_angle_ = msg->position[i];
      if (msg->name[i] == "camera_tilt_joint") tilt_angle_ = msg->position[i];
    }
    last_joint_time_ = ros::Time::now();
  }

  void gimbalConnectedCb(const std_msgs::Bool::ConstPtr& msg) {
    gimbal_connected_ = msg->data;
  }

  void gimbalReadyCb(const std_msgs::Bool::ConstPtr& msg) {
    gimbal_ready_ = msg->data;
  }

  void gimbalLockedCb(const std_msgs::Bool::ConstPtr& msg) {
    gimbal_locked_ = msg->data;
  }

  bool robotPose(double* x, double* y, double* yaw) {
    try {
      const geometry_msgs::TransformStamped transform =
          tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0),
                                     ros::Duration(0.05));
      *x = transform.transform.translation.x;
      *y = transform.transform.translation.y;
      *yaw = yawFromQuaternion(transform.transform.rotation);
      return true;
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(2.0, "Cannot transform %s -> %s: %s",
                        base_frame_.c_str(), map_frame_.c_str(), error.what());
      return false;
    }
  }

  bool systemReady(std::string* reason) {
    if (!move_base_.isServerConnected()) {
      *reason = "waiting for move_base action server";
      return false;
    }
    if (!have_map_) {
      *reason = "waiting for /map";
      return false;
    }
    if ((ros::Time::now() - last_observation_time_).toSec() > 1.0) {
      *reason = "waiting for camera/blue detector";
      return false;
    }
    if (require_camera_info_ && !have_camera_info_) {
      *reason = "waiting for calibrated /usb_cam/camera_info";
      return false;
    }
    if (require_localization_ && (!have_amcl_ || !localization_good_)) {
      std::ostringstream stream;
      stream << "AMCL not converged (position_std=" << localization_position_stddev_
             << ", yaw_std=" << localization_yaw_stddev_ << ")";
      *reason = stream.str();
      return false;
    }
    if (require_gimbal_ && (!gimbal_connected_ || !gimbal_ready_)) {
      *reason = "waiting for required gimbal serial and both servos";
      return false;
    }
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    if (!robotPose(&x, &y, &yaw)) {
      *reason = "waiting for map -> base_footprint TF";
      return false;
    }
    return true;
  }

  void observationCb(const stm32_bridge::BlueTargetObservation::ConstPtr& msg) {
    last_observation_time_ = ros::Time::now();
    if (!acquire_target_in_navigation_) {
      // The handoff manager independently observes the target and only switches
      // after its lidar corridor check. Do not interrupt the mapped course.
      target_visible_ = false;
      return;
    }
    target_visible_ = msg->detected;
    if (!msg->detected) return;
    latest_observation_ = *msg;
    have_target_observation_ = true;
    last_target_seen_time_ = ros::Time::now();
    if (!enabled_ || state_ == State::COMPLETE) return;

    // 已经连续确认蓝纸时先停车获取稳定观测，避免云台锁定期间底盘仍在
    // 沿搜索路径运动。之后只有锁定且位置样本稳定才会发送接近目标。
    if (state_ == State::SEARCH || state_ == State::OBSERVE ||
        state_ == State::LINE_SCAN) {
      if (goal_active_) move_base_.cancelGoal();
      goal_active_ = false;
      goal_kind_ = GoalKind::NONE;
      estimates_.clear();
      current_visual_standoff_ = visual_approach_standoff_;
      setState(State::ACQUIRE, "blue target detected; stabilizing map position");
    }

    if (visual_tracking_parking_enabled_ &&
        (state_ == State::ACQUIRE || state_ == State::APPROACH) &&
        visualParkingReached()) {
      finishMission();
      return;
    }

    // Visual following uses the measured pan and the current image offset even
    // while the firmware's lock flag flickers; estimateTarget still requires a
    // fresh joint reading when a gimbal is configured.
    if (require_gimbal_ && !gimbal_locked_ && !visual_tracking_parking_enabled_) return;

    TargetEstimate estimate;
    if (!estimateTarget(*msg, &estimate)) return;
    target_pose_valid_ = true;
    latest_target_ = estimate;
    estimates_.push_back(estimate);
    trimEstimates();

    geometry_msgs::PoseStamped target;
    target.header.stamp = estimate.stamp;
    target.header.frame_id = map_frame_;
    target.pose.position.x = estimate.x;
    target.pose.position.y = estimate.y;
    target.pose.orientation.w = 1.0;
    target_pose_pub_.publish(target);

  }

  bool estimateTarget(const stm32_bridge::BlueTargetObservation& observation,
                      TargetEstimate* estimate) {
    if (observation.image_width == 0 || observation.image_height == 0 ||
        observation.pixel_height < min_pixel_height_) return false;

    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.5 * observation.image_width;
    double cy = 0.5 * observation.image_height;
    if (have_camera_info_ && camera_info_.width > 0 && camera_info_.height > 0) {
      const double sx = static_cast<double>(observation.image_width) / camera_info_.width;
      const double sy = static_cast<double>(observation.image_height) / camera_info_.height;
      fx = camera_info_.K[0] * sx;
      fy = camera_info_.K[4] * sy;
      cx = camera_info_.K[2] * sx;
      cy = camera_info_.K[5] * sy;
    } else {
      const double hfov = clampValue(fallback_hfov_deg_, 20.0, 150.0) * M_PI / 180.0;
      fx = observation.image_width / (2.0 * std::tan(0.5 * hfov));
      fy = fx;
    }

    if (fx <= 1.0 || fy <= 1.0) return false;
    const double slant_range = fy * paper_height_ / observation.pixel_height;
    if (!std::isfinite(slant_range) || slant_range < min_target_range_ ||
        slant_range > max_target_range_) return false;

    const bool fresh_gimbal = (ros::Time::now() - last_joint_time_).toSec() <= 1.5;
    if (require_gimbal_ && !fresh_gimbal) return false;
    const double pan = fresh_gimbal ? pan_angle_ : 0.0;
    const double tilt = fresh_gimbal ? tilt_angle_ : 0.0;
    const double pixel_bearing = std::atan2(cx - observation.center_x, fx);
    const double pixel_down_angle = std::atan2(observation.center_y - cy, fy);
    const double bearing = pan + pixel_bearing;
    const double down_angle = tilt + pixel_down_angle;
    const double horizontal_range = slant_range * std::cos(down_angle);
    if (!std::isfinite(horizontal_range) || horizontal_range <= 0.0) return false;

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!robotPose(&robot_x, &robot_y, &robot_yaw)) return false;
    const double local_x = camera_x_ + horizontal_range * std::cos(bearing);
    const double local_y = camera_y_ + horizontal_range * std::sin(bearing);
    estimate->x = robot_x + std::cos(robot_yaw) * local_x - std::sin(robot_yaw) * local_y;
    estimate->y = robot_y + std::sin(robot_yaw) * local_x + std::cos(robot_yaw) * local_y;
    estimate->range = horizontal_range;
    estimate->stamp = ros::Time::now();
    return true;
  }

  void trimEstimates() {
    const ros::Time cutoff = ros::Time::now() - ros::Duration(target_sample_max_age_);
    while (!estimates_.empty() &&
           (estimates_.front().stamp < cutoff ||
            static_cast<int>(estimates_.size()) > target_sample_count_ * 3)) {
      estimates_.pop_front();
    }
  }

  bool stableTarget(TargetEstimate* result, double* stddev) {
    trimEstimates();
    if (static_cast<int>(estimates_.size()) < target_sample_count_) return false;
    double mean_x = 0.0;
    double mean_y = 0.0;
    double mean_range = 0.0;
    for (const TargetEstimate& value : estimates_) {
      mean_x += value.x;
      mean_y += value.y;
      mean_range += value.range;
    }
    const double count = static_cast<double>(estimates_.size());
    mean_x /= count;
    mean_y /= count;
    mean_range /= count;
    double variance = 0.0;
    for (const TargetEstimate& value : estimates_) {
      const double dx = value.x - mean_x;
      const double dy = value.y - mean_y;
      variance += dx * dx + dy * dy;
    }
    *stddev = std::sqrt(variance / count);
    if (*stddev > target_max_stddev_) return false;
    result->x = mean_x;
    result->y = mean_y;
    result->range = mean_range;
    result->stamp = ros::Time::now();
    return true;
  }

  bool worldToMap(double wx, double wy, int* mx, int* my) const {
    if (!have_map_ || map_.info.resolution <= 0.0) return false;
    const double origin_yaw = yawFromQuaternion(map_.info.origin.orientation);
    const double dx = wx - map_.info.origin.position.x;
    const double dy = wy - map_.info.origin.position.y;
    const double local_x = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
    const double local_y = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
    *mx = static_cast<int>(std::floor(local_x / map_.info.resolution));
    *my = static_cast<int>(std::floor(local_y / map_.info.resolution));
    return *mx >= 0 && *my >= 0 && *mx < static_cast<int>(map_.info.width) &&
           *my < static_cast<int>(map_.info.height);
  }

  Point2D mapToWorld(int mx, int my) const {
    const double local_x = (mx + 0.5) * map_.info.resolution;
    const double local_y = (my + 0.5) * map_.info.resolution;
    const double origin_yaw = yawFromQuaternion(map_.info.origin.orientation);
    Point2D result;
    result.x = map_.info.origin.position.x +
               std::cos(origin_yaw) * local_x - std::sin(origin_yaw) * local_y;
    result.y = map_.info.origin.position.y +
               std::sin(origin_yaw) * local_x + std::cos(origin_yaw) * local_y;
    return result;
  }

  bool cellHasClearance(int mx, int my, double clearance) const {
    if (!have_map_) return false;
    const int radius = static_cast<int>(std::ceil(clearance / map_.info.resolution));
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy > radius * radius) continue;
        const int x = mx + dx;
        const int y = my + dy;
        if (x < 0 || y < 0 || x >= static_cast<int>(map_.info.width) ||
            y >= static_cast<int>(map_.info.height)) return false;
        const std::size_t index = static_cast<std::size_t>(y) * map_.info.width + x;
        if (index >= map_.data.size() || map_.data[index] != 0) return false;
      }
    }
    return true;
  }

  bool pointHasClearance(double wx, double wy, double clearance) const {
    int mx = 0;
    int my = 0;
    return worldToMap(wx, wy, &mx, &my) && cellHasClearance(mx, my, clearance);
  }

  void generateSearchWaypoints() {
    search_waypoints_.clear();
    search_index_ = 0;
    if (!have_map_ || map_.info.resolution <= 0.0) return;
    const int stride = std::max(1, static_cast<int>(std::round(
        search_spacing_ / map_.info.resolution)));
    const int margin = std::max(1, static_cast<int>(std::ceil(
        search_clearance_ / map_.info.resolution)));
    bool reverse = false;
    for (int my = margin; my < static_cast<int>(map_.info.height) - margin;
         my += stride) {
      if (!reverse) {
        for (int mx = margin; mx < static_cast<int>(map_.info.width) - margin;
             mx += stride) {
          if (cellHasClearance(mx, my, search_clearance_))
            search_waypoints_.push_back(mapToWorld(mx, my));
        }
      } else {
        for (int mx = static_cast<int>(map_.info.width) - 1 - margin;
             mx >= margin; mx -= stride) {
          if (cellHasClearance(mx, my, search_clearance_))
            search_waypoints_.push_back(mapToWorld(mx, my));
        }
      }
      reverse = !reverse;
      if (static_cast<int>(search_waypoints_.size()) >= max_search_waypoints_) break;
    }

    if (search_waypoints_.empty()) {
      ROS_ERROR("No safe free-space waypoint found in /map");
      return;
    }
    if (static_cast<int>(search_waypoints_.size()) > max_search_waypoints_)
      search_waypoints_.resize(max_search_waypoints_);

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (robotPose(&robot_x, &robot_y, &robot_yaw)) {
      const std::size_t first = maze_planner::searchStartIndex(
          search_waypoints_, robot_x, robot_y, robot_yaw,
          prefer_forward_search_ && course_waypoints_.empty());
      std::rotate(search_waypoints_.begin(), search_waypoints_.begin() + first,
                  search_waypoints_.end());
    }
    ROS_INFO("Generated %zu autonomous blue-target search waypoints",
             search_waypoints_.size());
  }

  void sendMoveBaseGoal(double x, double y, double yaw, GoalKind kind) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = map_frame_;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = quaternionFromYaw(yaw);
    move_base_.sendGoal(goal);
    goal_active_ = true;
    goal_kind_ = kind;
    goal_sent_time_ = ros::Time::now();
  }

  void sendNextSearchGoal() {
    setParkingTolerance(false);
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!robotPose(&robot_x, &robot_y, &robot_yaw)) return;
    Point2D target;
    current_goal_is_course_ = course_index_ < course_waypoints_.size();
    if (current_goal_is_course_) {
      // Advance only when the action finishes. A temporary camera, lidar, or
      // localization interruption cancels the goal and must retry this point.
      target = course_waypoints_[course_index_];
    } else {
      if (search_waypoints_.empty()) generateSearchWaypoints();
      if (search_waypoints_.empty()) {
        setState(State::FAILED, "map contains no safe search waypoint");
        return;
      }
      if (search_index_ >= search_waypoints_.size()) search_index_ = 0;
      target = search_waypoints_[search_index_++];
    }
    const double yaw = std::atan2(target.y - robot_y, target.x - robot_x);
    sendMoveBaseGoal(target.x, target.y, yaw, GoalKind::SEARCH);
    setState(State::SEARCH, current_goal_is_course_ ?
        "following mapped course toward the finish area" :
        "scanning map free space for blue A4 paper");
  }

  bool chooseApproachGoal(const TargetEstimate& target,
                          Point2D* position, double* yaw) {
    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    if (!robotPose(&robot_x, &robot_y, &robot_yaw)) return false;
    const double from_target_to_robot = std::atan2(robot_y - target.y,
                                                    robot_x - target.x);
    const double offsets_deg[] = {0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0};
    for (double offset_deg : offsets_deg) {
      const double angle = from_target_to_robot + offset_deg * M_PI / 180.0;
      const Point2D candidate{target.x + approach_standoff_ * std::cos(angle),
                              target.y + approach_standoff_ * std::sin(angle)};
      if (!pointHasClearance(candidate.x, candidate.y, approach_clearance_)) continue;
      *position = candidate;
      *yaw = std::atan2(target.y - candidate.y, target.x - candidate.x);
      return true;
    }
    return false;
  }

  bool sendApproachGoal(const TargetEstimate& target) {
    Point2D position;
    double yaw = 0.0;
    maze_planner::WallObservation wall;
    const bool use_visual = visual_tracking_parking_enabled_;
    const bool use_wall = !use_visual && wall_parking_enabled_ && targetWallInMap(&wall);
    if (use_visual) {
      double robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0, bearing = 0.0;
      if (!robotPose(&robot_x, &robot_y, &robot_yaw) || !targetBearingInBase(&bearing))
        return false;
      const double camera_map_x = robot_x + std::cos(robot_yaw)*camera_x_ -
                                  std::sin(robot_yaw)*camera_y_;
      const double camera_map_y = robot_y + std::sin(robot_yaw)*camera_x_ +
                                  std::cos(robot_yaw)*camera_y_;
      double target_x = camera_map_x + target.range * std::cos(robot_yaw + bearing);
      double target_y = camera_map_y + target.range * std::sin(robot_yaw + bearing);
      // Prefer the ray/wall intersection when the laser sees the target wall;
      // otherwise continue with the existing A4-height range estimate.
      if (targetWallInMap(&wall)) {
        target_x = wall.marker.x;
        target_y = wall.marker.y;
      }
      if (!maze_planner::visualApproachGoal(robot_x, robot_y, target_x, target_y,
                                           current_visual_standoff_, &position.x,
                                           &position.y, &yaw)) return false;
      if (!pointHasClearance(position.x, position.y, approach_clearance_)) {
        // A closer retry must not fall back to an older, farther goal. The
        // existing static clearance test still determines whether it can be planned.
        if (current_visual_standoff_ + 1e-6 < visual_approach_standoff_) {
          ROS_WARN_THROTTLE(3.0, "No closer visual approach pose passes static-map clearance");
          return false;
        }
        // Keep obstacle routing in move_base when the direct sight-line goal
        // is not available; the requested heading still points at the paper.
        if (!chooseApproachGoal(target, &position, &yaw)) return false;
        yaw = std::atan2(target_y - robot_y, target_x - robot_x);
      }
    } else if (use_wall) {
      const maze_planner::Point2 goal = maze_planner::parkingPosition(
          wall, wall_parking_standoff_);
      position.x = goal.x;
      position.y = goal.y;
      yaw = maze_planner::parkingYaw(wall);
      if (!pointHasClearance(position.x, position.y, approach_clearance_)) {
        ROS_WARN_THROTTLE(3.0, "Wall-normal parking pose fails static-map clearance");
        return false;
      }
    } else if (!chooseApproachGoal(target, &position, &yaw)) {
      ROS_WARN_THROTTLE(3.0, "No statically safe approach pose around estimated blue target");
      return false;
    }
    if (goal_active_ && goal_kind_ == GoalKind::APPROACH &&
        use_wall == current_goal_uses_wall_ && use_visual == current_goal_uses_visual_) {
      const double displacement = std::hypot(position.x - current_approach_goal_.x,
                                             position.y - current_approach_goal_.y);
      const double yaw_error = std::fabs(std::atan2(std::sin(yaw - current_approach_yaw_),
                                                    std::cos(yaw - current_approach_yaw_)));
      if (use_visual && displacement < 0.10 && yaw_error < 0.08) return true;
      if (use_wall && displacement < 0.06 && yaw_error < 0.08) return true;
      if (!use_visual && !use_wall && displacement < goal_update_distance_ &&
          (ros::Time::now() - last_approach_update_).toSec() < goal_update_period_) return true;
    }

    if (goal_active_) move_base_.cancelGoal();
    setParkingTolerance(use_wall || use_visual);
    current_approach_goal_ = position;
    current_approach_yaw_ = yaw;
    current_goal_uses_wall_ = use_wall;
    current_goal_uses_visual_ = use_visual;
    last_approach_update_ = ros::Time::now();
    sendMoveBaseGoal(position.x, position.y, yaw, GoalKind::APPROACH);

    geometry_msgs::PoseStamped published;
    published.header.stamp = ros::Time::now();
    published.header.frame_id = map_frame_;
    published.pose.position.x = position.x;
    published.pose.position.y = position.y;
    published.pose.orientation = quaternionFromYaw(yaw);
    approach_goal_pub_.publish(published);
    setState(State::APPROACH, use_visual ?
        "following blue target direction using gimbal and image" : use_wall ?
        "wall and blue target localized; navigating to wall-normal parking pose" :
        "stable blue target localized; navigating to coarse approach pose");
    return true;
  }

  void enterLineScan(const std::string& reason) {
    if (goal_active_) move_base_.cancelGoal();
    goal_active_ = false;
    goal_kind_ = GoalKind::NONE;
    line_scan_start_time_ = ros::Time::now();
    setState(State::LINE_SCAN, reason);
  }

  void blackLineCb(const stm32_bridge::BlackLineObservation::ConstPtr& msg) {
    last_black_line_time_ = ros::Time::now();
    black_line_visible_ = msg->detected;
    if (!line_guidance_enabled_ || !enabled_ || !msg->detected || target_visible_) return;
    if (state_ != State::SEARCH && state_ != State::OBSERVE) return;
    if ((ros::Time::now() - last_line_scan_finish_time_).toSec() <
        line_retrigger_cooldown_) return;
    enterLineScan("black finish line detected; base stopped for horizontal gimbal scan");
  }

  void finishMission() {
    if (goal_active_) move_base_.cancelGoal();
    goal_active_ = false;
    goal_kind_ = GoalKind::NONE;
    setParkingTolerance(false);
    publishGoalReached(true);
    setState(State::COMPLETE, visual_tracking_parking_enabled_ ?
        "blue target area and facing thresholds reached; base stop latched" : wall_parking_enabled_ ?
        "estimated footprint inside finish area; base stop latched for judge confirmation" :
        "approach goal reached; base stop latched for judge confirmation");
  }

  bool shouldHoldAtCourseEnd() const {
    return hold_at_course_end_ && !course_waypoints_.empty() &&
           course_index_ >= course_waypoints_.size();
  }

  void finishObservationCb(const geometry_msgs::PoseStamped::ConstPtr& message) {
    const double age = (ros::Time::now() - message->header.stamp).toSec();
    if (!enabled_ || state_ != State::OBSERVE || !shouldHoldAtCourseEnd() ||
        goal_active_ || finish_observation_attempted_ || message->header.frame_id != base_frame_ ||
        message->header.stamp.isZero() || age < 0.0 || age > .5) return;

    std::string reason;
    if (!systemReady(&reason)) return;
    double x = 0.0, y = 0.0, yaw = 0.0;
    if (!robotPose(&x, &y, &yaw)) return;
    const maze_planner::Point2 relative(message->pose.position.x, message->pose.position.y);
    const auto goal = maze_planner::Point2(x, y) + maze_planner::rotate(relative, yaw);
    const double heading = yaw + yawFromQuaternion(message->pose.orientation);
    if (!std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(heading)) return;

    const maze_planner::Point2 toward_wall(std::cos(heading), std::sin(heading));
    const double extra_standoffs[] = {0.0, .10, .20, .30, .40};
    for (double extra : extra_standoffs) {
      const auto candidate = goal - extra * toward_wall;
      if (std::hypot(candidate.x-x, candidate.y-y) < .20 ||
          !pointHasClearance(candidate.x, candidate.y, approach_clearance_)) continue;
      finish_observation_attempted_ = true;
      sendMoveBaseGoal(candidate.x, candidate.y, heading, GoalKind::SEARCH);
      setState(State::OBSERVE, "navigating to a closer finish observation pose");
      return;
    }
    ROS_WARN_THROTTLE(3.0, "No aligned finish observation pose has enough map clearance");
  }

  void handleSearchState() {
    if (target_visible_) return;
    if (!goal_active_) {
      sendNextSearchGoal();
      return;
    }
    const actionlib::SimpleClientGoalState action_state = move_base_.getState();
    const bool timeout = (ros::Time::now() - goal_sent_time_).toSec() > search_goal_timeout_;
    if (action_state.isDone() || timeout) {
      if (timeout) move_base_.cancelGoal();
      const bool was_course_goal = current_goal_is_course_;
      goal_active_ = false;
      goal_kind_ = GoalKind::NONE;
      if (was_course_goal) ++course_index_;
      if (!timeout && action_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        if (was_course_goal && course_index_ < course_waypoints_.size()) {
          sendNextSearchGoal();
        } else {
          waypoint_observation_start_time_ = ros::Time::now();
          setState(State::OBSERVE, was_course_goal && shouldHoldAtCourseEnd() ?
              "mapped course end reached; scanning for blue marker" :
              "search waypoint reached; observing for blue paper and black line");
        }
      } else if (was_course_goal && course_index_ >= course_waypoints_.size()) {
        setState(State::FAILED, "mapped course end could not be reached");
      } else {
        sendNextSearchGoal();
      }
    }
  }

  void handleObserveState() {
    if (target_visible_) return;
    // The final recorded point is in the finish area. Keep scanning there;
    // generic free-space waypoints can lead back through the course.
    if (shouldHoldAtCourseEnd()) {
      if (!goal_active_) return;
      const auto action_state = move_base_.getState();
      const bool timeout = (ros::Time::now() - goal_sent_time_).toSec() > search_goal_timeout_;
      if (!action_state.isDone() && !timeout) return;
      if (timeout) move_base_.cancelGoal();
      goal_active_ = false;
      goal_kind_ = GoalKind::NONE;
      setState(State::OBSERVE, !timeout && action_state == actionlib::SimpleClientGoalState::SUCCEEDED ?
          "closer observation pose reached; scanning for blue marker" :
          "closer observation pose unreachable; waiting for blue marker");
      return;
    }
    const bool line_fresh = black_line_visible_ &&
        (ros::Time::now() - last_black_line_time_).toSec() <= black_line_timeout_;
    if (line_guidance_enabled_ && line_fresh &&
        (ros::Time::now() - last_line_scan_finish_time_).toSec() >=
            line_retrigger_cooldown_) {
      enterLineScan("black finish line confirmed at observation waypoint");
      return;
    }
    if ((ros::Time::now() - waypoint_observation_start_time_).toSec() >=
        waypoint_observation_time_) {
      setState(State::SEARCH, "no target at observation waypoint; continuing map fallback search");
      sendNextSearchGoal();
    }
  }

  void handleLineScanState() {
    if (target_visible_) return;
    if ((ros::Time::now() - line_scan_start_time_).toSec() < line_scan_hold_time_) return;
    last_line_scan_finish_time_ = ros::Time::now();
    black_line_visible_ = false;
    setState(State::SEARCH, "black-line scan timed out; resuming map fallback search");
    sendNextSearchGoal();
  }

  void handleAcquireState() {
    if ((ros::Time::now() - last_target_seen_time_).toSec() > target_lost_timeout_) {
      estimates_.clear();
      if (search_enabled_) {
        setState(State::SEARCH, "blue target lost before localization; resuming search");
        sendNextSearchGoal();
      } else {
        setState(State::WAIT_SYSTEM, "blue target lost; waiting in place");
      }
      return;
    }

    TargetEstimate stable;
    double stddev = 0.0;
    if (stableTarget(&stable, &stddev)) {
      if (visual_tracking_parking_enabled_ && visualParkingReached()) {
        finishMission();
        return;
      }
      if (!visual_tracking_parking_enabled_ && wall_parking_enabled_ &&
          robotInsideDetectedFinish()) {
        finishMission();
        return;
      }
      if (!sendApproachGoal(stable)) {
        setState(State::ACQUIRE, "target stable but no collision-free approach pose");
      }
    }
  }

  bool robotInsideDetectedFinish() {
    maze_planner::WallObservation wall;
    double x = 0.0, y = 0.0, yaw = 0.0;
    return targetWallInMap(&wall) && robotPose(&x, &y, &yaw) &&
           maze_planner::footprintInsideFinish(wall, maze_planner::Point2(x, y), yaw,
                                              footprint_);
  }

  void handleApproachState() {
    if (!goal_active_ || goal_kind_ != GoalKind::APPROACH) return;
    // The competition scores the whole vehicle inside the square, not its
    // distance from one nominal pose. Stop as soon as the currently observed
    // marker/wall and footprint establish that geometric condition.
    if (visual_tracking_parking_enabled_ && visualParkingReached()) {
      finishMission();
      return;
    }
    if (!visual_tracking_parking_enabled_ && wall_parking_enabled_ && current_goal_uses_wall_) {
      if (robotInsideDetectedFinish()) {
        finishMission();
        return;
      }
    }
    const actionlib::SimpleClientGoalState action_state = move_base_.getState();
    if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      if (visual_tracking_parking_enabled_) {
        goal_active_ = false;
        goal_kind_ = GoalKind::NONE;
        estimates_.clear();
        current_visual_standoff_ = std::max(0.0, current_visual_standoff_ - visual_approach_step_);
        setState(State::ACQUIRE, "approach goal reached; visual thresholds unmet, planning closer");
        return;
      }
      if (wall_parking_enabled_) {
        maze_planner::WallObservation wall;
        double x = 0.0, y = 0.0, yaw = 0.0;
        const bool have_wall = current_goal_uses_wall_ && targetWallInMap(&wall);
        const bool have_pose = robotPose(&x, &y, &yaw);
        if (!have_wall || !have_pose ||
            !maze_planner::footprintInsideFinish(wall, maze_planner::Point2(x, y), yaw,
                                                footprint_)) {
          goal_active_ = false;
          goal_kind_ = GoalKind::NONE;
          estimates_.clear();
          setState(State::ACQUIRE, !have_wall ?
              "coarse goal reached; observing blue target and wall for parking" :
              "parking pose reached; refining estimated footprint and alignment");
          return;
        }
      }
      finishMission();
      return;
    }
    if (action_state.isDone()) {
      goal_active_ = false;
      goal_kind_ = GoalKind::NONE;
      if ((ros::Time::now() - last_target_seen_time_).toSec() <= target_lost_timeout_) {
        estimates_.clear();
        setState(State::ACQUIRE, "approach plan failed; re-estimating target");
      } else if (search_enabled_) {
        setState(State::SEARCH, "approach failed and target lost; resuming search");
        sendNextSearchGoal();
      } else {
        setState(State::FAILED, "move_base could not reach blue target approach pose");
      }
      return;
    }

    TargetEstimate stable;
    double stddev = 0.0;
    const double update_period = visual_tracking_parking_enabled_ ?
        visual_goal_update_period_ : goal_update_period_;
    if (stableTarget(&stable, &stddev) &&
        (ros::Time::now() - last_approach_update_).toSec() >= update_period) {
      sendApproachGoal(stable);
    }
  }

  void timerCb(const ros::TimerEvent&) {
    if (!enabled_ || state_ == State::IDLE || state_ == State::COMPLETE ||
        state_ == State::FAILED) return;
    if ((ros::Time::now() - mission_enable_time_).toSec() < start_delay_) {
      setState(State::WAIT_SYSTEM, "start delay active");
      return;
    }

    std::string not_ready_reason;
    if (!systemReady(&not_ready_reason)) {
      if (goal_active_) move_base_.cancelGoal();
      goal_active_ = false;
      goal_kind_ = GoalKind::NONE;
      setState(State::WAIT_SYSTEM, not_ready_reason);
      return;
    }

    if (state_ == State::WAIT_SYSTEM) {
      if (target_visible_ && target_pose_valid_) {
        setState(State::ACQUIRE, "blue target already visible");
      } else if (shouldHoldAtCourseEnd()) {
        setState(State::OBSERVE, "mapped course end reached; scanning for blue marker");
      } else if (line_guidance_enabled_ && black_line_visible_ &&
                 (ros::Time::now() - last_black_line_time_).toSec() <=
                     black_line_timeout_) {
        enterLineScan("black finish line visible at mission start");
      } else if (search_enabled_) {
        generateSearchWaypoints();
        sendNextSearchGoal();
      } else {
        setState(State::ACQUIRE, "waiting in place for blue target");
      }
      return;
    }
    if (state_ == State::SEARCH) handleSearchState();
    else if (state_ == State::OBSERVE) handleObserveState();
    else if (state_ == State::LINE_SCAN) handleLineScanState();
    else if (state_ == State::ACQUIRE) handleAcquireState();
    else if (state_ == State::APPROACH) handleApproachState();
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  MoveBaseClient move_base_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber observation_sub_;
  ros::Subscriber black_line_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber amcl_sub_;
  ros::Subscriber joint_sub_;
  ros::Subscriber gimbal_connected_sub_;
  ros::Subscriber gimbal_ready_sub_;
  ros::Subscriber gimbal_locked_sub_;
  ros::Subscriber enable_sub_;
  ros::Subscriber finish_observation_sub_;
  ros::Publisher state_pub_;
  ros::Publisher target_pose_pub_;
  ros::Publisher approach_goal_pub_;
  ros::Publisher goal_reached_pub_;
  ros::ServiceClient tolerance_client_;
  ros::Timer timer_;

  bool auto_start_{false};
  double start_delay_{5.0};
  bool search_enabled_{true};
  bool require_localization_{true};
  bool require_gimbal_{false};
  bool require_camera_info_{false};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_footprint"};
  double camera_x_{0.145};
  double camera_y_{0.0};
  double paper_height_{0.297};
  double fallback_hfov_deg_{70.0};
  double min_pixel_height_{18.0};
  double min_target_range_{0.45};
  double max_target_range_{8.0};
  int target_sample_count_{10};
  double target_sample_max_age_{1.5};
  double target_max_stddev_{0.30};
  double target_lost_timeout_{1.5};
  double approach_standoff_{0.65};
  double approach_clearance_{0.46};
  double goal_update_distance_{0.25};
  double goal_update_period_{2.0};
  double search_spacing_{1.5};
  double search_clearance_{0.50};
  double search_goal_timeout_{30.0};
  int max_search_waypoints_{250};
  bool line_guidance_enabled_{true};
  bool prefer_forward_search_{false};
  bool acquire_target_in_navigation_{true};
  bool hold_at_course_end_{false};
  double waypoint_observation_time_{3.0};
  double line_scan_hold_time_{10.0};
  double line_retrigger_cooldown_{12.0};
  double black_line_timeout_{0.6};
  bool wall_parking_enabled_{false};
  double wall_parking_standoff_{0.56};
  double wall_parking_xy_tolerance_{0.04};
  double wall_parking_yaw_tolerance_{0.08};
  bool visual_tracking_parking_enabled_{false};
  double visual_stop_area_ratio_{0.16};
  double visual_facing_tolerance_deg_{5.0};
  double visual_approach_standoff_{0.48};
  double visual_approach_step_{0.05};
  double visual_goal_xy_tolerance_{0.04};
  double visual_goal_yaw_tolerance_{0.08};
  double current_visual_standoff_{0.48};
  double visual_goal_update_period_{1.0};
  double previous_xy_tolerance_{0.12};
  double previous_yaw_tolerance_{0.12};
  double max_localization_position_stddev_{0.50};
  double max_localization_yaw_stddev_{0.50};

  State state_{State::IDLE};
  GoalKind goal_kind_{GoalKind::NONE};
  std::string state_reason_;
  bool enabled_{false};
  bool goal_active_{false};
  bool target_visible_{false};
  bool target_pose_valid_{false};
  bool black_line_visible_{false};
  bool have_map_{false};
  bool have_scan_{false};
  bool have_target_observation_{false};
  bool parking_tolerance_active_{false};
  bool current_goal_uses_wall_{false};
  bool current_goal_uses_visual_{false};
  bool have_camera_info_{false};
  bool have_amcl_{false};
  bool localization_good_{false};
  bool gimbal_connected_{false};
  bool gimbal_ready_{false};
  bool gimbal_locked_{false};
  double localization_position_stddev_{std::numeric_limits<double>::infinity()};
  double localization_yaw_stddev_{std::numeric_limits<double>::infinity()};
  double pan_angle_{0.0};
  double tilt_angle_{0.0};
  nav_msgs::OccupancyGrid map_;
  sensor_msgs::CameraInfo camera_info_;
  sensor_msgs::LaserScan latest_scan_;
  stm32_bridge::BlueTargetObservation latest_observation_;
  std::vector<maze_planner::Point2> footprint_;
  std::deque<TargetEstimate> estimates_;
  TargetEstimate latest_target_;
  std::vector<Point2D> search_waypoints_;
  std::vector<Point2D> course_waypoints_;
  std::size_t course_index_{0};
  bool finish_observation_attempted_{false};
  bool current_goal_is_course_{false};
  std::size_t search_index_{0};
  Point2D current_approach_goal_;
  double current_approach_yaw_{0.0};
  ros::Time node_start_time_;
  ros::Time mission_enable_time_;
  ros::Time last_observation_time_;
  ros::Time last_scan_time_;
  ros::Time last_target_seen_time_;
  ros::Time last_joint_time_;
  ros::Time goal_sent_time_;
  ros::Time last_approach_update_;
  ros::Time last_black_line_time_;
  ros::Time waypoint_observation_start_time_;
  ros::Time line_scan_start_time_;
  ros::Time last_line_scan_finish_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "blue_goal_navigator");
  BlueGoalNavigator node;
  ros::spin();
  return 0;
}
