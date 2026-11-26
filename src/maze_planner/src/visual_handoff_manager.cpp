#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <stm32_bridge/BlueTargetObservation.h>
#include <stm32_bridge/footprint_parameters.hpp>
#include <maze_planner/handoff_corridor.h>
#include <maze_planner/gimbal_history.h>
#include <maze_planner/visual_tracking.h>
#include <maze_planner/wall_parking.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

class VisualHandoffManager {
 public:
  VisualHandoffManager() : nh_(), pnh_("~") {
    pnh_.param<bool>("auto_start", auto_start_, false);
    pnh_.param<bool>("competition_mode", competition_mode_, false);
    pnh_.param<double>("max_handoff_distance", max_handoff_distance_, 1.30);
    pnh_.param("max_handoff_lateral", max_handoff_lateral_, .38);
    pnh_.param("max_handoff_yaw_rad", max_handoff_yaw_rad_, .40);
    pnh_.param("entrance_max_handoff_lateral", entrance_max_handoff_lateral_, .75);
    pnh_.param("entrance_max_handoff_yaw_rad", entrance_max_handoff_yaw_rad_, .65);
    pnh_.param("approach_max_speed", approach_max_speed_, .35);
    pnh_.param("approach_max_turn", approach_max_turn_, .40);
    approach_max_speed_ = std::max(0.0, approach_max_speed_);
    approach_max_turn_ = std::max(0.0, approach_max_turn_);
    pnh_.param<int>("required_clear_scans", required_clear_scans_, 3);
    pnh_.param<double>("camera_x", camera_x_, .145);
    pnh_.param<double>("camera_y", camera_y_, 0.0);
    pnh_.param<double>("paper_height", paper_height_, .297);
    pnh_.param<double>("fallback_hfov_deg", fallback_hfov_deg_, 88.0);
    pnh_.param<double>("max_target_range", max_target_range_, 8.0);
    pnh_.param<double>("range_absolute_tolerance", range_absolute_tolerance_, .25);
    pnh_.param<double>("range_relative_tolerance", range_relative_tolerance_, .20);
    pnh_.param<double>("scan_timeout", scan_timeout_, .5);
    pnh_.param<double>("observation_timeout", observation_timeout_, .5);
    pnh_.param<double>("joint_timeout", joint_timeout_, 1.5);
    pnh_.param<double>("max_sensor_skew", max_sensor_skew_, .5);
    pnh_.param<double>("laser_x", corridor_params_.laser_x, .315);
    pnh_.param<double>("laser_y", corridor_params_.laser_y, 0.0);
    pnh_.param<double>("laser_yaw", corridor_params_.laser_yaw, 0.0);
    pnh_.param<double>("corridor_check_standoff", corridor_params_.check_standoff, .48);
    pnh_.param<double>("corridor_sample_step", corridor_params_.sample_step, .10);
    pnh_.param<double>("corridor_ray_margin", corridor_params_.ray_margin, .04);
    pnh_.param<double>("turn_check_rate", corridor_params_.turn_rate, .25);
    required_clear_scans_ = std::max(1, required_clear_scans_);
    max_target_range_ = std::max(.5, max_target_range_);
    range_absolute_tolerance_ = std::max(0.0, range_absolute_tolerance_);
    range_relative_tolerance_ = std::max(0.0, range_relative_tolerance_);

    ros::NodeHandle geometry_nh("/robot_geometry");
    footprint_.padding = 0.0;
    if (!stm32_bridge::readFootprintParam(geometry_nh, &footprint_)) return;
    self_footprint_ = footprint_;
    footprint_.padding = .03;

    command_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    nav_enable_pub_ = nh_.advertise<std_msgs::Bool>("/handoff/nav_enable", 1, true);
    visual_enable_pub_ = nh_.advertise<std_msgs::Bool>("/handoff/visual_enable", 1, true);
    goal_pub_ = nh_.advertise<std_msgs::Bool>("/goal_reached", 1, true);
    state_pub_ = nh_.advertise<std_msgs::String>("/blue_autonomy/state", 1, true);
    enable_sub_ = nh_.subscribe("/blue_autonomy/enable", 2, &VisualHandoffManager::enableCb, this);
    nav_cmd_sub_ = nh_.subscribe("/handoff/cmd_vel_nav", 2, &VisualHandoffManager::navCmdCb, this);
    visual_cmd_sub_ = nh_.subscribe("/handoff/cmd_vel_visual", 2, &VisualHandoffManager::visualCmdCb, this);
    nav_state_sub_ = nh_.subscribe("/handoff/nav_state", 3, &VisualHandoffManager::navStateCb, this);
    visual_state_sub_ = nh_.subscribe("/handoff/visual_state", 3, &VisualHandoffManager::visualStateCb, this);
    visual_goal_sub_ = nh_.subscribe("/handoff/visual_goal_reached", 2,
                                      &VisualHandoffManager::visualGoalCb, this);
    observation_sub_ = nh_.subscribe("/blue_target/observation", 3,
                                      &VisualHandoffManager::observationCb, this);
    joint_sub_ = nh_.subscribe("/gimbal/joint_states", 3, &VisualHandoffManager::jointCb, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &VisualHandoffManager::scanCb, this);
    camera_sub_ = nh_.subscribe("/usb_cam/camera_info", 2, &VisualHandoffManager::cameraCb, this);
    publishBool(nav_enable_pub_, false);
    publishBool(visual_enable_pub_, false);
    publishBool(goal_pub_, false);
    publishState("IDLE: automatic handoff disabled");
    if (auto_start_) startNavigation();
    timer_ = nh_.createTimer(ros::Duration(.05), &VisualHandoffManager::timerCb, this);
  }

 private:
  enum class Mode { IDLE, NAVIGATION, VISUAL, COMPLETE };

  void publishBool(ros::Publisher& pub, bool value) {
    std_msgs::Bool msg;
    msg.data = value;
    pub.publish(msg);
  }
  void publishStop() { command_pub_.publish(geometry_msgs::Twist()); }
  void publishState(const std::string& state) {
    if (state == state_) return;
    state_ = state;
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
    ROS_INFO("Visual handoff: %s", state.c_str());
  }
  void startNavigation() {
    mode_ = Mode::NAVIGATION;
    clear_scans_ = 0;
    nav_state_.clear();
    nav_wait_since_ = ros::Time();
    visual_state_.clear();
    visual_blocked_since_ = ros::Time();
    race_start_ = ros::Time::now();
    recheck_after_ = ros::Time();
    checked_scan_stamp_ = ros::Time(0);
    publishStop();
    publishBool(goal_pub_, false);
    publishBool(visual_enable_pub_, false);
    publishBool(nav_enable_pub_, true);
    publishState("EXPLORE: waiting for navigation and target");
  }
  void enableCb(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
      if (mode_ == Mode::IDLE || mode_ == Mode::COMPLETE) startNavigation();
      return;
    }
    mode_ = Mode::IDLE;
    clear_scans_ = 0;
    publishStop();
    publishBool(nav_enable_pub_, false);
    publishBool(visual_enable_pub_, false);
    publishBool(goal_pub_, false);
    publishState("IDLE: automatic handoff disabled");
  }
  void navCmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    if (mode_ != Mode::NAVIGATION) return;
    geometry_msgs::Twist out = *msg;
    if (nav_state_.find("APPROACH:") == 0) {
      const double v = std::abs(out.linear.x), w = std::abs(out.angular.z);
      const double vs = v > approach_max_speed_ ? approach_max_speed_ / v : 1.0;
      const double ws = w > approach_max_turn_ ? approach_max_turn_ / w : 1.0;
      out.linear.x *= std::min(vs, ws);
      out.angular.z *= std::min(vs, ws);
    }
    command_pub_.publish(out);
  }
  void visualCmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    if (mode_ == Mode::VISUAL) command_pub_.publish(*msg);
  }
  void navStateCb(const std_msgs::String::ConstPtr& msg) {
    nav_state_ = msg->data;
    if (nav_state_.find("APPROACH: waiting for a clear") != 0)
      nav_wait_since_ = ros::Time();
    else if (nav_wait_since_.isZero())
      nav_wait_since_ = ros::Time::now();
    if (mode_ == Mode::NAVIGATION && nav_state_.find("IDLE:") != 0)
      publishState((nav_state_.find("APPROACH:") == 0 ? "TARGET_APPROACH: " : "EXPLORE: ") +
                   nav_state_);
  }
  void visualStateCb(const std_msgs::String::ConstPtr& msg) {
    visual_state_ = msg->data;
    if (mode_ == Mode::VISUAL) publishState("TERMINAL_PARK: " + msg->data);
  }
  void visualGoalCb(const std_msgs::Bool::ConstPtr& msg) {
    if (mode_ != Mode::VISUAL || !msg->data) return;
    mode_ = Mode::COMPLETE;
    publishStop();
    publishBool(goal_pub_, true);
    publishState("COMPLETE: visual parking confirmed");
  }
  void observationCb(const stm32_bridge::BlueTargetObservation::ConstPtr& msg) {
    observation_ = *msg;
    observation_received_ = ros::Time::now();
    have_observation_ = true;
  }
  void jointCb(const sensor_msgs::JointState::ConstPtr& msg) {
    gimbal_history_.add(*msg);
    joint_received_ = ros::Time::now();
  }
  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
    scan_ = *msg;
    scan_received_ = ros::Time::now();
  }
  void cameraCb(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    camera_info_ = *msg;
    have_camera_info_ = msg->width > 0 && msg->height > 0 &&
        std::isfinite(msg->K[0]) && std::isfinite(msg->K[4]) &&
        msg->K[0] > 1.0 && msg->K[4] > 1.0;
  }

  bool targetGeometry(double* bearing, double* horizontal_range) const {
    if (!observation_.detected || !observation_.image_width || !observation_.image_height ||
        observation_.pixel_height <= 0.0) return false;
    double pan = 0.0, tilt = 0.0;
    if (!gimbal_history_.angles(observation_.header.stamp, &pan, &tilt, max_sensor_skew_))
      return false;
    double fx, fy;
    double cx = .5*observation_.image_width, cy = .5*observation_.image_height;
    if (have_camera_info_) {
      const double sx = static_cast<double>(observation_.image_width)/camera_info_.width;
      const double sy = static_cast<double>(observation_.image_height)/camera_info_.height;
      fx = camera_info_.K[0]*sx; fy = camera_info_.K[4]*sy;
      cx = camera_info_.K[2]*sx; cy = camera_info_.K[5]*sy;
    } else {
      const double hfov = std::max(20.0, std::min(150.0, fallback_hfov_deg_))*M_PI/180.0;
      fx = observation_.image_width/(2.0*std::tan(.5*hfov));
      fy = fx;
    }
    if (!maze_planner::visualBearing(pan, tilt, observation_.center_x,
                                      observation_.center_y, cx, cy, fx, fy, bearing)) return false;
    const double down = tilt+std::atan2(observation_.center_y-cy, fy);
    *horizontal_range = fy*paper_height_/observation_.pixel_height*std::cos(down);
    return std::isfinite(*horizontal_range) && *horizontal_range > 0.0 &&
           *horizontal_range <= max_target_range_;
  }

  bool corridorReady(std::string* reason) {
    const ros::Time now = ros::Time::now();
    if (!have_observation_ || (now-observation_received_).toSec() > observation_timeout_ ||
        (now-joint_received_).toSec() > joint_timeout_ ||
        (now-scan_received_).toSec() > scan_timeout_) {
      *reason = "waiting for camera, gimbal or lidar";
      return false;
    }
    if (observation_.header.stamp < race_start_ || scan_.header.stamp.isZero() ||
        std::fabs((observation_.header.stamp-scan_.header.stamp).toSec()) > max_sensor_skew_) {
      *reason = "camera, gimbal and lidar timestamps do not agree";
      return false;
    }
    double bearing = 0.0, visual_range = 0.0;
    if (!targetGeometry(&bearing, &visual_range)) {
      *reason = "blue marker not localized in view";
      return false;
    }

    std::vector<maze_planner::Point2> points;
    std::vector<stm32_bridge::SafetyPoint> obstacles;
    for (std::size_t i = 0; i < scan_.ranges.size(); ++i) {
      const double r = scan_.ranges[i];
      if (!std::isfinite(r) || r <= scan_.range_min || r >= scan_.range_max ||
          r > max_target_range_) continue;
      const double angle = scan_.angle_min+i*scan_.angle_increment+corridor_params_.laser_yaw;
      const stm32_bridge::SafetyPoint point{corridor_params_.laser_x+r*std::cos(angle),
                                             corridor_params_.laser_y+r*std::sin(angle)};
      if (stm32_bridge::pointInsideFootprint(point, {0.0,0.0}, 0.0, self_footprint_)) continue;
      points.emplace_back(point.x, point.y);
      obstacles.push_back(point);
    }
    maze_planner::WallObservation wall;
    const double tolerance = std::max(range_absolute_tolerance_,
                                       range_relative_tolerance_ * visual_range);
    if (!maze_planner::fitTargetWall(points, {camera_x_, camera_y_}, bearing, &wall,
                                      max_target_range_, visual_range, tolerance)) {
      *reason = "no lidar surface associated with blue marker";
      return false;
    }
    const maze_planner::Point2 tangent(-wall.outward.y, wall.outward.x);
    const double lateral = std::abs(maze_planner::dot(wall.marker, tangent));
    const double depth = -maze_planner::dot(wall.marker, wall.outward);
    const double range = maze_planner::norm(wall.marker);
    const double yaw = std::abs(maze_planner::parkingYaw(wall));
    const bool close_and_aligned = range <= max_handoff_distance_ &&
        lateral <= max_handoff_lateral_ && (!competition_mode_ || yaw <= max_handoff_yaw_rad_);
    const bool arrived = nav_state_.find("APPROACH: entrance reached") == 0;
    const bool waited = !nav_wait_since_.isZero() && (now - nav_wait_since_).toSec() >= 1.5;
    const bool entrance_or_stalled_close = (arrived || waited) && depth <= 1.15 &&
        range <= max_handoff_distance_ && lateral <= entrance_max_handoff_lateral_ &&
        (!competition_mode_ || yaw <= entrance_max_handoff_yaw_rad_);
    std::ostringstream geometry;
    geometry << " (norm=" << range << ", depth=" << depth << ", lat=" << lateral
             << ", yaw=" << yaw << ")";
    if (!close_and_aligned && !entrance_or_stalled_close) {
      *reason = "outside the short-range terminal corridor";
      *reason += geometry.str();
      return false;
    }
    // The same swept-body and observed-ray check also governs the entrance
    // fallback. Arriving at a navigation goal never proves the corridor clear.
    const bool clear = maze_planner::handoffCorridorClear(scan_, obstacles, footprint_,
        wall.marker.x, wall.marker.y, corridor_params_, reason);
    if (!clear) *reason += geometry.str();
    return clear;
  }

  void switchToVisual(const std::string& reason) {
    // The arbiter stops forwarding navigation before enabling visual control.
    mode_ = Mode::VISUAL;
    publishStop();
    if (!competition_mode_) publishBool(nav_enable_pub_, false);
    visual_state_.clear();
    visual_blocked_since_ = ros::Time();
    publishBool(visual_enable_pub_, true);
    publishState("TERMINAL_PARK: " + reason);
  }

  void timerCb(const ros::TimerEvent&) {
    if (mode_ == Mode::IDLE || mode_ == Mode::COMPLETE) {
      publishStop();
      if (mode_ == Mode::COMPLETE) publishBool(goal_pub_, true);
      return;
    }
    const auto now = ros::Time::now();
    if (mode_ == Mode::VISUAL) {
      const bool blocked = visual_state_.find("BLOCKED") == 0;
      if (!blocked) visual_blocked_since_ = ros::Time();
      else if (visual_blocked_since_.isZero()) visual_blocked_since_ = now;
      else if (competition_mode_ && (now - visual_blocked_since_).toSec() >= .4) {
        publishStop();
        mode_ = Mode::NAVIGATION;
        clear_scans_ = 0;
        checked_scan_stamp_ = scan_.header.stamp;
        recheck_after_ = now + ros::Duration(1.0);
        publishBool(visual_enable_pub_, false);
        publishState("TARGET_APPROACH: terminal corridor blocked; returning to move_base");
      }
      return;
    }
    if ((now - scan_received_).toSec() > scan_timeout_ ||
        (competition_mode_ && nav_state_.find("APPROACH:") != 0) ||
        now < recheck_after_) {
      clear_scans_ = 0;
      return;
    }
    if (scan_.header.stamp == checked_scan_stamp_) return;
    checked_scan_stamp_ = scan_.header.stamp;
    std::string reason;
    if (!corridorReady(&reason)) {
      clear_scans_ = 0;
      if (nav_state_.find("APPROACH:") == 0)
        ROS_INFO_THROTTLE(2.0, "Terminal handoff waiting: %s", reason.c_str());
      else
        ROS_DEBUG_THROTTLE(1.0, "Terminal handoff waiting: %s", reason.c_str());
      return;
    }
    if (++clear_scans_ >= required_clear_scans_)
      switchToVisual("lidar corridor clear; final parking enabled");
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher command_pub_, nav_enable_pub_, visual_enable_pub_, goal_pub_, state_pub_;
  ros::Subscriber enable_sub_, nav_cmd_sub_, visual_cmd_sub_, nav_state_sub_, visual_state_sub_;
  ros::Subscriber visual_goal_sub_, observation_sub_, joint_sub_, scan_sub_, camera_sub_;
  ros::Timer timer_;
  maze_planner::HandoffCorridorParameters corridor_params_;
  stm32_bridge::SafetyFootprint footprint_, self_footprint_;
  stm32_bridge::BlueTargetObservation observation_;
  sensor_msgs::LaserScan scan_;
  sensor_msgs::CameraInfo camera_info_;
  ros::Time observation_received_, joint_received_, scan_received_, checked_scan_stamp_;
  ros::Time race_start_, visual_blocked_since_, recheck_after_;
  ros::Time nav_wait_since_;
  Mode mode_{Mode::IDLE};
  bool auto_start_{false}, competition_mode_{false};
  bool have_observation_{false}, have_camera_info_{false};
  int required_clear_scans_{3}, clear_scans_{0};
  double max_handoff_distance_{1.30};
  double max_handoff_lateral_{.38}, max_handoff_yaw_rad_{.40};
  double entrance_max_handoff_lateral_{.75}, entrance_max_handoff_yaw_rad_{.65};
  double approach_max_speed_{.35}, approach_max_turn_{.40};
  maze_planner::GimbalHistory gimbal_history_;
  double camera_x_{.145}, camera_y_{0.0}, paper_height_{.297}, fallback_hfov_deg_{88.0};
  double max_target_range_{8.0}, range_absolute_tolerance_{.25}, range_relative_tolerance_{.20};
  double scan_timeout_{.5}, observation_timeout_{.5}, joint_timeout_{1.5}, max_sensor_skew_{.5};
  std::string state_, nav_state_, visual_state_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_handoff_manager");
  VisualHandoffManager node;
  ros::spin();
  return 0;
}
