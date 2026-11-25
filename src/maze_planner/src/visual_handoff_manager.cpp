#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <stm32_bridge/BlueTargetObservation.h>
#include <stm32_bridge/footprint_parameters.hpp>
#include <maze_planner/handoff_corridor.h>
#include <maze_planner/visual_tracking.h>
#include <maze_planner/wall_parking.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

class VisualHandoffManager {
 public:
  VisualHandoffManager() : nh_(), pnh_("~") {
    pnh_.param<bool>("auto_start", auto_start_, false);
    pnh_.param<int>("required_clear_scans", required_clear_scans_, 3);
    pnh_.param<double>("camera_x", camera_x_, .145);
    pnh_.param<double>("camera_y", camera_y_, 0.0);
    pnh_.param<double>("paper_height", paper_height_, .297);
    pnh_.param<double>("fallback_hfov_deg", fallback_hfov_deg_, 70.0);
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
    pnh_.param<double>("finish_observation_distance", finish_observation_distance_, 0.0);
    pnh_.param<double>("finish_max_lateral_error", finish_max_lateral_error_, .20);
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
    finish_observation_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/handoff/finish_observation_goal", 1);
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
    timer_ = nh_.createTimer(ros::Duration(.20), &VisualHandoffManager::timerCb, this);
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
    finish_observation_samples_ = 0;
    nav_state_.clear();
    checked_scan_stamp_ = ros::Time(0);
    publishStop();
    publishBool(goal_pub_, false);
    publishBool(visual_enable_pub_, false);
    publishBool(nav_enable_pub_, true);
    publishState("NAVIGATION: searching for the blue marker");
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
    if (mode_ == Mode::NAVIGATION) command_pub_.publish(*msg);
  }
  void visualCmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    if (mode_ == Mode::VISUAL) command_pub_.publish(*msg);
  }
  void navStateCb(const std_msgs::String::ConstPtr& msg) { nav_state_ = msg->data; }
  void visualStateCb(const std_msgs::String::ConstPtr& msg) {
    if (mode_ == Mode::VISUAL) publishState("VISUAL: " + msg->data);
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
    bool found_pan = false;
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      if (msg->name[i] == "camera_pan_joint") { pan_ = msg->position[i]; found_pan = true; }
      if (msg->name[i] == "camera_tilt_joint") tilt_ = msg->position[i];
    }
    if (found_pan) {
      joint_received_ = ros::Time::now();
      joint_stamp_ = msg->header.stamp;
      have_joint_ = true;
    }
  }
  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
    scan_ = *msg;
    scan_received_ = ros::Time::now();
    have_scan_ = true;
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
    if (!maze_planner::visualBearing(pan_, tilt_, observation_.center_x,
                                      observation_.center_y, cx, cy, fx, fy, bearing)) return false;
    const double down = tilt_+std::atan2(observation_.center_y-cy, fy);
    *horizontal_range = fy*paper_height_/observation_.pixel_height*std::cos(down);
    return std::isfinite(*horizontal_range) && *horizontal_range > 0.0 &&
           *horizontal_range <= max_target_range_;
  }

  bool corridorReady(maze_planner::WallObservation* associated_wall, std::string* reason) {
    const ros::Time now = ros::Time::now();
    if (!have_observation_ || (now-observation_received_).toSec() > observation_timeout_ ||
        !have_joint_ || (now-joint_received_).toSec() > joint_timeout_ ||
        !have_scan_ || (now-scan_received_).toSec() > scan_timeout_) {
      *reason = "waiting for camera, gimbal or lidar";
      return false;
    }
    if (observation_.header.stamp.isZero() || joint_stamp_.isZero() || scan_.header.stamp.isZero() ||
        std::fabs((observation_.header.stamp-joint_stamp_).toSec()) > max_sensor_skew_ ||
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
    if (!maze_planner::fitTargetWall(points, {camera_x_,camera_y_}, bearing, &wall,
                                      max_target_range_)) {
      *reason = "no lidar surface associated with blue marker";
      return false;
    }
    const double lidar_range = std::hypot(wall.marker.x-camera_x_, wall.marker.y-camera_y_);
    const double tolerance = std::max(range_absolute_tolerance_,
                                       range_relative_tolerance_*visual_range);
    if (std::fabs(lidar_range-visual_range) > tolerance) {
      *reason = "camera and lidar target distances differ";
      return false;
    }
    *associated_wall = wall;
    const maze_planner::Point2 tangent(-wall.outward.y, wall.outward.x);
    if (finish_observation_distance_ > corridor_params_.check_standoff &&
        std::fabs(maze_planner::dot(wall.marker, tangent)) > finish_max_lateral_error_) {
      *reason = "blue marker approach needs lateral alignment";
      return false;
    }
    return maze_planner::handoffCorridorClear(scan_, obstacles, footprint_,
                                               wall.marker.x, wall.marker.y,
                                               corridor_params_, reason);
  }

  void suggestFinishObservation(const maze_planner::WallObservation& wall) {
    if (finish_observation_distance_ <= corridor_params_.check_standoff ||
        nav_state_ != "OBSERVE: mapped course end reached; scanning for blue marker" ||
        wall.inliers == 0) {
      finish_observation_samples_ = 0;
      return;
    }
    const auto candidate = maze_planner::parkingPosition(wall, finish_observation_distance_);
    const double travel = maze_planner::norm(candidate);
    const double progress = maze_planner::norm(wall.marker) - finish_observation_distance_;
    if (travel < .20 || travel > 1.25 || progress < .20) {
      finish_observation_samples_ = 0;
      return;
    }
    if (maze_planner::norm(candidate-last_finish_observation_) > .15)
      finish_observation_samples_ = 0;
    last_finish_observation_ = candidate;
    if (++finish_observation_samples_ < required_clear_scans_) return;
    finish_observation_samples_ = 0;

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "base_footprint";
    goal.pose.position.x = candidate.x;
    goal.pose.position.y = candidate.y;
    const double half_yaw = .5*maze_planner::parkingYaw(wall);
    goal.pose.orientation.z = std::sin(half_yaw);
    goal.pose.orientation.w = std::cos(half_yaw);
    finish_observation_pub_.publish(goal);
  }

  void timerCb(const ros::TimerEvent&) {
    if (mode_ != Mode::NAVIGATION) return;
    if (!have_scan_ || (ros::Time::now()-scan_received_).toSec() > scan_timeout_) {
      clear_scans_ = 0;
      finish_observation_samples_ = 0;
      publishState("NAVIGATION: waiting for lidar");
      return;
    }
    if (scan_.header.stamp == checked_scan_stamp_) return;
    checked_scan_stamp_ = scan_.header.stamp;
    std::string reason;
    maze_planner::WallObservation wall;
    if (!corridorReady(&wall, &reason)) {
      clear_scans_ = 0;
      publishState("NAVIGATION: " + reason);
      suggestFinishObservation(wall);
      return;
    }
    finish_observation_samples_ = 0;
    if (++clear_scans_ < required_clear_scans_) {
      publishState("NAVIGATION: confirming clear corridor");
      return;
    }

    // The arbiter stops forwarding navigation before enabling visual control.
    mode_ = Mode::VISUAL;
    publishStop();
    publishBool(nav_enable_pub_, false);
    publishBool(visual_enable_pub_, true);
    publishState("VISUAL: lidar corridor clear; direct tracking enabled");
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher command_pub_, nav_enable_pub_, visual_enable_pub_, goal_pub_, state_pub_;
  ros::Publisher finish_observation_pub_;
  ros::Subscriber enable_sub_, nav_cmd_sub_, visual_cmd_sub_, nav_state_sub_, visual_state_sub_;
  ros::Subscriber visual_goal_sub_, observation_sub_, joint_sub_, scan_sub_, camera_sub_;
  ros::Timer timer_;
  maze_planner::HandoffCorridorParameters corridor_params_;
  stm32_bridge::SafetyFootprint footprint_, self_footprint_;
  stm32_bridge::BlueTargetObservation observation_;
  sensor_msgs::LaserScan scan_;
  sensor_msgs::CameraInfo camera_info_;
  ros::Time observation_received_, joint_received_, joint_stamp_, scan_received_, checked_scan_stamp_;
  Mode mode_{Mode::IDLE};
  bool auto_start_{false}, have_observation_{false}, have_joint_{false}, have_scan_{false};
  bool have_camera_info_{false};
  int required_clear_scans_{3}, clear_scans_{0};
  int finish_observation_samples_{0};
  double finish_observation_distance_{0.0};
  double finish_max_lateral_error_{.20};
  maze_planner::Point2 last_finish_observation_;
  double camera_x_{.145}, camera_y_{0.0}, paper_height_{.297}, fallback_hfov_deg_{70.0};
  double max_target_range_{8.0}, range_absolute_tolerance_{.25}, range_relative_tolerance_{.20};
  double scan_timeout_{.5}, observation_timeout_{.5}, joint_timeout_{1.5}, max_sensor_skew_{.5};
  double pan_{0.0}, tilt_{0.0};
  std::string state_, nav_state_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "visual_handoff_manager");
  VisualHandoffManager node;
  ros::spin();
  return 0;
}
