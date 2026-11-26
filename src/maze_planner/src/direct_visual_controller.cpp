#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <stm32_bridge/BlueTargetObservation.h>
#include <stm32_bridge/footprint_parameters.hpp>
#include <maze_planner/direct_reactive_policy.h>
#include <maze_planner/gimbal_history.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

class DirectVisualController {
 public:
  DirectVisualController() : nh_(), pnh_("~") {
    pnh_.param<bool>("auto_start", enabled_, false);
    pnh_.param<bool>("require_gimbal", require_gimbal_, true);
    pnh_.param<double>("max_speed", params_.max_speed, 0.10);
    pnh_.param<double>("min_approach_speed", params_.min_approach_speed, 0.10);
    pnh_.param<double>("max_turn", params_.max_turn, 0.25);
    pnh_.param<double>("turn_gain", params_.turn_gain, 1.0);
    pnh_.param<double>("turn_before_drive", params_.turn_before_drive, 0.44);
    pnh_.param<double>("stop_ratio", params_.stop_ratio, 0.16);
    double tolerance_deg = 5.0;
    pnh_.param<double>("facing_tolerance_deg", tolerance_deg, 5.0);
    params_.stop_bearing = tolerance_deg * M_PI / 180.0;
    pnh_.param<double>("avoid_distance", params_.avoid_distance, 0.65);
    pnh_.param<double>("target_distance_margin", params_.target_distance_margin, 0.25);
    pnh_.param<double>("lookahead_time", params_.lookahead_time, 2.0);
    pnh_.param<double>("paper_height", paper_height_, 0.297);
    pnh_.param<double>("fallback_hfov_deg", fallback_hfov_deg_, 88.0);
    pnh_.param<double>("camera_x", camera_x_, 0.145);
    pnh_.param<double>("laser_x", laser_x_, 0.315);
    pnh_.param<double>("laser_y", laser_y_, 0.0);
    pnh_.param<double>("laser_yaw", laser_yaw_, 0.0);
    pnh_.param<bool>("terminal_only", params_.terminal_only, false);
    pnh_.param<double>("gimbal_sweep_hold_time", gimbal_sweep_hold_time_, 2.6);
    pnh_.param<double>("metric_stop_depth_min", params_.metric_stop_depth_min, .48);
    pnh_.param<double>("metric_stop_depth_max", params_.metric_stop_depth_max, .63);
    pnh_.param<double>("scan_timeout", scan_timeout_, 0.5);
    pnh_.param<double>("observation_timeout", observation_timeout_, 0.5);
    pnh_.param<double>("joint_timeout", joint_timeout_, 1.5);
    pnh_.param<double>("gimbal_max_skew", gimbal_max_skew_, .55);

    params_.max_speed = std::max(0.0, params_.max_speed);
    params_.max_turn = std::max(0.0, params_.max_turn);
    params_.stop_ratio = std::max(0.0, std::min(1.0, params_.stop_ratio));
    params_.stop_bearing = std::max(0.0, std::min(M_PI, params_.stop_bearing));
    params_.lookahead_time = std::max(0.1, params_.lookahead_time);

    // The exact same STL polygon is loaded into the serial bridge below.
    footprint_.padding = 0.0;
    ros::NodeHandle geometry_nh("/robot_geometry");
    if (!stm32_bridge::readFootprintParam(geometry_nh, &footprint_)) return;
    self_footprint_ = footprint_;
    footprint_.padding = 0.03;

    command_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    goal_pub_ = nh_.advertise<std_msgs::Bool>("/goal_reached", 1, true);
    state_pub_ = nh_.advertise<std_msgs::String>("/blue_autonomy/state", 1, true);
    observation_sub_ = nh_.subscribe("/blue_target/observation", 3,
                                      &DirectVisualController::observationCb, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &DirectVisualController::scanCb, this);
    joint_sub_ = nh_.subscribe("/gimbal/joint_states", 3, &DirectVisualController::jointCb, this);
    connected_sub_ = nh_.subscribe("/gimbal/connected", 2,
                                   &DirectVisualController::connectedCb, this);
    ready_sub_ = nh_.subscribe("/gimbal/servo_ready", 2, &DirectVisualController::readyCb, this);
    camera_sub_ = nh_.subscribe("/usb_cam/camera_info", 2, &DirectVisualController::cameraCb, this);
    odom_sub_ = nh_.subscribe("/odom", 5, &DirectVisualController::odomCb, this);
    enable_sub_ = nh_.subscribe("/blue_autonomy/enable", 2, &DirectVisualController::enableCb, this);
    publishGoal(false);
    publishState(enabled_ ? "WAIT: sensor data" : "IDLE: direct control disabled");
    timer_ = nh_.createTimer(ros::Duration(maze_planner::kDirectControlPeriod), &DirectVisualController::timerCb, this);
  }

 private:
  void publishGoal(bool reached) {
    std_msgs::Bool msg;
    msg.data = reached;
    goal_pub_.publish(msg);
  }

  void publishState(const std::string& state) {
    if (state == state_) return;
    state_ = state;
    std_msgs::String msg;
    msg.data = state;
    state_pub_.publish(msg);
    ROS_INFO("Direct visual: %s", state.c_str());
  }

  void publishCommand(double v, double w) {
    geometry_msgs::Twist msg;
    msg.linear.x = v;
    msg.angular.z = w;
    command_pub_.publish(msg);
  }

  void enableCb(const std_msgs::Bool::ConstPtr& msg) {
    if (enabled_ == msg->data) return;
    enabled_ = msg->data;
    cached_wall_time_ = ros::Time();
    search_start_ = ros::Time();
    complete_ = false;
    avoid_side_ = 0;
    publishCommand(0.0, 0.0);
    publishGoal(false);
    publishState(enabled_ ? "WAIT: sensor data" : "IDLE: direct control disabled");
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

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(yaw)) return;
    odom_position_ = {p.x, p.y};
    yaw_ = yaw;
    odom_time_ = ros::Time::now();
  }

  void connectedCb(const std_msgs::Bool::ConstPtr& msg) { gimbal_connected_ = msg->data; }
  void readyCb(const std_msgs::Bool::ConstPtr& msg) { gimbal_ready_ = msg->data; }
  void cameraCb(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    camera_info_ = *msg;
    have_camera_info_ = msg->width > 0 && msg->height > 0 &&
        std::isfinite(msg->K[0]) && std::isfinite(msg->K[4]) &&
        msg->K[0] > 1.0 && msg->K[4] > 1.0;
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
    obstacles_.clear();
    scan_points_.clear();
    front_clearance_ = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      if (!std::isfinite(r) || r <= msg->range_min || r >= msg->range_max || r > 6.0) continue;
      const double angle = msg->angle_min + i*msg->angle_increment + laser_yaw_;
      const stm32_bridge::SafetyPoint point{laser_x_+r*std::cos(angle),
                                             laser_y_+r*std::sin(angle)};
      if (stm32_bridge::pointInsideFootprint(point, {0.0, 0.0}, 0.0, self_footprint_))
        continue;
      scan_points_.emplace_back(point.x, point.y);
      if (r > 4.0) continue;
      obstacles_.push_back(point);
      if (std::fabs(point.y) <= footprint_.half_width + footprint_.padding) {
        double rear = 0.0, front = 0.0;
        stm32_bridge::longitudinalBodyLimits(footprint_, point.y, &rear, &front);
        if (point.x >= front) front_clearance_ = std::min(front_clearance_, point.x-front);
      }
    }
    scan_received_ = ros::Time::now();
    have_scan_ = true;
  }

  bool observationGeometry(double* bearing, double* visual_range, double* target_front_distance) const {
    if (!observation_.detected || observation_.image_width == 0 ||
        observation_.image_height == 0 || observation_.pixel_height <= 0.0) return false;
    double fx, fy;
    double cx = .5 * observation_.image_width;
    double cy = .5 * observation_.image_height;
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
    double pan = 0.0, tilt = 0.0;
    if (require_gimbal_ && !gimbal_history_.angles(observation_.header.stamp, &pan, &tilt, gimbal_max_skew_))
      return false;
    if (!maze_planner::visualBearing(pan, tilt, observation_.center_x,
                                      observation_.center_y, cx, cy, fx, fy, bearing))
      return false;
    const double down = tilt + std::atan2(observation_.center_y-cy, fy);
    const double horizontal_range = fy*paper_height_/observation_.pixel_height*std::cos(down);
    if (!std::isfinite(horizontal_range) || horizontal_range <= 0.0) return false;
    *visual_range = horizontal_range;
    *target_front_distance = std::max(0.0, camera_x_+horizontal_range*std::cos(*bearing)-footprint_.front);
    return true;
  }

  void updateWall(maze_planner::ReactiveInput* input, double visual_range,
                  const ros::Time& now, bool odom_fresh) {
    maze_planner::WallObservation wall;
    if (input->target_visible && maze_planner::fitTargetWall(
        scan_points_, {camera_x_, 0.0}, input->bearing, &wall, 6.0, visual_range,
        std::max(.25, .30 * visual_range))) {
      if (odom_fresh && scan_received_ > last_wall_scan_time_ &&
          -maze_planner::dot(wall.marker, wall.outward) <= .75) {
        cached_marker_ = odom_position_ + maze_planner::rotate(wall.marker, yaw_);
        cached_normal_ = maze_planner::rotate(wall.outward, yaw_);
        cached_wall_time_ = now;
        last_wall_scan_time_ = scan_received_;
      }
    } else {
      if (!odom_fresh || cached_wall_time_.isZero() || (now - cached_wall_time_).toSec() > 1.0)
        return;
      const auto predicted = maze_planner::rotate(cached_marker_ - odom_position_, -yaw_);
      const auto normal = maze_planner::rotate(cached_normal_, -yaw_);
      const double bearing = std::atan2(predicted.y, predicted.x - camera_x_);
      if (!maze_planner::fitTargetWall(scan_points_, {camera_x_, 0.0}, bearing, &wall, 6.0,
                                      maze_planner::norm(predicted - maze_planner::Point2(camera_x_, 0)),
                                      .10) || maze_planner::dot(wall.outward, normal) < .94)
        return;
      input->bearing = std::atan2(wall.marker.y, wall.marker.x);
    }
    input->wall_valid = true;
    input->wall_marker = wall.marker;
    input->wall_outward = wall.outward;
    input->target_front_distance = std::max(0.0,
        -maze_planner::dot(wall.marker, wall.outward) - footprint_.front);
  }

  void timerCb(const ros::TimerEvent&) {
    if (!enabled_) return;
    if (complete_) {
      publishCommand(0.0, 0.0);
      publishGoal(true);
      return;
    }
    const ros::Time now = ros::Time::now();
    if (!have_scan_ || (now-scan_received_).toSec() > scan_timeout_ ||
        (require_gimbal_ && (!gimbal_connected_ || !gimbal_ready_ ||
                             (now-joint_received_).toSec() > joint_timeout_))) {
      publishCommand(0.0, 0.0);
      publishState("WAIT: lidar or gimbal data unavailable");
      return;
    }

    maze_planner::ReactiveInput input;
    input.footprint = footprint_;
    input.obstacles = obstacles_;
    input.front_clearance = front_clearance_;
    input.contour_ratio = observation_.contour_ratio;
    double visual_range = 0.0;
    input.target_visible = have_observation_ && (now-observation_received_).toSec() <=
        observation_timeout_ &&
        observationGeometry(&input.bearing, &visual_range, &input.target_front_distance);
    const bool odom_fresh = (now - odom_time_).toSec() <= .30;
    updateWall(&input, visual_range, now, odom_fresh);
    // A recently established metric wall cannot be replaced by a single
    // large contour while its matching scan is momentarily unavailable.
    input.permit_visual_stop = cached_wall_time_.isZero() ||
                               (now - cached_wall_time_).toSec() > 1.0;
    if (input.target_visible || input.wall_valid) {
      search_side_ = input.bearing < 0.0 ? -1 : 1;
      search_start_ = ros::Time();
    } else {
      if (search_start_.isZero()) {
        search_start_ = now;
        search_center_yaw_ = yaw_;
      }
      const double elapsed = (now - search_start_).toSec();
      const double turn_speed = std::min(.28, params_.max_turn);
      double turn = 0.0;
      if (odom_fresh && elapsed >= gimbal_sweep_hold_time_) {
        const double offset = std::atan2(std::sin(yaw_ - search_center_yaw_),
                                         std::cos(yaw_ - search_center_yaw_));
        if (search_side_ * offset >= M_PI / 4 - .05) search_side_ *= -1;
        turn = search_side_ * turn_speed;
      }
      if (params_.terminal_only && elapsed >= gimbal_sweep_hold_time_ + 3.2) {
        publishCommand(0.0, 0.0);
        publishState("BLOCKED: marker lost in terminal corridor");
        return;
      }
      publishCommand(0.0, maze_planner::reactivePathClear(input, 0.0, turn, .25) ? turn : 0.0);
      publishState("SEARCH: gimbal-first bounded sweep");
      return;
    }
    const auto command = maze_planner::reactiveVisualCommand(input, params_, avoid_side_, search_side_);
    avoid_side_ = command.avoid_side;
    if (command.complete) {
      publishCommand(0.0, 0.0);
      publishGoal(true);
      complete_ = true;
      publishState("COMPLETE: finish position confirmed");
      return;
    }
    publishCommand(command.v, command.w);
    publishState(std::string(command.state) + ": direct lidar and blue-target control");
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher command_pub_, goal_pub_, state_pub_;
  ros::Subscriber observation_sub_, scan_sub_, joint_sub_, connected_sub_, ready_sub_, camera_sub_, enable_sub_, odom_sub_;
  ros::Timer timer_;
  maze_planner::ReactiveParameters params_;
  stm32_bridge::SafetyFootprint footprint_, self_footprint_;
  std::vector<stm32_bridge::SafetyPoint> obstacles_;
  std::vector<maze_planner::Point2> scan_points_;
  stm32_bridge::BlueTargetObservation observation_;
  sensor_msgs::CameraInfo camera_info_;
  ros::Time observation_received_, scan_received_, joint_received_;
  bool enabled_{false}, complete_{false}, require_gimbal_{true};
  bool have_observation_{false}, have_scan_{false}, have_camera_info_{false};
  bool gimbal_connected_{false}, gimbal_ready_{false};
  double paper_height_{0.297}, fallback_hfov_deg_{88.0};
  double camera_x_{0.145}, laser_x_{0.315}, laser_y_{0.0}, laser_yaw_{0.0};
  double scan_timeout_{0.5}, observation_timeout_{0.5}, joint_timeout_{1.5};
  double front_clearance_{std::numeric_limits<double>::infinity()};
  double gimbal_sweep_hold_time_{2.6}, yaw_{0.0}, search_center_yaw_{0.0};
  maze_planner::Point2 odom_position_, cached_marker_, cached_normal_;
  ros::Time odom_time_, search_start_, cached_wall_time_, last_wall_scan_time_;
  maze_planner::GimbalHistory gimbal_history_;
  double gimbal_max_skew_{.55};
  int avoid_side_{0}, search_side_{1};
  std::string state_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "direct_visual_controller");
  DirectVisualController node;
  ros::spin();
  return 0;
}
