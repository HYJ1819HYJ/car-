#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <stm32_bridge/BlueTargetObservation.h>
#include <stm32_bridge/footprint_parameters.hpp>
#include <maze_planner/direct_reactive_policy.h>

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
    pnh_.param<double>("fallback_hfov_deg", fallback_hfov_deg_, 70.0);
    pnh_.param<double>("camera_x", camera_x_, 0.145);
    pnh_.param<double>("laser_x", laser_x_, 0.315);
    pnh_.param<double>("laser_y", laser_y_, 0.0);
    pnh_.param<double>("laser_yaw", laser_yaw_, 0.0);
    pnh_.param<double>("scan_timeout", scan_timeout_, 0.5);
    pnh_.param<double>("observation_timeout", observation_timeout_, 0.5);
    pnh_.param<double>("joint_timeout", joint_timeout_, 1.5);

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
    enable_sub_ = nh_.subscribe("/blue_autonomy/enable", 2, &DirectVisualController::enableCb, this);
    publishGoal(false);
    publishState(enabled_ ? "WAIT: sensor data" : "IDLE: direct control disabled");
    timer_ = nh_.createTimer(ros::Duration(0.05), &DirectVisualController::timerCb, this);
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
    enabled_ = msg->data;
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
    bool found_pan = false;
    for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      if (msg->name[i] == "camera_pan_joint") { pan_ = msg->position[i]; found_pan = true; }
      if (msg->name[i] == "camera_tilt_joint") tilt_ = msg->position[i];
    }
    if (found_pan) { have_joint_ = true; joint_received_ = ros::Time::now(); }
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
    front_clearance_ = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      if (!std::isfinite(r) || r <= msg->range_min || r >= msg->range_max || r > 4.0) continue;
      const double angle = msg->angle_min + i*msg->angle_increment + laser_yaw_;
      const stm32_bridge::SafetyPoint point{laser_x_+r*std::cos(angle),
                                             laser_y_+r*std::sin(angle)};
      if (stm32_bridge::pointInsideFootprint(point, {0.0, 0.0}, 0.0, self_footprint_))
        continue;
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

  bool observationGeometry(double* bearing, double* target_front_distance) const {
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
    if (!maze_planner::visualBearing(pan_, tilt_, observation_.center_x,
                                      observation_.center_y, cx, cy, fx, fy, bearing))
      return false;
    const double down = tilt_ + std::atan2(observation_.center_y-cy, fy);
    const double horizontal_range = fy*paper_height_/observation_.pixel_height*std::cos(down);
    if (!std::isfinite(horizontal_range) || horizontal_range <= 0.0) return false;
    *target_front_distance = std::max(0.0, camera_x_+horizontal_range*std::cos(*bearing)-footprint_.front);
    return true;
  }

  void timerCb(const ros::TimerEvent&) {
    if (!enabled_ || complete_) return;
    const ros::Time now = ros::Time::now();
    if (!have_scan_ || (now-scan_received_).toSec() > scan_timeout_ ||
        !have_observation_ || (now-observation_received_).toSec() > observation_timeout_ ||
        (require_gimbal_ && (!gimbal_connected_ || !gimbal_ready_ || !have_joint_ ||
                             (now-joint_received_).toSec() > joint_timeout_))) {
      publishCommand(0.0, 0.0);
      publishState("WAIT: camera, lidar or gimbal data unavailable");
      return;
    }

    maze_planner::ReactiveInput input;
    input.footprint = footprint_;
    input.obstacles = obstacles_;
    input.front_clearance = front_clearance_;
    input.contour_ratio = observation_.contour_ratio;
    input.target_visible = observationGeometry(&input.bearing, &input.target_front_distance);
    if (input.target_visible) search_side_ = input.bearing < 0.0 ? -1 : 1;
    const auto command = maze_planner::reactiveVisualCommand(input, params_, avoid_side_, search_side_);
    avoid_side_ = command.avoid_side;
    if (command.complete) {
      publishCommand(0.0, 0.0);
      publishGoal(true);
      complete_ = true;
      publishState("COMPLETE: area and facing thresholds reached");
      return;
    }
    publishCommand(command.v, command.w);
    publishState(std::string(command.state) + ": direct lidar and blue-target control");
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher command_pub_, goal_pub_, state_pub_;
  ros::Subscriber observation_sub_, scan_sub_, joint_sub_, connected_sub_, ready_sub_, camera_sub_, enable_sub_;
  ros::Timer timer_;
  maze_planner::ReactiveParameters params_;
  stm32_bridge::SafetyFootprint footprint_, self_footprint_;
  std::vector<stm32_bridge::SafetyPoint> obstacles_;
  stm32_bridge::BlueTargetObservation observation_;
  sensor_msgs::CameraInfo camera_info_;
  ros::Time observation_received_, scan_received_, joint_received_;
  bool enabled_{false}, complete_{false}, require_gimbal_{true};
  bool have_observation_{false}, have_scan_{false}, have_joint_{false}, have_camera_info_{false};
  bool gimbal_connected_{false}, gimbal_ready_{false};
  double pan_{0.0}, tilt_{0.0}, paper_height_{0.297}, fallback_hfov_deg_{70.0};
  double camera_x_{0.145}, laser_x_{0.315}, laser_y_{0.0}, laser_yaw_{0.0};
  double scan_timeout_{0.5}, observation_timeout_{0.5}, joint_timeout_{1.5};
  double front_clearance_{std::numeric_limits<double>::infinity()};
  int avoid_side_{0}, search_side_{1};
  std::string state_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "direct_visual_controller");
  DirectVisualController node;
  ros::spin();
  return 0;
}
