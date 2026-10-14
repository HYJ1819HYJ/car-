#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <serial/serial.h>
#include <algorithm>
#include <cerrno>
#include <limits>
#include <vector>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <sstream>
#include "stm32_bridge/protocol.hpp"
#include "stm32_bridge/safety_geometry.hpp"
#include "stm32_bridge/footprint_parameters.hpp"

using stm32_bridge::ControlCmd;
using stm32_bridge::ControlProtocol;
using stm32_bridge::EncoderFeedback;
using stm32_bridge::FeedbackProtocol;
using stm32_bridge::SafetyFootprint;
using stm32_bridge::SafetyPoint;
using stm32_bridge::packControl;
using stm32_bridge::trajectoryIntersects;
using stm32_bridge::unpackFeedback;

/**
 * 串口桥接节点（手柄控制 + 接收 /cmd_vel 自动导航）
 *
 * 功能：
 *   1. 直接读取 /dev/input/js0 获取手柄输入（低延迟 LB+摇杆）
 *   2. 订阅 /cmd_vel 接收自动导航速度（move_base / 手动控制）
 *   3. 手柄优先：LB按住=手柄控制，LB松开= /cmd_vel
 *   4. 将选中的速度通过串口发给 STM32
 *   5. 发布 /wheel_encoder 编码器数据
 *   6. 监听 /goal_reached，仅在导航状态机确认到达后锁停
 *
 * 手柄映射 (ShanWan XBOX 360)：
 *   axis 0 = 左摇杆左右, axis 1 = 左摇杆前后
 *   button 4 = LB (死手柄关)
 */

class SerialBridgeNode {
public:
  SerialBridgeNode(): nh_("~") {
    nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
    nh_.param<int>("baud", baud_, 115200);
    nh_.param<std::string>("joy_dev", joy_dev_, "/dev/input/js0");
    nh_.param<double>("max_v", max_v_, 0.5);
    nh_.param<double>("max_w", max_w_, 1.0);
    nh_.param<double>("deadzone", deadzone_, 0.1);
    nh_.param<int>("angular_command_sign", angular_command_sign_, -1);
    nh_.param<int>("joy_angular_axis_sign", joy_angular_axis_sign_, -1);
    nh_.param<bool>("safety_enable", safety_enable_, true);
    nh_.param<double>("safety_stop_distance", safety_stop_distance_, 0.15);
    nh_.param<double>("safety_slow_distance", safety_slow_distance_, 0.60);
    nh_.param<double>("safety_scan_timeout", safety_scan_timeout_, 0.30);
    nh_.param<double>("safety_rear_stop_distance", safety_rear_stop_distance_, 0.15);
    nh_.param<double>("safety_prediction_time", safety_prediction_time_, 0.60);
    nh_.param<double>("safety_prediction_step", safety_prediction_step_, 0.03);
    nh_.param<double>("safety_collision_padding", safety_collision_padding_, 0.03);
    nh_.param<bool>("navigation_allow_reverse", navigation_allow_reverse_, false);
    nh_.param<double>("joystick_reverse_max_speed", joystick_reverse_max_speed_, 0.15);
    nh_.param<double>("vehicle_front_x", vehicle_front_x_, 0.3483);
    nh_.param<double>("vehicle_rear_x", vehicle_rear_x_, -0.1517);
    nh_.param<double>("vehicle_half_width", vehicle_half_width_, 0.24);
    nh_.param<double>("vehicle_footprint_padding", vehicle_footprint_padding_, 0.02);
    nh_.param<double>("safety_half_width", safety_half_width_, 0.27);
    nh_.param<double>("laser_x", laser_x_, 0.315);
    nh_.param<double>("laser_y", laser_y_, 0.0);
    nh_.param<double>("laser_yaw", laser_yaw_, 0.0);

    angular_command_sign_ = angular_command_sign_ < 0 ? -1 : 1;
    joy_angular_axis_sign_ = joy_angular_axis_sign_ < 0 ? -1 : 1;
    if (safety_stop_distance_ < 0.0) safety_stop_distance_ = 0.0;
    if (safety_slow_distance_ <= safety_stop_distance_) {
      safety_slow_distance_ = safety_stop_distance_ + 0.10;
    }
    safety_rear_stop_distance_ = std::max(0.0, safety_rear_stop_distance_);
    safety_prediction_time_ = std::max(0.10, safety_prediction_time_);
    safety_prediction_step_ = std::max(0.01, safety_prediction_step_);
    safety_collision_padding_ = std::max(0.0, safety_collision_padding_);
    joystick_reverse_max_speed_ = std::max(0.0, joystick_reverse_max_speed_);

    SafetyFootprint body;
    body.front=vehicle_front_x_;
    body.rear=vehicle_rear_x_;
    body.half_width=vehicle_half_width_;
    if (!stm32_bridge::readFootprintParam(nh_,&body)) {
      ros::shutdown();
      return;
    }
    if (!body.vertices.empty()) {
      vehicle_front_x_=body.front;
      vehicle_rear_x_=body.rear;
      vehicle_half_width_=body.half_width;
      safety_half_width_=body.half_width+safety_collision_padding_;
    }
    self_filter_footprint_=body;
    self_filter_footprint_.padding=vehicle_footprint_padding_;
    collision_footprint_=body;
    collision_footprint_.padding=safety_collision_padding_;

    fb_pub_  = nh_.advertise<std_msgs::String>("/stm32_feedback_text", 20);
    enc_pub_ = nh_.advertise<std_msgs::Int16MultiArray>("/wheel_encoder", 20);
    front_distance_pub_ = nh_.advertise<std_msgs::Float32>("/front_obstacle_distance", 10);
    rear_distance_pub_ = nh_.advertise<std_msgs::Float32>("/rear_obstacle_distance", 10);
    nearest_distance_pub_ = nh_.advertise<std_msgs::Float32>("/nearest_obstacle_radius", 10);
    safety_stop_pub_ = nh_.advertise<std_msgs::Bool>("/safety_stop", 10);
    safety_collision_pub_ = nh_.advertise<std_msgs::Bool>("/safety_collision_prediction", 10);
    sent_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/stm32_command_sent", 10);
    lb_pressed_pub_ = nh_.advertise<std_msgs::Bool>("/joystick_lb_pressed", 10);
    control_source_pub_ = nh_.advertise<std_msgs::String>("/control_source", 10);

    openSerial();
    openJoystick();

    // 订阅 /cmd_vel（来自 move_base 或手柄映射）
    cmd_sub_ = nh_.subscribe("/cmd_vel", 5, &SerialBridgeNode::cmdCb, this);
    // 蓝纸被看见不等于已经进入0.8m终点区。只接受任务节点确认到达后的停车信号。
    std::string goal_stop_topic;
    nh_.param<std::string>("goal_stop_topic", goal_stop_topic, "/goal_reached");
    goal_sub_ = nh_.subscribe(goal_stop_topic, 5, &SerialBridgeNode::goalCb, this);
    scan_sub_ = nh_.subscribe("/scan", 5, &SerialBridgeNode::scanCb, this);

    loop_timer_ = nh_.createTimer(ros::Duration(0.01), &SerialBridgeNode::mainLoop, this);

    ROS_INFO("=== SerialBridgeNode ===");
    ROS_INFO("  Collision outline: %s, vertices=%zu",
             body.vertices.empty()?"legacy rectangle":"convex polygon",
             body.vertices.size());
    ROS_INFO("  Serial: %s @ %d", port_.c_str(), baud_);
    ROS_INFO("  Joystick: %s", joy_dev_.c_str());
    ROS_INFO("  Hold LB for joystick; release LB for /cmd_vel navigation");
    ROS_INFO("  Confirmed /goal_reached triggers an immediate latched stop");
    ROS_INFO("  Angular command sign: %d", angular_command_sign_);
    ROS_INFO("  Joystick angular axis sign: %d", joy_angular_axis_sign_);
    ROS_INFO("  Lidar safety: %s, front stop=%.2fm rear stop=%.2fm slow=%.2fm",
             safety_enable_ ? "ON" : "OFF",
             safety_stop_distance_, safety_rear_stop_distance_, safety_slow_distance_);
    ROS_INFO("  Full-footprint prediction: %.2fs, padding=%.3fm; navigation reverse=%s",
             safety_prediction_time_, safety_collision_padding_,
             navigation_allow_reverse_ ? "allowed" : "disabled");
  }

  ~SerialBridgeNode() {
    // STM32 会持续向电机驱动器重发最后一条速度；正常退出前明确发送零速。
    if (ser_.isOpen()) {
      try {
        const auto stop_frame = packControl(ControlCmd{}, ControlProtocol{});
        for (int i = 0; i < 3; ++i) ser_.write(stop_frame);
      } catch (const std::exception& e) {
        ROS_ERROR("Failed to send shutdown stop frame: %s", e.what());
      }
    }
    if (joy_fd_ >= 0) ::close(joy_fd_);
  }

private:
  void openSerial() {
    try {
      ser_.setPort(port_);
      ser_.setBaudrate(baud_);
      serial::Timeout to = serial::Timeout::simpleTimeout(static_cast<uint32_t>(5));
      ser_.setTimeout(to);
      ser_.open();
      ser_.flush();
      ROS_INFO("Serial port opened");
    } catch (const std::exception& e) {
      ROS_FATAL("Failed to open serial port: %s", e.what());
      ros::shutdown();
    }
  }

  void openJoystick() {
    joy_fd_ = ::open(joy_dev_.c_str(), O_RDONLY | O_NONBLOCK);
    if (joy_fd_ < 0) {
      ROS_WARN("Cannot open joystick %s (not connected?)", joy_dev_.c_str());
    } else {
      ROS_INFO("Joystick opened");
    }
  }

  void cmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_v_ = msg->linear.x;
    cmd_w_ = msg->angular.z;
    last_cmd_time_ = ros::Time::now();
  }

  void goalCb(const std_msgs::Bool::ConstPtr& msg) {
    // 自动任务节点已确认到达接近点；立即锁停，满足至少3秒静止要求。
    if (msg->data && !goal_stopped_) {
      goal_stopped_ = true;
      ROS_WARN(">>> BLUE GOAL CONFIRMED: immediate latched stop <<<");
    } else if (!msg->data && goal_stopped_) {
      // 只有任务状态机在明确禁用/重新开始任务时才发布 false，允许下一次
      // 比赛机会复用同一批节点，而不必重启串口桥。
      goal_stopped_ = false;
      cmd_v_ = 0.0;
      cmd_w_ = 0.0;
      last_cmd_time_ = ros::Time(0);
      ROS_WARN("Blue-goal stop latch reset; waiting for a fresh command");
    }
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
    last_scan_time_ = ros::Time::now();
    have_scan_ = true;
    double clearance = std::numeric_limits<double>::infinity();
    double rear_clearance = std::numeric_limits<double>::infinity();
    double obstacle_radius = std::numeric_limits<double>::infinity();
    std::vector<SafetyPoint> obstacles;
    obstacles.reserve(msg->ranges.size());

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      if (!std::isfinite(r) || r <= std::max(0.05, static_cast<double>(msg->range_min)) ||
          r >= msg->range_max) {
        continue;
      }

      const double angle = msg->angle_min + i * msg->angle_increment + laser_yaw_;
      const double x_base = laser_x_ + r * std::cos(angle);
      const double y_base = laser_y_ + r * std::sin(angle);

      // Lidar can see brackets/chassis inside the robot's own footprint.
      // Costmap footprint clearing removes those returns; the independent
      // rotation guard must do the same or it permanently blocks all turns.
      const bool inside_robot = stm32_bridge::pointInsideFootprint(
          SafetyPoint{x_base,y_base},SafetyPoint{0.0,0.0},0.0,
          self_filter_footprint_);
      if (!inside_robot) {
        obstacles.push_back(SafetyPoint{x_base, y_base});
        obstacle_radius = std::min(obstacle_radius, std::hypot(x_base, y_base));
        // Directional clearances are measured from the actual front/rear body
        // edge, across the complete vehicle width.  Side obstacles do not
        // falsely block straight motion; curved motion is checked separately
        // by sweeping the complete footprint.
        if (std::fabs(y_base) <= safety_half_width_) {
          double body_rear,body_front;
          stm32_bridge::longitudinalBodyLimits(collision_footprint_,y_base,
                                               &body_rear,&body_front);
          if (x_base >= body_front) clearance=std::min(clearance,x_base-body_front);
          if (x_base <= body_rear) rear_clearance=std::min(rear_clearance,body_rear-x_base);
        }
      }
    }

    obstacle_points_.swap(obstacles);
    front_clearance_ = clearance;
    rear_clearance_ = rear_clearance;
    nearest_obstacle_radius_ = obstacle_radius;
    std_msgs::Float32 distance_msg;
    distance_msg.data = static_cast<float>(clearance);
    front_distance_pub_.publish(distance_msg);
    std_msgs::Float32 rear_msg;
    rear_msg.data = static_cast<float>(rear_clearance);
    rear_distance_pub_.publish(rear_msg);
    std_msgs::Float32 nearest_msg;
    nearest_msg.data = static_cast<float>(obstacle_radius);
    nearest_distance_pub_.publish(nearest_msg);
  }

  bool trajectoryBlocked(const double v, const double w) const {
    return trajectoryIntersects(obstacle_points_, collision_footprint_, v, w,
                                safety_prediction_time_, safety_prediction_step_);
  }

  void mainLoop(const ros::TimerEvent&) {
    // ========== 1. 读取手柄 ==========
    if (joy_fd_ >= 0) {
      struct js_event e;
      while (::read(joy_fd_, &e, sizeof(e)) == sizeof(e)) {
        // JS_EVENT_INIT 是正常的初始状态，不可直接丢弃。若节点启动时
        // 用户正按着 LB，丢弃 INIT 会让手柄一直被误判为未按下，直到
        // 再次松开并重按。
        const uint8_t event_type = e.type & ~JS_EVENT_INIT;
        switch (event_type) {
          case JS_EVENT_BUTTON:
            if (e.number < 11) joy_buttons_[e.number] = e.value;
            break;
          case JS_EVENT_AXIS:
            if (e.number < 8) joy_axes_[e.number] = static_cast<double>(e.value) / 32767.0;
            break;
        }
      }
      if (errno != EAGAIN) errno = 0;
    }

    // ========== 2. 计算速度 ==========
    double v = 0.0, w = 0.0;

    std::string control_source = "idle";
    bool lb_pressed = (joy_fd_ >= 0 && joy_buttons_[4] == 1);

    if (goal_stopped_) {
      // 终点了，不动
      v = 0.0; w = 0.0;
      control_source = "goal_stop_latched";
    } else {
      if (lb_pressed && joy_fd_ >= 0) {
        // ---- 手柄控制 ----
        double ax = joy_axes_[0];
        double ay = joy_axes_[1];
        if (fabs(ax) < deadzone_) ax = 0.0;
        if (fabs(ay) < deadzone_) ay = 0.0;
        v = -ay * max_v_;
        // Linux js axis 0 is normally negative to the left. Convert it to
        // ROS semantics first (+z=left); angular_command_sign_ is applied later.
        w = static_cast<double>(joy_angular_axis_sign_) * ax * max_w_;
        control_source = "joystick";
      } else {
        // ---- 自动导航 (/cmd_vel) ----
        double dt = (ros::Time::now() - last_cmd_time_).toSec();
        if (dt < 0.5) {
          v = cmd_v_;
          w = cmd_w_;
          control_source = "cmd_vel";
        }
      }
    }

    // ========== 3. 独立雷达安全层 ==========
    bool safety_stopped = false;
    bool collision_predicted = false;

    // DWA/recovery code must never back the vehicle into an unobserved corner.
    // Manual reverse remains available at a reduced speed and is protected by
    // the same rear clearance and full-footprint trajectory check below.
    if (control_source == "cmd_vel" && v < -1e-3 && !navigation_allow_reverse_) {
      v = 0.0;
      // Reject the complete planned twist. Keeping its angular component
      // turns a rejected reverse arc into an unintended spin in place.
      w = 0.0;
      safety_stopped = true;
      control_source += "+navigation_reverse_blocked";
      ROS_WARN_THROTTLE(0.5, "Autonomous reverse command rejected: full twist stopped");
    } else if (control_source == "joystick" && v < -joystick_reverse_max_speed_) {
      v = -joystick_reverse_max_speed_;
      control_source += "+reverse_speed_limited";
    }

    if (safety_enable_) {
      const bool scan_fresh = have_scan_ &&
          (ros::Time::now() - last_scan_time_).toSec() <= safety_scan_timeout_;
      if (!scan_fresh && (std::fabs(v) > 1e-3 || std::fabs(w) > 1e-3)) {
        v = 0.0;
        w = 0.0;
        safety_stopped = true;
        control_source += "+lidar_timeout_stop";
        ROS_WARN_THROTTLE(1.0, "Lidar timeout: safety stop");
      } else if (scan_fresh) {
        bool directional_stop = false;
        if (v > 1e-3 && front_clearance_ <= safety_stop_distance_) {
          v = 0.0;
          if (control_source == "cmd_vel") w = 0.0;
          directional_stop = true;
          safety_stopped = true;
          control_source += "+front_emergency_stop";
          ROS_WARN_THROTTLE(0.5, "Front clearance %.3fm: forward blocked",
                            front_clearance_);
        } else if (v < -1e-3 &&
                   rear_clearance_ <= safety_rear_stop_distance_) {
          v = 0.0;
          if (control_source == "cmd_vel") w = 0.0;
          directional_stop = true;
          safety_stopped = true;
          control_source += "+rear_emergency_stop";
          ROS_WARN_THROTTLE(0.5, "Rear clearance %.3fm: reverse blocked",
                            rear_clearance_);
        } else if (v > 1e-3 && front_clearance_ < safety_slow_distance_) {
          const double scale = std::max(0.0, std::min(1.0,
              (front_clearance_ - safety_stop_distance_) /
              (safety_slow_distance_ - safety_stop_distance_)));
          v *= scale;
          // Autonomous arcs must keep their curvature when slowed; reducing
          // v alone makes the car turn harder than its collision-checked plan.
          if (control_source == "cmd_vel") w *= scale;
          control_source += "+front_slow";
        } else if (v < -1e-3 && rear_clearance_ < safety_slow_distance_) {
          const double scale = std::max(0.0, std::min(1.0,
              (rear_clearance_ - safety_rear_stop_distance_) /
              (safety_slow_distance_ - safety_rear_stop_distance_)));
          v *= scale;
          if (control_source == "cmd_vel") w *= scale;
          control_source += "+rear_slow";
        }

        // Replace the old 0.45 m radius approximation with an exact sweep of
        // the complete asymmetric rectangle along the requested arc.
        if ((std::fabs(v) > 1e-3 || std::fabs(w) > 1e-3) &&
            trajectoryBlocked(v, w)) {
          collision_predicted = true;
          safety_stopped = true;
          // A joystick operator may explicitly keep a safe turn when blocked.
          // Autonomous motion must stop the entire rejected arc: replacing it
          // with a spin executes a different trajectory from the TEB plan.
          const bool turn_only_safe =
                                      control_source.compare(0, 8, "joystick") == 0 &&
                                      std::fabs(w) > 1e-3 &&
                                      !trajectoryBlocked(0.0, w);
          v = 0.0;
          if (turn_only_safe) {
            control_source += "+trajectory_stop_turn_allowed";
          } else {
            w = 0.0;
            control_source += "+trajectory_collision_stop";
          }
          ROS_WARN_THROTTLE(0.5,
                            "Full-footprint collision predicted: turn-only %s",
                            turn_only_safe ? "allowed" : "blocked");
        } else if (directional_stop && std::fabs(w) > 1e-3 &&
                   trajectoryBlocked(0.0, w)) {
          // Directional stop already removed translation; independently make
          // sure the remaining corner sweep is safe.
          collision_predicted = true;
          w = 0.0;
          control_source += "+turn_collision_stop";
        }
      }
    }

    std_msgs::Bool safety_msg;
    safety_msg.data = safety_stopped;
    safety_stop_pub_.publish(safety_msg);
    std_msgs::Bool collision_msg;
    collision_msg.data = collision_predicted;
    safety_collision_pub_.publish(collision_msg);

    std_msgs::Bool lb_msg;
    lb_msg.data = lb_pressed;
    lb_pressed_pub_.publish(lb_msg);

    std_msgs::String source_msg;
    source_msg.data = control_source;
    control_source_pub_.publish(source_msg);

    geometry_msgs::Twist sent_cmd_msg;
    sent_cmd_msg.linear.x = v;
    // Publish the safe command in standard ROS coordinates. The electrical
    // sign conversion belongs only on the serial wire below.
    sent_cmd_msg.angular.z = w;
    sent_cmd_pub_.publish(sent_cmd_msg);

    // ========== 4. 发送串口 ==========
    if (ser_.isOpen()) {
      ControlCmd cmd;
      cmd.v_mm_s = static_cast<int16_t>(std::max(-1.8, std::min(1.8, v)) * 1000.0);
      const double hardware_w = w * static_cast<double>(angular_command_sign_);
      cmd.w_mrad_s = static_cast<int16_t>(
          std::max(-9.0, std::min(9.0, hardware_w)) * 1000.0);
      const auto frame = packControl(cmd, ControlProtocol{});
      ser_.write(frame);
    }

    // ========== 5. 接收编码器 ==========
    if (!ser_.isOpen()) return;
    size_t n = ser_.available();
    if (n == 0) return;

    std::string data = ser_.read(n);
    for (auto c : data) rx_buf_.push_back(static_cast<uint8_t>(c));

    const size_t FRAME_LEN = 9;
    while (rx_buf_.size() >= FRAME_LEN) {
      if (!(rx_buf_[0] == 0xAA && rx_buf_[1] == 0x55)) {
        rx_buf_.erase(rx_buf_.begin());
        continue;
      }
      const std::vector<uint8_t> frame(rx_buf_.begin(), rx_buf_.begin() + FRAME_LEN);
      EncoderFeedback feedback;
      if (!unpackFeedback(frame, feedback, FeedbackProtocol{})) {
        rx_buf_.erase(rx_buf_.begin());
        continue;
      }

      std_msgs::String msg;
      std::ostringstream oss;
      oss << "enc_left=" << feedback.left << ", enc_right=" << feedback.right;
      msg.data = oss.str();
      fb_pub_.publish(msg);

      std_msgs::Int16MultiArray enc;
      enc.data.resize(2);
      enc.data[0] = feedback.left;
      enc.data[1] = feedback.right;
      enc_pub_.publish(enc);

      rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + FRAME_LEN);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher fb_pub_;
  ros::Publisher enc_pub_;
  ros::Publisher front_distance_pub_;
  ros::Publisher rear_distance_pub_;
  ros::Publisher nearest_distance_pub_;
  ros::Publisher safety_stop_pub_;
  ros::Publisher safety_collision_pub_;
  ros::Publisher sent_cmd_pub_;
  ros::Publisher lb_pressed_pub_;
  ros::Publisher control_source_pub_;
  ros::Timer loop_timer_;

  serial::Serial ser_;
  std::string port_;
  int baud_{115200};
  std::string joy_dev_;
  double max_v_{0.5};
  double max_w_{1.0};
  double deadzone_{0.1};
  int angular_command_sign_{-1};
  int joy_angular_axis_sign_{-1};

  bool safety_enable_{true};
  double safety_stop_distance_{0.15};
  double safety_slow_distance_{0.60};
  double safety_scan_timeout_{0.30};
  double safety_rear_stop_distance_{0.15};
  double safety_prediction_time_{0.60};
  double safety_prediction_step_{0.03};
  double safety_collision_padding_{0.03};
  bool navigation_allow_reverse_{false};
  double joystick_reverse_max_speed_{0.15};
  double vehicle_front_x_{0.3483};
  double vehicle_rear_x_{-0.1517};
  double vehicle_half_width_{0.24};
  double vehicle_footprint_padding_{0.02};
  double safety_half_width_{0.27};
  double laser_x_{0.315};
  double laser_y_{0.0};
  double laser_yaw_{0.0};
  double front_clearance_{std::numeric_limits<double>::infinity()};
  double rear_clearance_{std::numeric_limits<double>::infinity()};
  double nearest_obstacle_radius_{std::numeric_limits<double>::infinity()};
  std::vector<SafetyPoint> obstacle_points_;
  SafetyFootprint self_filter_footprint_;
  SafetyFootprint collision_footprint_;
  bool have_scan_{false};
  ros::Time last_scan_time_;

  int joy_fd_{-1};
  double joy_axes_[8]{0.0};
  int joy_buttons_[11]{0};

  double cmd_v_{0.0}, cmd_w_{0.0};
  ros::Time last_cmd_time_;

  std::vector<uint8_t> rx_buf_;

  // 终点检测
  bool goal_stopped_{false};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_bridge_node");
  SerialBridgeNode node;
  ros::spin();
  return 0;
}
