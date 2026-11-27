#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <stm32_bridge/BlueTargetObservation.h>
#include <stm32_bridge/gimbal_status.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr uint8_t kTargetHeader = 0xAA;
constexpr uint8_t kCommandHeader = 0xAB;
constexpr uint8_t kCommandDisable = 0x00;
constexpr uint8_t kCommandEnable = 0x01;
constexpr uint8_t kCommandCenter = 0x02;
constexpr uint8_t kCommandStatus = 0x03;
constexpr uint8_t kCommandSearchDirPos = 0x04;
constexpr uint8_t kCommandSearchDirNeg = 0x05;
constexpr uint16_t kFirmwareWidth = 640;
constexpr uint16_t kFirmwareHeight = 480;

double degToRad(double value) { return value * M_PI / 180.0; }

}  // namespace

class GimbalBridgeNode {
 public:
  GimbalBridgeNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("port", port_, "/dev/ttyUSB1");
    pnh_.param<int>("baud", baud_, 115200);
    pnh_.param<bool>("required", required_, false);
    pnh_.param<bool>("auto_enable", auto_enable_, true);
    pnh_.param<bool>("center_on_connect", center_on_connect_, true);
    pnh_.param<double>("send_rate", send_rate_, 20.0);
    pnh_.param<double>("reconnect_period", reconnect_period_, 1.0);
    pnh_.param<double>("pan_to_ros_sign", pan_to_ros_sign_, 1.0);
    pnh_.param<double>("tilt_to_ros_sign", tilt_to_ros_sign_, -1.0);
    // The pan servo on this robot is installed with the opposite mechanical
    // direction to the firmware's image-error convention.  Mirror only the
    // target X coordinate so tracking converges without changing the image or
    // requiring another STM32 flash.
    pnh_.param<bool>("invert_target_x", invert_target_x_, true);
    pnh_.param<bool>("publish_camera_tf", publish_camera_tf_, true);
    pnh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    pnh_.param<std::string>("camera_frame", camera_frame_, "usb_cam");
    pnh_.param<double>("camera_x", camera_x_, 0.145);
    pnh_.param<double>("camera_y", camera_y_, 0.0);
    pnh_.param<double>("camera_z", camera_z_, 0.42);
    pnh_.param<double>("camera_roll", camera_roll_, 0.0);
    pnh_.param<double>("camera_pitch_offset", camera_pitch_offset_, 0.0);
    pnh_.param<double>("camera_yaw_offset", camera_yaw_offset_, 0.0);

    bool require_checksum = false;
    pnh_.param("require_status_checksum", require_checksum, false);
    status_parser_ = stm32_bridge::GimbalStatusParser(require_checksum);

    stm32_bridge::GimbalTiltConfig tilt_cfg;
    pnh_.param<double>("tilt_min_deg", tilt_cfg.min_tilt_deg, -25.0);
    pnh_.param<double>("tilt_max_deg", tilt_cfg.max_tilt_deg, 0.0);
    pnh_.param<double>("default_tilt_deg", tilt_cfg.default_tilt_deg, -10.0);
    pnh_.param<double>("min_target_down_deg", tilt_cfg.min_target_down_deg, -4.0);
    pnh_.param<double>("fallback_hfov_deg", fallback_hfov_deg_, 88.0);
    pnh_.param("yaw_feedforward_sec", yaw_feedforward_sec_, .12);
    pnh_.param("max_yaw_feedforward_px", max_yaw_feedforward_px_, 120.0);
    pnh_.param("avoid_status_tx_window", avoid_status_tx_window_, true);
    pnh_.param("status_period_est_sec", status_period_est_sec_, .516);
    pnh_.param("turn_search_bias_enabled", turn_search_bias_enabled_, true);
    yaw_feedforward_sec_ = std::max(0.0, yaw_feedforward_sec_);
    max_yaw_feedforward_px_ = std::max(0.0, max_yaw_feedforward_px_);
    status_period_est_sec_ = std::max(.05, status_period_est_sec_);
    tilt_controller_.configure(tilt_cfg);

    if (send_rate_ < 1.0) send_rate_ = 1.0;
    if (reconnect_period_ < 0.2) reconnect_period_ = 0.2;
    fallback_hfov_deg_ = std::max(20.0, std::min(150.0, fallback_hfov_deg_));
    pan_to_ros_sign_ = pan_to_ros_sign_ < 0.0 ? -1.0 : 1.0;
    tilt_to_ros_sign_ = tilt_to_ros_sign_ < 0.0 ? -1.0 : 1.0;

    observation_sub_ = nh_.subscribe("/blue_target/observation", 5,
                                     &GimbalBridgeNode::observationCb, this);
    odom_sub_ = nh_.subscribe("/odom", 1, &GimbalBridgeNode::odomCb, this);
    connected_pub_ = nh_.advertise<std_msgs::Bool>("/gimbal/connected", 1, true);
    ready_pub_ = nh_.advertise<std_msgs::Bool>("/gimbal/servo_ready", 1, true);
    enabled_pub_ = nh_.advertise<std_msgs::Bool>("/gimbal/enabled", 1, true);
    autonomy_sub_ = nh_.subscribe("/blue_autonomy/enable", 2, &GimbalBridgeNode::autonomyCb, this);
    center_service_ = nh_.advertiseService("/gimbal/center", &GimbalBridgeNode::centerCb, this);
    locked_pub_ = nh_.advertise<std_msgs::Bool>("/gimbal/target_locked", 1, true);
    searching_pub_ = nh_.advertise<std_msgs::Bool>("/gimbal/searching", 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>("/gimbal/status_text", 10);
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/gimbal/joint_states", 10);

    timer_ = nh_.createTimer(ros::Duration(0.02), &GimbalBridgeNode::timerCb, this);
    publishFlags();
    ROS_INFO("Gimbal bridge: %s @ %d, required=%s, tilt=[%.1f, %.1f] deg, default=%.1f deg, min_down=%.1f deg",
             port_.c_str(), baud_, required_ ? "true" : "false",
             tilt_controller_.config().min_tilt_deg,
             tilt_controller_.config().max_tilt_deg,
             tilt_controller_.config().default_tilt_deg,
             tilt_controller_.config().min_target_down_deg);
  }

  ~GimbalBridgeNode() {
    if (serial_.isOpen()) {
      try {
        sendCommand(kCommandDisable);
      } catch (...) {
      }
      serial_.close();
    }
  }

 private:
  void connectSerial() {
    const ros::Time now = ros::Time::now();
    if ((now - last_connect_attempt_).toSec() < reconnect_period_) return;
    last_connect_attempt_ = now;

    try {
      serial_.setPort(port_);
      serial_.setBaudrate(static_cast<uint32_t>(baud_));
      serial::Timeout timeout = serial::Timeout::simpleTimeout(20);
      serial_.setTimeout(timeout);
      serial_.open();
      serial_.flush();
      connected_ = true;
      startup_.reset(center_on_connect_);
      tilt_controller_.reset();
      if (center_on_connect_) tilt_controller_.requestLevel();
      last_status_time_ = now;
      have_status_ = false;
      rx_line_.clear();
      rx_overflow_ = false;
      ROS_INFO("Gimbal serial connected: %s", port_.c_str());
      publishFlags();
    } catch (const std::exception& error) {
      connected_ = false;
      if (serial_.isOpen()) serial_.close();
      if (required_) {
        ROS_ERROR_THROTTLE(2.0, "Required gimbal serial unavailable (%s): %s",
                           port_.c_str(), error.what());
      } else {
        ROS_WARN_THROTTLE(5.0, "Gimbal serial unavailable (%s): %s; fixed-camera mode remains usable",
                          port_.c_str(), error.what());
      }
      publishFlags();
    }
  }

  void handleDisconnect(const std::string& reason) {
    if (connected_) ROS_ERROR("Gimbal serial disconnected: %s", reason.c_str());
    connected_ = false;
    servo_ready_ = false;
    gimbal_enabled_ = false;
    target_locked_ = false;
    searching_ = false;
    if (serial_.isOpen()) serial_.close();
    publishFlags();
  }

  void sendCommand(uint8_t command) {
    if (!serial_.isOpen()) return;
    std::vector<uint8_t> frame(3);
    frame[0] = kCommandHeader;
    frame[1] = command;
    frame[2] = static_cast<uint8_t>(frame[0] + frame[1]);
    serial_.write(frame);
    if (command != kCommandStatus) last_control_command_ = ros::Time::now();
  }

  void autonomyCb(const std_msgs::Bool::ConstPtr& msg) {
    if (autonomy_enabled_ && !msg->data && connected_ && servo_ready_) {
      startup_.requestCenter();
      tilt_controller_.requestLevel();
    }
    autonomy_enabled_ = msg->data;
  }

  bool centerCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& reply) {
    reply.success = connected_ && servo_ready_ && !autonomy_enabled_;
    if (reply.success) {
      startup_.requestCenter();
      tilt_controller_.requestLevel();
    }
    reply.message = reply.success ? "Gimbal center and pitch level queued" :
        "Gimbal must be online, ready and in standby";
    return true;
  }

  void sendTarget(bool found, uint16_t x, uint16_t y) {
    if (!serial_.isOpen()) return;
    if (!found) {
      x = 0xFFFF;
      y = 0xFFFF;
    }
    std::vector<uint8_t> frame(6);
    frame[0] = kTargetHeader;
    frame[1] = static_cast<uint8_t>(x & 0xFF);
    frame[2] = static_cast<uint8_t>((x >> 8) & 0xFF);
    frame[3] = static_cast<uint8_t>(y & 0xFF);
    frame[4] = static_cast<uint8_t>((y >> 8) & 0xFF);
    frame[5] = static_cast<uint8_t>(frame[0] + frame[1] + frame[2] + frame[3] + frame[4]);
    serial_.write(frame);
  }

  void observationCb(const stm32_bridge::BlueTargetObservation::ConstPtr& msg) {
    latest_observation_ = *msg;
    have_observation_ = true;
    observation_time_ = ros::Time::now();
  }

  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    const double wz = msg->twist.twist.angular.z;
    odom_wz_ = std::isfinite(wz) ? wz : 0.0;
    last_odom_time_ = ros::Time::now();
  }

  double freshOdomYawRate(const ros::Time& now) const {
    const double age = (now - last_odom_time_).toSec();
    return !last_odom_time_.isZero() && age >= 0.0 && age <= .30 ? odom_wz_ : 0.0;
  }

  void transmitObservation() {
    if (!serial_.isOpen()) return;
    const ros::Time now = ros::Time::now();
    if ((now - last_target_send_).toSec() < 1.0 / send_rate_) return;
    const double since_stat = (now - last_status_time_).toSec();
    if (avoid_status_tx_window_ && have_status_ &&
        std::abs(since_stat - status_period_est_sec_) <= .025) return;
    last_target_send_ = now;

    const bool fresh = have_observation_ &&
        (now - observation_time_).toSec() <= 0.30;
    const bool has_observation = fresh && latest_observation_.detected &&
        latest_observation_.image_width > 0 && latest_observation_.image_height > 0 &&
        latest_observation_.pixel_width >= 1.0f && latest_observation_.pixel_height >= 1.0f;

    double scaled_x = 320.0;
    double scaled_y = 240.0;
    double target_down_deg = std::numeric_limits<double>::quiet_NaN();
    if (has_observation) {
      const double hfov_rad = degToRad(fallback_hfov_deg_);
      const double focal_px = latest_observation_.image_width / (2.0 * std::tan(0.5 * hfov_rad));
      const double wz = freshOdomYawRate(now);
      double source_x = stm32_bridge::applyYawFeedforwardPx(
          latest_observation_.center_x, focal_px, wz, yaw_feedforward_sec_,
          max_yaw_feedforward_px_, latest_observation_.image_width);
      if (invert_target_x_) {
        source_x = static_cast<double>(latest_observation_.image_width - 1U) - source_x;
      }
      scaled_x = source_x * kFirmwareWidth / latest_observation_.image_width;
      scaled_y = latest_observation_.center_y * kFirmwareHeight /
                 latest_observation_.image_height;

      const double cy = 0.5 * latest_observation_.image_height;
      const double pixel_down_deg =
          std::atan2(latest_observation_.center_y - cy, focal_px) * 180.0 / M_PI;
      const double camera_down_deg =
          tilt_to_ros_sign_ * tilt_deg_ + camera_pitch_offset_ * 180.0 / M_PI;
      target_down_deg = camera_down_deg + pixel_down_deg;
    }

    const auto cmd = tilt_controller_.compute(
        now.toSec(), gimbal_enabled_, servo_ready_, tilt_deg_,
        has_observation, scaled_x, scaled_y, target_down_deg, autonomy_enabled_);
    sendTarget(cmd.found, cmd.x, cmd.y);
  }

  void readSerial() {
    if (!serial_.isOpen()) return;
    try {
      const std::size_t available = serial_.available();
      if (available == 0) return;
      const std::string bytes = serial_.read(available);
      for (char byte : bytes) {
        if (byte == '\n') {
          if (!rx_line_.empty() && rx_line_.back() == '\r') rx_line_.pop_back();
          if (rx_overflow_) rejectLine();
          else if (!rx_line_.empty()) parseLine(rx_line_);
          rx_line_.clear();
          rx_overflow_ = false;
        } else if (!rx_overflow_) {
          if (rx_line_.size() < 512) rx_line_.push_back(byte);
          else rx_overflow_ = true;
        }
      }
    } catch (const std::exception& error) {
      handleDisconnect(error.what());
    }
  }

  void rejectLine() {
    ++bad_lines_;
    ROS_WARN_THROTTLE(2.0, "Discarded invalid gimbal serial line (%zu total)", bad_lines_);
  }

  void parseLine(const std::string& line) {
    if (!stm32_bridge::printableGimbalLine(line)) {
      rejectLine();
      return;
    }
    if (line.find("STAT,") != std::string::npos || line.compare(0, 4, "STAT") == 0) {
      stm32_bridge::GimbalStatus status;
      if (!status_parser_.parse(line, &status)) {
        rejectLine();
        return;
      }
      gimbal_enabled_ = status.enabled;
      servo_ready_ = status.ready;
      target_locked_ = status.locked;
      searching_ = status.searching;
      pan_deg_ = status.pan_deg;
      tilt_deg_ = status.tilt_deg;
      tilt_controller_.onStatus(tilt_deg_);
      last_status_time_ = ros::Time::now();
      have_status_ = true;
      publishFlags();
      publishJointState();
    } else {
      ROS_INFO("Gimbal: %s", line.c_str());
    }
    std_msgs::String text;
    text.data = line;
    status_pub_.publish(text);
  }

  void publishFlags() {
    std_msgs::Bool value;
    value.data = connected_;
    connected_pub_.publish(value);
    value.data = servo_ready_;
    ready_pub_.publish(value);
    value.data = gimbal_enabled_;
    enabled_pub_.publish(value);
    value.data = target_locked_;
    locked_pub_.publish(value);
    value.data = searching_;
    searching_pub_.publish(value);
  }

  void publishJointState() {
    sensor_msgs::JointState joints;
    joints.header.stamp = ros::Time::now();
    joints.name.push_back("camera_pan_joint");
    joints.name.push_back("camera_tilt_joint");
    joints.position.push_back(pan_to_ros_sign_ * degToRad(pan_deg_));
    joints.position.push_back(tilt_to_ros_sign_ * degToRad(tilt_deg_));
    joint_pub_.publish(joints);
  }

  void publishCameraTf() {
    if (!publish_camera_tf_) return;
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = camera_frame_;
    transform.transform.translation.x = camera_x_;
    transform.transform.translation.y = camera_y_;
    transform.transform.translation.z = camera_z_;
    tf2::Quaternion rotation;
    rotation.setRPY(camera_roll_,
                    camera_pitch_offset_ + tilt_to_ros_sign_ * degToRad(tilt_deg_),
                    camera_yaw_offset_ + pan_to_ros_sign_ * degToRad(pan_deg_));
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    tf_broadcaster_.sendTransform(transform);
  }

  void timerCb(const ros::TimerEvent&) {
    if (!serial_.isOpen()) {
      connectSerial();
      publishCameraTf();
      return;
    }

    try {
      readSerial();
      if (!connected_) return;
      const ros::Time now = ros::Time::now();
      const double status_age = (now - last_status_time_).toSec();
      if (status_age > 2.0 && (servo_ready_ || gimbal_enabled_ || target_locked_ || searching_)) {
        servo_ready_ = gimbal_enabled_ = target_locked_ = searching_ = false;
        publishFlags();
      }
      const auto action = startup_.next(now.toSec(), servo_ready_, auto_enable_,
                                        gimbal_enabled_, tilt_deg_,
                                        tilt_controller_.config().default_tilt_deg);
      using Action = stm32_bridge::GimbalStartup::Action;
      if (action == Action::Center || action == Action::Enable) {
        if (auto_enable_ && !gimbal_enabled_)
          ROS_WARN_THROTTLE(2.0, "Gimbal mode=0 with auto-enable; restoring tracking");
        if (action == Action::Center) tilt_controller_.requestLevel();
        sendCommand(action == Action::Center ? kCommandCenter : kCommandEnable);
      }
      // Status requests share the MCU's pending control command slot.
      if (status_age > 0.6 && (now - last_status_request_).toSec() >= 1.0 &&
          (now - last_control_command_).toSec() >= 0.2) {
        sendCommand(kCommandStatus);
        last_status_request_ = now;
      } else if (turn_search_bias_enabled_ && status_age >= 0.0 && status_age <= .6 &&
                 servo_ready_ && gimbal_enabled_ && !target_locked_ &&
                 (now - last_search_bias_).toSec() >= .8 &&
                 (now - last_control_command_).toSec() >= .2 &&
                 (now - last_status_request_).toSec() >= .2 &&
                 (!avoid_status_tx_window_ ||
                  std::abs(status_age - status_period_est_sec_) > .025)) {
        const int direction = stm32_bridge::desiredSearchDirectionForTurn(
            autonomy_enabled_, searching_, freshOdomYawRate(now),
            pan_to_ros_sign_ * pan_deg_, pan_to_ros_sign_);
        if (direction != 0) {
          sendCommand(direction > 0 ? kCommandSearchDirPos : kCommandSearchDirNeg);
          last_search_bias_ = now;
        }
      }
      transmitObservation();
      publishCameraTf();
    } catch (const std::exception& error) {
      handleDisconnect(error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber observation_sub_, autonomy_sub_, odom_sub_;
  ros::ServiceServer center_service_;
  ros::Publisher connected_pub_;
  ros::Publisher ready_pub_, enabled_pub_;
  ros::Publisher locked_pub_;
  ros::Publisher searching_pub_;
  ros::Publisher status_pub_;
  ros::Publisher joint_pub_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  serial::Serial serial_;
  ros::Time last_control_command_, last_search_bias_;

  std::string port_;
  int baud_{115200};
  bool required_{false};
  bool auto_enable_{true};
  bool center_on_connect_{true};
  double send_rate_{20.0};
  double reconnect_period_{1.0};
  double pan_to_ros_sign_{1.0};
  double tilt_to_ros_sign_{-1.0};
  bool invert_target_x_{true};
  bool publish_camera_tf_{true};
  std::string base_frame_{"base_footprint"};
  std::string camera_frame_{"usb_cam"};
  double camera_x_{0.145};
  double camera_y_{0.0};
  double camera_z_{0.42};
  double camera_roll_{0.0};
  double camera_pitch_offset_{0.0};
  double camera_yaw_offset_{0.0};
  double fallback_hfov_deg_{88.0};
  double yaw_feedforward_sec_{.12}, max_yaw_feedforward_px_{120.0}, odom_wz_{0.0};
  double status_period_est_sec_{.516};
  bool avoid_status_tx_window_{true}, have_status_{false};
  bool turn_search_bias_enabled_{true};
  ros::Time last_odom_time_;

  bool connected_{false};
  bool servo_ready_{false};
  bool target_locked_{false};
  bool searching_{false};
  bool gimbal_enabled_{false}, autonomy_enabled_{false}, rx_overflow_{false};
  std::size_t bad_lines_{0};
  stm32_bridge::GimbalStartup startup_;
  stm32_bridge::GimbalStatusParser status_parser_;
  stm32_bridge::GimbalTiltController tilt_controller_;
  double pan_deg_{0.0};
  double tilt_deg_{0.0};
  std::string rx_line_;
  stm32_bridge::BlueTargetObservation latest_observation_;
  bool have_observation_{false};
  ros::Time observation_time_;
  ros::Time last_connect_attempt_;
  ros::Time last_target_send_;
  ros::Time last_status_request_;
  ros::Time last_status_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gimbal_bridge");
  GimbalBridgeNode node;
  ros::spin();
  return 0;
}
