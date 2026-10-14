#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <stm32_bridge/BlueTargetObservation.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr uint8_t kTargetHeader = 0xAA;
constexpr uint8_t kCommandHeader = 0xAB;
constexpr uint8_t kCommandDisable = 0x00;
constexpr uint8_t kCommandEnable = 0x01;
constexpr uint8_t kCommandCenter = 0x02;
constexpr uint8_t kCommandStatus = 0x03;
constexpr uint16_t kFirmwareWidth = 640;
constexpr uint16_t kFirmwareHeight = 480;

double degToRad(double value) { return value * M_PI / 180.0; }

bool parseIntegerField(const std::string& line, const std::string& key, long* value) {
  const std::string token = key + "=";
  const std::size_t begin = line.find(token);
  if (begin == std::string::npos) return false;
  const char* text = line.c_str() + begin + token.size();
  char* end = nullptr;
  const long parsed = std::strtol(text, &end, 10);
  if (end == text) return false;
  *value = parsed;
  return true;
}

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

    if (send_rate_ < 1.0) send_rate_ = 1.0;
    if (reconnect_period_ < 0.2) reconnect_period_ = 0.2;
    pan_to_ros_sign_ = pan_to_ros_sign_ < 0.0 ? -1.0 : 1.0;
    tilt_to_ros_sign_ = tilt_to_ros_sign_ < 0.0 ? -1.0 : 1.0;

    observation_sub_ = nh_.subscribe("/blue_target/observation", 5,
                                     &GimbalBridgeNode::observationCb, this);
    connected_pub_ = nh_.advertise<std_msgs::Bool>("/gimbal/connected", 1, true);
    ready_pub_ = nh_.advertise<std_msgs::Bool>("/gimbal/servo_ready", 1, true);
    locked_pub_ = nh_.advertise<std_msgs::Bool>("/gimbal/target_locked", 1, true);
    searching_pub_ = nh_.advertise<std_msgs::Bool>("/gimbal/searching", 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>("/gimbal/status_text", 10);
    joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/gimbal/joint_states", 10);

    timer_ = nh_.createTimer(ros::Duration(0.02), &GimbalBridgeNode::timerCb, this);
    publishFlags();
    ROS_INFO("Gimbal bridge: %s @ %d, required=%s", port_.c_str(), baud_,
             required_ ? "true" : "false");
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
      connection_time_ = now;
      startup_stage_ = 0;
      rx_line_.clear();
      ROS_INFO("Gimbal serial connected: %s", port_.c_str());
      if (center_on_connect_) {
        sendCommand(kCommandCenter);
        startup_stage_ = 1;
      } else if (auto_enable_) {
        sendCommand(kCommandEnable);
        startup_stage_ = 2;
      }
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

  void transmitObservation() {
    if (!serial_.isOpen()) return;
    const ros::Time now = ros::Time::now();
    if ((now - last_target_send_).toSec() < 1.0 / send_rate_) return;
    last_target_send_ = now;

    const bool fresh = have_observation_ &&
        (now - observation_time_).toSec() <= 0.30;
    if (!fresh || !latest_observation_.detected ||
        latest_observation_.image_width == 0 || latest_observation_.image_height == 0 ||
        latest_observation_.pixel_width < 1.0f || latest_observation_.pixel_height < 1.0f) {
      sendTarget(false, 0, 0);
      return;
    }

    double source_x = latest_observation_.center_x;
    if (invert_target_x_) {
      source_x = static_cast<double>(latest_observation_.image_width - 1U) - source_x;
    }
    const double scaled_x = source_x * kFirmwareWidth /
                            latest_observation_.image_width;
    const double scaled_y = latest_observation_.center_y * kFirmwareHeight /
                            latest_observation_.image_height;
    const uint16_t x = static_cast<uint16_t>(std::max(0.0, std::min(639.0, scaled_x)));
    const uint16_t y = static_cast<uint16_t>(std::max(0.0, std::min(479.0, scaled_y)));
    sendTarget(true, x, y);
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
          if (!rx_line_.empty()) parseLine(rx_line_);
          rx_line_.clear();
        } else if (rx_line_.size() < 512) {
          rx_line_.push_back(byte);
        } else {
          rx_line_.clear();
        }
      }
    } catch (const std::exception& error) {
      handleDisconnect(error.what());
    }
  }

  void parseLine(const std::string& line) {
    std_msgs::String text;
    text.data = line;
    status_pub_.publish(text);
    if (line.compare(0, 5, "STAT,") != 0) {
      ROS_INFO("Gimbal: %s", line.c_str());
      return;
    }

    long value = 0;
    if (parseIntegerField(line, "ready", &value)) servo_ready_ = value != 0;
    if (parseIntegerField(line, "lock", &value)) target_locked_ = value != 0;
    if (parseIntegerField(line, "search", &value)) searching_ = value != 0;
    if (parseIntegerField(line, "bearing10", &value)) pan_deg_ = value / 10.0;
    if (parseIntegerField(line, "tilt10", &value)) tilt_deg_ = value / 10.0;
    last_status_time_ = ros::Time::now();
    publishFlags();
    publishJointState();
  }

  void publishFlags() {
    std_msgs::Bool value;
    value.data = connected_;
    connected_pub_.publish(value);
    value.data = servo_ready_;
    ready_pub_.publish(value);
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
      const ros::Time now = ros::Time::now();
      if (startup_stage_ == 1 && auto_enable_ &&
          (now - connection_time_).toSec() >= 0.8) {
        sendCommand(kCommandEnable);
        startup_stage_ = 2;
      }
      // The MCU stores only one pending command until its main loop runs.
      // Do not overwrite center/enable with a status query immediately after it.
      if ((now - last_status_request_).toSec() >= 1.0 &&
          (now - last_control_command_).toSec() >= 0.2) {
        sendCommand(kCommandStatus);
        last_status_request_ = now;
      }
      transmitObservation();
      readSerial();
      publishCameraTf();
      if ((now - last_status_time_).toSec() > 2.0) {
        servo_ready_ = false;
        target_locked_ = false;
        searching_ = false;
        publishFlags();
      }
    } catch (const std::exception& error) {
      handleDisconnect(error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber observation_sub_;
  ros::Publisher connected_pub_;
  ros::Publisher ready_pub_;
  ros::Publisher locked_pub_;
  ros::Publisher searching_pub_;
  ros::Publisher status_pub_;
  ros::Publisher joint_pub_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  serial::Serial serial_;
  ros::Time last_control_command_;

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

  bool connected_{false};
  bool servo_ready_{false};
  bool target_locked_{false};
  bool searching_{false};
  int startup_stage_{0};
  double pan_deg_{0.0};
  double tilt_deg_{0.0};
  std::string rx_line_;
  stm32_bridge::BlueTargetObservation latest_observation_;
  bool have_observation_{false};
  ros::Time observation_time_;
  ros::Time last_connect_attempt_;
  ros::Time connection_time_;
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
