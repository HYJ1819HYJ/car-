#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cstdint>
#include <cmath>
#include <limits>

/**
 * 里程计发布节点
 * 订阅 /wheel_encoder 编码器数据，计算里程并发布 /odom
 * 
 * 参数：
 *   ~wheel_diameter    有效轮径 (m)    默认 0.20894（实车标定）
 *   ~wheel_base        有效轮距 (m)    默认 0.51375（实车标定）
 *   ~ticks_per_rev     编码器每圈脉冲   默认 4096
 *   ~left_encoder_sign 左编码器方向     默认 1
 *   ~right_encoder_sign右编码器方向     默认 1
 *   ~max_delta_ticks   单帧最大合理增量 默认 10000
 */

class OdomPublisher {
public:
  OdomPublisher() : nh_("~") {
    nh_.param<double>("wheel_diameter", wheel_diameter_, 0.20894);
    nh_.param<double>("wheel_base", wheel_base_, 0.51375);
    nh_.param<int>("ticks_per_rev", ticks_per_rev_, 4096);
    nh_.param<int>("left_encoder_sign", left_encoder_sign_, 1);
    nh_.param<int>("right_encoder_sign", right_encoder_sign_, 1);
    nh_.param<int>("max_delta_ticks", max_delta_ticks_, 10000);

    if (!(wheel_diameter_ > 0.0) || !(wheel_base_ > 0.0) ||
        ticks_per_rev_ <= 0 || max_delta_ticks_ <= 0) {
      ROS_FATAL("Invalid odometry parameters: wheel_diameter, wheel_base, "
                "ticks_per_rev and max_delta_ticks must all be positive");
      ros::shutdown();
      return;
    }
    left_encoder_sign_ = left_encoder_sign_ < 0 ? -1 : 1;
    right_encoder_sign_ = right_encoder_sign_ < 0 ? -1 : 1;

    enc_sub_ = nh_.subscribe("/wheel_encoder", 10, &OdomPublisher::encCb, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 50);

    // 计算脉冲→距离的转换因子
    // 一圈 = PI * diameter 米
    // 一圈 = ticks_per_rev 脉冲
    // 每个脉冲 = PI * diameter / ticks_per_rev 米
    tick_to_m_ = (M_PI * wheel_diameter_) / static_cast<double>(ticks_per_rev_);

    ROS_INFO("Odometry: diameter=%.6f m, base=%.6f m, ticks/rev=%d, signs=(%d,%d)",
             wheel_diameter_, wheel_base_, ticks_per_rev_,
             left_encoder_sign_, right_encoder_sign_);
  }

private:
  // 计算 16 位累计计数器的最短有符号差值，正确处理 32767/-32768 回绕。
  static int32_t wrappedDelta(int16_t current, int16_t previous) {
    int32_t delta = static_cast<int32_t>(static_cast<uint16_t>(current)) -
                    static_cast<int32_t>(static_cast<uint16_t>(previous));
    if (delta > std::numeric_limits<int16_t>::max()) delta -= 65536;
    if (delta < std::numeric_limits<int16_t>::min()) delta += 65536;
    return delta;
  }

  void encCb(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 2) return;

    // 电机驱动器反馈通道与车体物理左右相反，在里程计入口统一交换。
    int16_t enc_left = msg->data[1];
    int16_t enc_right = msg->data[0];

    const ros::Time now = ros::Time::now();

    // 第一帧只建立累计编码器基准，不能把上电以来的累计值算成一次位移。
    if (!encoder_initialized_) {
      last_enc_left_ = enc_left;
      last_enc_right_ = enc_right;
      last_time_ = now;
      encoder_initialized_ = true;
      ROS_INFO("Encoder baseline initialized: left=%d right=%d",
               static_cast<int>(enc_left), static_cast<int>(enc_right));
      return;
    }

    const double dt = (now - last_time_).toSec();
    if (dt <= 0.0) {
      ROS_WARN_THROTTLE(1.0, "Ignoring encoder sample with non-positive dt");
      return;
    }

    // 计算编码器增量
    const int32_t d_left_raw = wrappedDelta(enc_left, last_enc_left_);
    const int32_t d_right_raw = wrappedDelta(enc_right, last_enc_right_);

    // 串口重连、驱动器重启或坏帧可能产生不可能的跳变；丢弃该帧并重建基准。
    if (std::abs(d_left_raw) > max_delta_ticks_ ||
        std::abs(d_right_raw) > max_delta_ticks_) {
      ROS_WARN("Rejecting implausible encoder jump: left=%d right=%d",
               static_cast<int>(d_left_raw), static_cast<int>(d_right_raw));
      last_enc_left_ = enc_left;
      last_enc_right_ = enc_right;
      last_time_ = now;
      return;
    }

    const int32_t d_left = left_encoder_sign_ * d_left_raw;
    const int32_t d_right = right_encoder_sign_ * d_right_raw;

    // 计算左右轮移动距离 (米)
    double dist_left = d_left * tick_to_m_;
    double dist_right = d_right * tick_to_m_;

    // 计算位移和转角
    double dist_center = (dist_left + dist_right) / 2.0;
    double delta_theta = (dist_right - dist_left) / wheel_base_;

    // 更新位置
    x_ += dist_center * cos(theta_ + delta_theta / 2.0);
    y_ += dist_center * sin(theta_ + delta_theta / 2.0);
    theta_ += delta_theta;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    // 计算速度
    double vx = dist_center / dt;
    double vth = delta_theta / dt;

    // 发布里程计
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

    nav_msgs::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = 0.01;
    odom.twist.covariance[0] = 0.02;
    odom.twist.covariance[7] = 0.02;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = 0.02;

    odom_pub_.publish(odom);

    // 广播 TF: odom → base_footprint
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = now;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom_quat;
    tf_br_.sendTransform(transform);

    last_enc_left_ = enc_left;
    last_enc_right_ = enc_right;
    last_time_ = now;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber enc_sub_;
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_br_;

  double wheel_diameter_;
  double wheel_base_;
  int ticks_per_rev_;
  int left_encoder_sign_;
  int right_encoder_sign_;
  int max_delta_ticks_;
  double tick_to_m_;

  int16_t last_enc_left_{0};
  int16_t last_enc_right_{0};
  ros::Time last_time_;
  bool encoder_initialized_{false};

  double x_{0.0}, y_{0.0}, theta_{0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_publisher");
  OdomPublisher node;
  ros::spin();
  return 0;
}
