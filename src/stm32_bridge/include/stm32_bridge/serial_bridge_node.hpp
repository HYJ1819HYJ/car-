#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <sstream>

static uint8_t crc_xor(const uint8_t* d, size_t n){
  uint8_t c=0; for(size_t i=0;i<n;++i) c^=d[i]; return c;
}

class SerialBridge {
public:
  SerialBridge(): nh_("~"){
    nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
    nh_.param<int>("baud", baud_, 115200);
    nh_.param<double>("tx_hz", tx_hz_, 50.0);
    nh_.param<double>("wheel_base_m", wheel_base_m_, 0.23);
    nh_.param<int>("max_wheel_mm_s", max_mm_s_, 1200);
    nh_.param<int>("cmd_timeout_ms", cmd_timeout_ms_, 300);

    cmd_sub_ = nh_.subscribe("/cmd_vel", 20, &SerialBridge::cmdCb, this);
    fb_pub_  = nh_.advertise<std_msgs::String>("/chassis_feedback_text", 20);

    openPort();
    tx_timer_ = nh_.createTimer(ros::Duration(1.0/tx_hz_), &SerialBridge::txLoop, this);
    rx_timer_ = nh_.createTimer(ros::Duration(0.01), &SerialBridge::rxLoop, this);
    wd_timer_ = nh_.createTimer(ros::Duration(0.02), &SerialBridge::watchdogLoop, this);
    last_cmd_time_ = ros::Time::now();
  }

private:
  void openPort(){
    try{
      ser_.setPort(port_);
      ser_.setBaudrate(baud_);
      serial::Timeout to = serial::Timeout::simpleTimeout(static_cast<uint32_t>(20));
      ser_.setTimeout(to);
      ser_.open();
      ROS_INFO("Serial opened: %s @ %d", port_.c_str(), baud_);
    }catch(const std::exception& e){
      ROS_FATAL("Serial open failed: %s", e.what());
      ros::shutdown();
    }
  }

  void cmdCb(const geometry_msgs::Twist::ConstPtr& msg){
    // 差速运动学: v,w -> left,right
    // v[m/s], w[rad/s], L[m]
    const double v = msg->linear.x;
    const double w = msg->angular.z;
    double vl = v - 0.5 * wheel_base_m_ * w;
    double vr = v + 0.5 * wheel_base_m_ * w;

    left_mm_s_  = (int16_t)std::max(-max_mm_s_, std::min(max_mm_s_, (int)(vl*1000.0)));
    right_mm_s_ = (int16_t)std::max(-max_mm_s_, std::min(max_mm_s_, (int)(vr*1000.0)));
    last_cmd_time_ = ros::Time::now();
  }

  std::vector<uint8_t> packLR(int16_t l, int16_t r){
    std::vector<uint8_t> f(7,0);
    f[0]=0x5A; f[1]=0xA5;
    f[2]=(uint8_t)((l>>8)&0xFF); f[3]=(uint8_t)(l&0xFF);
    f[4]=(uint8_t)((r>>8)&0xFF); f[5]=(uint8_t)(r&0xFF);
    f[6]=crc_xor(f.data(),6);
    return f;
  }

  void txLoop(const ros::TimerEvent&){
    if(!ser_.isOpen()) return;
    auto f = packLR(left_mm_s_, right_mm_s_);
    ser_.write(f);
  }

  void rxLoop(const ros::TimerEvent&){
    if(!ser_.isOpen()) return;
    size_t n = ser_.available();
    if(n==0) return;
    std::string s = ser_.read(n);
    for(auto c: s) rx_.push_back((uint8_t)c);

    // 回传固定8字节示例: 5A 5A m1H m1L m2H m2L state crc
    while(rx_.size() >= 8){
      if(!(rx_[0]==0x5A && rx_[1]==0x5A)){ rx_.erase(rx_.begin()); continue; }
      uint8_t c = crc_xor(rx_.data(),7);
      if(c != rx_[7]){ rx_.erase(rx_.begin()); continue; }

      int16_t m1 = (int16_t)((rx_[2]<<8)|rx_[3]);
      int16_t m2 = (int16_t)((rx_[4]<<8)|rx_[5]);
      uint8_t st = rx_[6];

      std_msgs::String out;
      std::ostringstream oss;
      oss << "m1="<<m1<<" mm/s, m2="<<m2<<" mm/s, state="<<(int)st;
      out.data = oss.str();
      fb_pub_.publish(out);

      rx_.erase(rx_.begin(), rx_.begin()+8);
    }
  }

  void watchdogLoop(const ros::TimerEvent&){
    double dt = (ros::Time::now()-last_cmd_time_).toSec();
    if(dt > cmd_timeout_ms_/1000.0){
      left_mm_s_=0; right_mm_s_=0;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;
  ros::Publisher fb_pub_;
  ros::Timer tx_timer_, rx_timer_, wd_timer_;
  serial::Serial ser_;

  std::string port_;
  int baud_{115200};
  double tx_hz_{50.0};
  double wheel_base_m_{0.23};
  int max_mm_s_{1200};
  int cmd_timeout_ms_{300};

  int16_t left_mm_s_{0}, right_mm_s_{0};
  ros::Time last_cmd_time_;
  std::vector<uint8_t> rx_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "serial_bridge_node");
  SerialBridge n;
  ros::spin();
  return 0;
}