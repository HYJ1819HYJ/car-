#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include <algorithm>

struct CmdSample {
  double t{0.0};
  double v{0.0};
  double w{0.0};
};

class TeachRepeatNode {
public:
  TeachRepeatNode() : nh_("~") {
    nh_.param<std::string>("mode", mode_, "record");
    nh_.param<double>("loop_hz", loop_hz_, 20.0);
    nh_.param<std::string>("cmd_in_topic", cmd_in_topic_, "/cmd_vel_in");
    nh_.param<std::string>("cmd_out_topic", cmd_out_topic_, "/cmd_vel");
    nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    nh_.param<double>("front_stop_m", front_stop_m_, 0.35);
    nh_.param<bool>("safety_enable", safety_enable_, true);
    nh_.param<double>("replay_speed_scale", replay_speed_scale_, 1.0);
    nh_.param<std::string>("record_path", record_path_, defaultRecordPath());

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_out_topic_, 10);
    scan_sub_ = nh_.subscribe(scan_topic_, 10, &TeachRepeatNode::scanCb, this);

    if (mode_ == "record") {
      cmd_sub_ = nh_.subscribe(cmd_in_topic_, 20, &TeachRepeatNode::cmdCb, this);
      openRecordFile();
    } else if (mode_ == "replay") {
      if (!loadRecordFile()) {
        ROS_FATAL("Failed to load record file: %s", record_path_.c_str());
        ros::shutdown();
        return;
      }
      start_time_ = ros::Time::now();
    } else {
      ROS_WARN("Unknown mode '%s'. Use 'record' or 'replay'.", mode_.c_str());
    }

    timer_ = nh_.createTimer(ros::Duration(1.0 / loop_hz_), &TeachRepeatNode::loop, this);
  }

private:
  std::string defaultRecordPath() const {
    const char* home = std::getenv("HOME");
    if (home && home[0] != '\0') {
      return std::string(home) + "/.ros/maze_teach.csv";
    }
    return std::string("/tmp/maze_teach.csv");
  }

  void cmdCb(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_ = *msg;
    last_cmd_time_ = ros::Time::now();
    have_cmd_ = true;
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
    last_scan_ = msg;
    have_scan_ = true;
  }

  float sectorMin(const sensor_msgs::LaserScan& s, double deg_center, double deg_half) const {
    double a0 = (deg_center - deg_half) * M_PI / 180.0;
    double a1 = (deg_center + deg_half) * M_PI / 180.0;
    float mn = 100.f;
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      double a = s.angle_min + i * s.angle_increment;
      if (a < a0 || a > a1) continue;
      float r = s.ranges[i];
      if (std::isfinite(r) && r > s.range_min && r < s.range_max) mn = std::min(mn, r);
    }
    return mn;
  }

  bool frontBlocked() const {
    if (!safety_enable_ || !have_scan_) return false;
    float front = sectorMin(*last_scan_, 0.0, 18.0);
    return front < front_stop_m_;
  }

  void openRecordFile() {
    record_.open(record_path_.c_str(), std::ios::out | std::ios::trunc);
    if (!record_) {
      ROS_FATAL("Unable to open record file: %s", record_path_.c_str());
      ros::shutdown();
      return;
    }
    record_ << "# t,linear_x,angular_z\n";
    record_start_time_ = ros::Time::now();
  }

  bool loadRecordFile() {
    std::ifstream in(record_path_.c_str());
    if (!in) return false;

    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::stringstream ss(line);
      CmdSample s;
      char comma1 = 0, comma2 = 0;
      if (!(ss >> s.t >> comma1 >> s.v >> comma2 >> s.w)) continue;
      if (comma1 != ',' || comma2 != ',') continue;
      samples_.push_back(s);
    }

    if (samples_.empty()) return false;
    return true;
  }

  geometry_msgs::Twist safeCmd(const geometry_msgs::Twist& in) const {
    if (!frontBlocked()) return in;
    geometry_msgs::Twist stop;
    return stop;
  }

  void recordStep() {
    double t = (ros::Time::now() - record_start_time_).toSec();

    if (have_cmd_) {
      double dt = (ros::Time::now() - last_cmd_time_).toSec();
      if (dt < 0.3) {
        if (record_) {
          record_ << t << "," << last_cmd_.linear.x << "," << last_cmd_.angular.z << "\n";
          record_.flush();
        }
      } else {
        if (record_) {
          record_ << t << ",0,0\n";
          record_.flush();
        }
      }
    } else {
      if (record_) {
        record_ << t << ",0,0\n";
        record_.flush();
      }
    }
  }

  void replayStep() {
    if (samples_.empty()) return;

    if (frontBlocked()) {
      if (!paused_) {
        pause_start_ = ros::Time::now();
        paused_ = true;
      }
      cmd_pub_.publish(geometry_msgs::Twist());
      return;
    }

    if (paused_) {
      pause_offset_ += (ros::Time::now() - pause_start_).toSec();
      paused_ = false;
    }

    double elapsed = (ros::Time::now() - start_time_).toSec() - pause_offset_;
    while (idx_ + 1 < samples_.size() && samples_[idx_ + 1].t <= elapsed) {
      idx_++;
    }

    if (idx_ >= samples_.size()) {
      cmd_pub_.publish(geometry_msgs::Twist());
      return;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = samples_[idx_].v * replay_speed_scale_;
    cmd.angular.z = samples_[idx_].w * replay_speed_scale_;
    cmd_pub_.publish(cmd);
  }

  void loop(const ros::TimerEvent&) {
    if (mode_ == "record") {
      recordStep();
    } else if (mode_ == "replay") {
      replayStep();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_pub_;
  ros::Timer timer_;

  std::string mode_{"record"};
  double loop_hz_{20.0};
  std::string cmd_in_topic_{"/cmd_vel_in"};
  std::string cmd_out_topic_{"/cmd_vel"};
  std::string scan_topic_{"/scan"};
  double front_stop_m_{0.35};
  bool safety_enable_{true};
  double replay_speed_scale_{1.0};
  std::string record_path_;

  geometry_msgs::Twist last_cmd_;
  ros::Time last_cmd_time_;
  bool have_cmd_{false};

  sensor_msgs::LaserScan::ConstPtr last_scan_;
  bool have_scan_{false};

  std::ofstream record_;
  ros::Time record_start_time_;

  std::vector<CmdSample> samples_;
  size_t idx_{0};
  ros::Time start_time_;
  bool paused_{false};
  ros::Time pause_start_;
  double pause_offset_{0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "teach_repeat_node");
  TeachRepeatNode node;
  ros::spin();
  return 0;
}
