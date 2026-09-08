#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <algorithm>

class MazeController {
public:
  MazeController(): nh_("~"){
    nh_.param<double>("loop_hz", loop_hz_, 20.0);
    nh_.param<double>("front_stop_m", front_stop_m_, 0.35);
    nh_.param<double>("side_open_m", side_open_m_, 0.45);
    nh_.param<double>("v_forward", v_forward_, 0.18);
    nh_.param<double>("w_turn", w_turn_, 0.9);
    nh_.param<bool>("enable_camera_goal", enable_camera_goal_, true);
    nh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    nh_.param<std::string>("goal_topic", goal_topic_, "/goal_detected");
    nh_.param<std::string>("cmd_topic", cmd_topic_, "/cmd_vel");
    nh_.param<std::string>("mode", mode_, "explore");
    nh_.param<std::string>("path_file", path_file_, defaultPathFile());
    nh_.param<bool>("use_shortest_path", use_shortest_path_, true);
    nh_.param<int>("turn_ticks_need", turn_ticks_need_, 12);

    scan_sub_ = nh_.subscribe(scan_topic_, 10, &MazeController::scanCb, this);
    if (enable_camera_goal_) {
      goal_sub_ = nh_.subscribe(goal_topic_, 5, &MazeController::goalCb, this);
    }
    cmd_pub_  = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 10);

    if (mode_ == "replay") {
      if (!loadPath()) {
        ROS_FATAL("Failed to load path file: %s", path_file_.c_str());
        ros::shutdown();
      }
    }

    timer_ = nh_.createTimer(ros::Duration(1.0/loop_hz_), &MazeController::loop, this);
  }

private:
  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg){
    last_scan_ = msg;
    have_scan_ = true;
  }

  void goalCb(const std_msgs::Bool::ConstPtr& msg){
    // 终点一旦经 blue_detector 多帧确认就锁定，避免因遮挡重新起步。
    if (msg->data) goal_seen_ = true;
  }

  float sectorMin(const sensor_msgs::LaserScan& s, double deg_center, double deg_half){
    double a0 = (deg_center-deg_half) * M_PI/180.0;
    double a1 = (deg_center+deg_half) * M_PI/180.0;
    float mn = 100.f;
    for(size_t i=0;i<s.ranges.size();++i){
      double a = s.angle_min + i*s.angle_increment;
      if(a<a0 || a>a1) continue;
      float r = s.ranges[i];
      if(std::isfinite(r) && r > s.range_min && r < s.range_max) mn = std::min(mn, r);
    }
    return mn;
  }

  void loop(const ros::TimerEvent&){
    geometry_msgs::Twist cmd;
    if(!have_scan_){
      cmd_pub_.publish(cmd); // stop
      return;
    }

    const auto& s = *last_scan_;
    float front = sectorMin(s, 0, 18);
    float left  = sectorMin(s, 90, 25);
    float right = sectorMin(s, -90, 25);

    if (goal_seen_ && mode_ == "explore") {
      cmd_pub_.publish(geometry_msgs::Twist());
      savePath();
      return;
    }
    if (goal_seen_ && mode_ == "replay") {
      cmd_pub_.publish(geometry_msgs::Twist());
      return;
    }

    if (mode_ == "replay") {
      cmd = replayStep(front, left, right);
      cmd_pub_.publish(cmd);
      return;
    }

    cmd = exploreStep(front, left, right);
    cmd_pub_.publish(cmd);
  }

  geometry_msgs::Twist exploreStep(float front, float left, float right) {
    geometry_msgs::Twist cmd;
    bool can_front = (front >= front_stop_m_);
    bool can_left = (left >= side_open_m_);
    bool can_right = (right >= side_open_m_);

    int options = (can_left ? 1 : 0) + (can_front ? 1 : 0) + (can_right ? 1 : 0);
    bool at_junction = options >= 2;

    if (!turning_) {
      if (at_junction) {
        char decision = decideTurn(can_left, can_front, can_right);
        pending_turn_ = decision;
        turning_ = true;
      }
    }

    if (turning_) {
      cmd.linear.x = 0.0;
      if (pending_turn_ == 'L') cmd.angular.z = +w_turn_;
      else if (pending_turn_ == 'R') cmd.angular.z = -w_turn_;
      else if (pending_turn_ == 'B') cmd.angular.z = +w_turn_;
      else cmd.angular.z = 0.0;

      turn_ticks_++;
      if (turn_ticks_ >= turn_ticks_need_) {
        turning_ = false;
        turn_ticks_ = 0;
        recordDecision(pending_turn_);
        pending_turn_ = 'S';
      }
      return cmd;
    }

    if (!can_front) {
      cmd.linear.x = 0.0;
      cmd.angular.z = +w_turn_;
    } else {
      cmd.linear.x = v_forward_;
      if (left > side_open_m_ + 0.15) cmd.angular.z = +0.25;
      else if (left < side_open_m_ - 0.15) cmd.angular.z = -0.20;
      else cmd.angular.z = 0.0;
    }
    return cmd;
  }

  geometry_msgs::Twist replayStep(float front, float left, float right) {
    geometry_msgs::Twist cmd;
    bool can_front = (front >= front_stop_m_);
    bool can_left = (left >= side_open_m_);
    bool can_right = (right >= side_open_m_);

    int options = (can_left ? 1 : 0) + (can_front ? 1 : 0) + (can_right ? 1 : 0);
    bool at_junction = options >= 2;

    if (!turning_ && at_junction && replay_idx_ < path_.size()) {
      pending_turn_ = path_[replay_idx_++];
      turning_ = true;
    }

    if (turning_) {
      cmd.linear.x = 0.0;
      if (pending_turn_ == 'L') cmd.angular.z = +w_turn_;
      else if (pending_turn_ == 'R') cmd.angular.z = -w_turn_;
      else if (pending_turn_ == 'B') cmd.angular.z = +w_turn_;
      else cmd.angular.z = 0.0;

      turn_ticks_++;
      if (turn_ticks_ >= turn_ticks_need_) {
        turning_ = false;
        turn_ticks_ = 0;
        pending_turn_ = 'S';
      }
      return cmd;
    }

    if (!can_front) {
      cmd.linear.x = 0.0;
      cmd.angular.z = +w_turn_;
    } else {
      cmd.linear.x = v_forward_;
      if (left > side_open_m_ + 0.15) cmd.angular.z = +0.25;
      else if (left < side_open_m_ - 0.15) cmd.angular.z = -0.20;
      else cmd.angular.z = 0.0;
    }
    return cmd;
  }

  char decideTurn(bool can_left, bool can_front, bool can_right) const {
    if (can_left) return 'L';
    if (can_front) return 'S';
    if (can_right) return 'R';
    return 'B';
  }

  void recordDecision(char d) {
    path_.push_back(d);
    if (!use_shortest_path_) return;
    if (path_.size() < 3) return;
    size_t n = path_.size();
    char a = path_[n - 3];
    char b = path_[n - 2];
    char c = path_[n - 1];
    if (b != 'B') return;
    char reduced = 0;
    if (a == 'L' && c == 'R') reduced = 'S';
    else if (a == 'R' && c == 'L') reduced = 'S';
    else if (a == 'L' && c == 'L') reduced = 'B';
    else if (a == 'R' && c == 'R') reduced = 'B';
    if (reduced != 0) {
      path_.resize(n - 3);
      path_.push_back(reduced);
    }
  }

  void savePath() {
    if (path_saved_) return;
    std::ofstream out(path_file_.c_str(), std::ios::out | std::ios::trunc);
    if (!out) {
      ROS_WARN("Failed to write path file: %s", path_file_.c_str());
      return;
    }
    for (char c : path_) out << c;
    out << "\n";
    out.close();
    path_saved_ = true;
    ROS_INFO("Path saved: %s", path_file_.c_str());
  }

  bool loadPath() {
    std::ifstream in(path_file_.c_str());
    if (!in) return false;
    std::string line;
    if (!std::getline(in, line)) return false;
    path_.clear();
    for (char c : line) {
      if (c == 'L' || c == 'R' || c == 'S' || c == 'B') path_.push_back(c);
    }
    return !path_.empty();
  }

  std::string defaultPathFile() const {
    const char* home = std::getenv("HOME");
    if (home && home[0] != '\0') return std::string(home) + "/.ros/maze_path.txt";
    return std::string("/tmp/maze_path.txt");
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_, goal_sub_;
  ros::Publisher cmd_pub_;
  ros::Timer timer_;

  sensor_msgs::LaserScan::ConstPtr last_scan_;
  bool have_scan_{false};
  bool goal_seen_{false};

  double loop_hz_{20.0}, front_stop_m_{0.35}, side_open_m_{0.45};
  double v_forward_{0.18}, w_turn_{0.9};
  bool enable_camera_goal_{true};
  std::string scan_topic_{"/scan"};
  std::string goal_topic_{"/goal_detected"};
  std::string cmd_topic_{"/cmd_vel"};

  std::string mode_{"explore"};
  std::string path_file_;
  bool use_shortest_path_{true};
  std::vector<char> path_;
  size_t replay_idx_{0};
  bool turning_{false};
  int turn_ticks_{0};
  int turn_ticks_need_{12};
  char pending_turn_{'S'};
  bool path_saved_{false};
};

int main(int argc, char** argv){
  ros::init(argc, argv, "maze_controller_node");
  MazeController n;
  ros::spin();
  return 0;
}
