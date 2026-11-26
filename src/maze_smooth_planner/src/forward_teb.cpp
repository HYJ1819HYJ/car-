#include <maze_smooth_planner/smoother.h>
#include <nav_core/base_local_planner.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <pluginlib/class_list_macros.h>
#include <boost/thread/locks.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <sstream>
#include <algorithm>
#include <cmath>

namespace maze_smooth_planner {
class ForwardTebLocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* map) override {
    if (map_) return;
    map_ = map;
    teb_.initialize("TebLocalPlannerROS", tf, map);
    ros::NodeHandle nh("~/" + name);
    nh.param("allow_controlled_reverse", allow_reverse_, false);
    nh.param("max_reverse_speed", max_reverse_speed_, .15);
    nh.param("max_continuous_reverse_dist", max_reverse_dist_, .40);
    nh.param("reverse_check_horizon", reverse_horizon_, .60);
    nh.param("reverse_collision_padding", reverse_padding_, .02);
    max_reverse_speed_ = std::max(0.0, std::min(.15, max_reverse_speed_));
    max_reverse_dist_ = std::max(0.0, std::min(.40, max_reverse_dist_));
    reverse_horizon_ = std::max(.60, std::min(2.0, reverse_horizon_));
    reverse_padding_ = std::max(.02, reverse_padding_);
    // An unobserved cell must stay unavailable for reverse, even in a
    // rolling map where the default is to initialize every cell as free.
    ros::NodeHandle local("~/" + map_->getName());
    bool tracks_unknown = false;
    local.param("track_unknown_space", tracks_unknown, false);
    if (allow_reverse_ && !tracks_unknown) {
      ROS_ERROR("Controlled reverse requires track_unknown_space in the local costmap");
      allow_reverse_ = false;
    }
    diagnostic_ = nh.advertise<std_msgs::String>("diagnostics", 1, true);
  }
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override {
    return teb_.setPlan(plan);
  }
  bool isGoalReached() override { return teb_.isGoalReached(); }
  bool computeVelocityCommands(geometry_msgs::Twist& command) override {
    command = geometry_msgs::Twist();
    if (!map_) return false;
    geometry_msgs::PoseStamped robot;
    const bool have_pose = map_->getRobotPose(robot);
    const Pose start{robot.pose.position.x, robot.pose.position.y, tf2::getYaw(robot.pose.orientation)};
    if (allow_reverse_ && have_pose && (!have_last_pose_ || robot.header.stamp > last_pose_stamp_)) {
      if (have_last_pose_) {
        const double ds = (start.x-last_pose_.x)*std::cos(last_pose_.yaw) +
                          (start.y-last_pose_.y)*std::sin(last_pose_.yaw);
        continuous_reverse_dist_ = std::max(0.0, continuous_reverse_dist_ - ds);
      }
      last_pose_ = start;
      last_pose_stamp_ = robot.header.stamp;
      have_last_pose_ = true;
    }
    geometry_msgs::Twist proposed;
    if (!teb_.computeVelocityCommands(proposed)) return false;
    if (!std::isfinite(proposed.linear.x) || !std::isfinite(proposed.angular.z)) return false;
    if (proposed.linear.x >= 0) { command = proposed; report("forward", proposed); return true; }
    if (!map_->isCurrent() || !have_pose) return false;
    const Pose finish{start.x, start.y, start.yaw + proposed.angular.z * 0.60};
    costmap_2d::Costmap2D snapshot;
    {
      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> guard(*map_->getCostmap()->getMutex());
      snapshot = *map_->getCostmap();
    }
    const auto body = map_->getUnpaddedRobotFootprint();
    const double remaining = max_reverse_dist_ - continuous_reverse_dist_;
    const double reverse_speed = std::min({-proposed.linear.x, max_reverse_speed_,
                                          remaining / reverse_horizon_});
    if (allow_reverse_ && reverse_speed > 1e-3) {
      const double v = -reverse_speed, w = proposed.angular.z * v / proposed.linear.x;
      const int steps = static_cast<int>(std::ceil(reverse_horizon_ / .05));
      std::vector<Pose> arc{start};
      for (int i = 1; i <= steps; ++i) {
        const double t = reverse_horizon_ * i / steps;
        const double yaw = start.yaw + w * t;
        const double dx = std::abs(w) > 1e-6 ? v/w*(std::sin(yaw)-std::sin(start.yaw)) :
                                              v*t*std::cos(start.yaw);
        const double dy = std::abs(w) > 1e-6 ? v/w*(std::cos(start.yaw)-std::cos(yaw)) :
                                              v*t*std::sin(start.yaw);
        arc.push_back({start.x+dx, start.y+dy, yaw});
      }
      if (pathFreeOrPaddingEscape(arc, snapshot, body, reverse_padding_)) {
        command.linear.x = v;
        command.angular.z = w;
        report(poseFree(start, snapshot, body, reverse_padding_) ?
            "controlled_reverse" : "controlled_reverse_escape", proposed);
        return true;
      }
    }
    // Never preserve the angular part of a rejected reverse arc unchecked.
    // This is a different maneuver, tested with the complete physical body.
    if (std::abs(proposed.angular.z) > 1e-3 &&
        pathFreeOrPaddingEscape({start, finish}, snapshot, body, 0.02)) {
      command.angular.z = proposed.angular.z;
      report(poseFree(start, snapshot, body, 0.02) ?
          "checked_turn_in_place" : "checked_turn_escape", proposed);
      return true;
    }
    report(allow_reverse_ && reverse_speed > 1e-3 ?
        "reverse_rejected_blocked" : "reverse_rejected_turn_blocked", proposed);
    return false;
  }
 private:
  void report(const char* mode, const geometry_msgs::Twist& input) {
    std::ostringstream json;
    json << "{\"mode\":\"" << mode << "\",\"teb_v\":" << input.linear.x
         << ",\"teb_w\":" << input.angular.z << "}";
    std_msgs::String msg; msg.data = json.str(); diagnostic_.publish(msg);
  }
  teb_local_planner::TebLocalPlannerROS teb_;
  costmap_2d::Costmap2DROS* map_ = nullptr;
  bool allow_reverse_{false}, have_last_pose_{false};
  double max_reverse_speed_{.15}, max_reverse_dist_{.40}, reverse_horizon_{.60}, reverse_padding_{.02};
  double continuous_reverse_dist_{0.0};
  Pose last_pose_{0, 0, 0};
  ros::Time last_pose_stamp_;
  ros::Publisher diagnostic_;
};
}
PLUGINLIB_EXPORT_CLASS(maze_smooth_planner::ForwardTebLocalPlanner, nav_core::BaseLocalPlanner)
