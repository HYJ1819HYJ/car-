#include <maze_smooth_planner/smoother.h>
#include <nav_core/base_local_planner.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <pluginlib/class_list_macros.h>
#include <boost/thread/locks.hpp>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <sstream>

namespace maze_smooth_planner {
// TEB's packaged dynamic configuration cannot express zero reverse speed.
// A reverse request therefore needs a separately validated forward-only
// maneuver before it is passed to the unchanged chassis safety layer.
class ForwardTebLocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* map) override {
    if (map_) return;
    map_ = map;
    teb_.initialize("TebLocalPlannerROS", tf, map);
    ros::NodeHandle nh("~/" + name);
    diagnostic_ = nh.advertise<std_msgs::String>("diagnostics", 1, true);
  }
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override {
    return teb_.setPlan(plan);
  }
  bool isGoalReached() override { return teb_.isGoalReached(); }
  bool computeVelocityCommands(geometry_msgs::Twist& command) override {
    command = geometry_msgs::Twist();
    geometry_msgs::Twist proposed;
    if (!map_ || !teb_.computeVelocityCommands(proposed)) return false;
    if (!std::isfinite(proposed.linear.x) || !std::isfinite(proposed.angular.z)) return false;
    if (proposed.linear.x >= 0) { command = proposed; report("forward", proposed); return true; }
    geometry_msgs::PoseStamped robot;
    if (!map_->isCurrent() || !map_->getRobotPose(robot)) return false;
    const Pose start{robot.pose.position.x, robot.pose.position.y, tf2::getYaw(robot.pose.orientation)};
    const Pose finish{start.x, start.y, start.yaw + proposed.angular.z * 0.60};
    costmap_2d::Costmap2D snapshot;
    {
      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> guard(*map_->getCostmap()->getMutex());
      snapshot = *map_->getCostmap();
    }
    // Never preserve the angular part of a rejected reverse arc unchecked.
    // This is a different maneuver, tested with the complete physical body.
    if (std::abs(proposed.angular.z) > 1e-3 &&
        pathFree({start, finish}, snapshot, map_->getUnpaddedRobotFootprint(), 0.02)) {
      command.angular.z = proposed.angular.z;
      report("checked_turn_in_place", proposed);
      return true;
    }
    report("reverse_rejected_turn_blocked", proposed);
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
  ros::Publisher diagnostic_;
};
}
PLUGINLIB_EXPORT_CLASS(maze_smooth_planner::ForwardTebLocalPlanner, nav_core::BaseLocalPlanner)
