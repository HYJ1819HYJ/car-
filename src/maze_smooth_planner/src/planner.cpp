#include <maze_smooth_planner/smoother.h>
#include <nav_core/base_global_planner.h>
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <boost/thread/locks.hpp>
#include <mutex>
#include <sstream>

namespace maze_smooth_planner {
class SmoothGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  void initialize(std::string name,costmap_2d::Costmap2DROS* map) override {
    if(map_)return;
    map_=map;ros::NodeHandle nh("~/"+name),priv("~");
    std::string source;nh.param<std::string>("source_planner_ns",source,"GlobalPlanner");
    XmlRpc::XmlRpcValue params;
    if(priv.getParam(source,params))priv.setParam(name+"/raw",params);
    raw_.initialize(name+"/raw",map);
    nh.param("sample_spacing",cfg_.sample_spacing,cfg_.sample_spacing);
    nh.param("shortcut_deviation",cfg_.shortcut_deviation,cfg_.shortcut_deviation);
    nh.param("shortcut_length",cfg_.shortcut_length,cfg_.shortcut_length);
    nh.param("preferred_radius",cfg_.preferred_radius,cfg_.preferred_radius);
    nh.param("smoothing_clearance",cfg_.smoothing_clearance,cfg_.smoothing_clearance);
    nh.param("collision_padding",cfg_.collision_padding,cfg_.collision_padding);
    nh.param("max_compute_ms",cfg_.max_compute_ms,cfg_.max_compute_ms);
    nh.param("max_lateral_adjustment",cfg_.max_lateral_adjustment,cfg_.max_lateral_adjustment);
    nh.param("goal_tangent_length",cfg_.goal_tangent_length,cfg_.goal_tangent_length);
    if(cfg_.sample_spacing<=0 || cfg_.preferred_radius<=0 || cfg_.max_compute_ms<=0 ||
       cfg_.collision_padding<0 || cfg_.smoothing_clearance<cfg_.collision_padding)
      throw std::runtime_error("Invalid SmoothGlobalPlanner geometry settings");
    plan_pub_=nh.advertise<nav_msgs::Path>("plan",1,true);
    compatible_pub_=priv.advertise<nav_msgs::Path>("GlobalPlanner/plan",1,true);
    raw_pub_=nh.advertise<nav_msgs::Path>("raw_plan",1,true);
    diag_pub_=nh.advertise<std_msgs::String>("diagnostics",1,true);
  }
  bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override {
    std::lock_guard<std::mutex> lock(mutex_);plan.clear();if(!map_)return false;
    std::vector<geometry_msgs::PoseStamped> raw;
    if(!raw_.makePlan(start,goal,raw) || raw.size()<2)return false;
    nav_msgs::Path message;message.header=raw.front().header;message.header.stamp=ros::Time::now();
    message.poses=raw;raw_pub_.publish(message);
    // Actual robot pose anchors the output; grid quantization must not move it.
    raw.front()=start;
    costmap_2d::Costmap2D snapshot;
    {boost::unique_lock<costmap_2d::Costmap2D::mutex_t> guard(*map_->getCostmap()->getMutex());snapshot=*map_->getCostmap();}
    auto body=map_->getUnpaddedRobotFootprint();
    // SAT collision checker supports convex polygons. Reject unsupported outlines.
    double direction=0;
    for(size_t i=0;i<body.size();++i){const auto& a=body[i];const auto& b=body[(i+1)%body.size()];const auto& c=body[(i+2)%body.size()];
      double cross=(b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x);
      if(std::abs(cross)<1e-10)continue;
      if(direction*cross<0){ROS_ERROR_THROTTLE(2,"Smooth planner requires a convex footprint");return false;}direction=cross;
    }
    if(body.size()<3 || std::abs(direction)<1e-10)return false;
    std::vector<Pose> input,output;
    for(const auto& p:raw){const auto& q=p.pose.orientation;
      double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      input.push_back({p.pose.position.x,p.pose.position.y,yaw});}
    Statistics stats;bool valid=smooth(input,snapshot,body,cfg_,output,stats);
    std::ostringstream json;json<<"{\"valid\":"<<(valid?"true":"false")<<",\"raw_poses\":"<<raw.size()
      <<",\"smooth_poses\":"<<output.size()<<",\"rounded_corners\":"<<stats.rounded_corners
      <<",\"repair_steps\":"<<stats.repair_steps<<",\"fallback\":"<<(stats.fallback?"true":"false")<<",\"smoothing_ms\":"<<stats.elapsed_ms
      <<",\"start_free\":"<<(poseFree(input.front(),snapshot,body,cfg_.collision_padding)?"true":"false")
      <<",\"goal_free\":"<<(poseFree(input.back(),snapshot,body,cfg_.collision_padding)?"true":"false")
      <<",\"failed_pose\":["<<stats.failed_pose.x<<","<<stats.failed_pose.y<<","<<stats.failed_pose.yaw<<"]}";
    std_msgs::String diagnostic;diagnostic.data=json.str();diag_pub_.publish(diagnostic);
    if(!valid){ROS_WARN_THROTTLE(2,"No footprint-checked global reference; request replanning");return false;}
    for(const auto& p:output){geometry_msgs::PoseStamped pose;pose.header=message.header;
      pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.orientation.z=std::sin(p.yaw/2);pose.pose.orientation.w=std::cos(p.yaw/2);plan.push_back(pose);}
    plan.front().pose.orientation=start.pose.orientation;
    plan.back().pose.orientation=goal.pose.orientation;
    message.poses=plan;plan_pub_.publish(message);compatible_pub_.publish(message);return true;
  }
 private:
  costmap_2d::Costmap2DROS* map_=nullptr;
  global_planner::GlobalPlanner raw_;
  Config cfg_;std::mutex mutex_;
  ros::Publisher plan_pub_,compatible_pub_,raw_pub_,diag_pub_;
};
}
PLUGINLIB_EXPORT_CLASS(maze_smooth_planner::SmoothGlobalPlanner,nav_core::BaseGlobalPlanner)
