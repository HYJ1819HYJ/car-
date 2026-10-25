// Independent, scan-based final gate for the virtual chassis. Reuses the
// physical bridge's full footprint sweep, without opening a serial device.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "stm32_bridge/footprint_parameters.hpp"
#include <limits>
#include <stdexcept>

class SafetyGate {
 public:
  SafetyGate() : private_("~") {
    ros::NodeHandle geometry("/robot_geometry");
    if (!geometry.hasParam("footprint") ||
        !stm32_bridge::readFootprintParam(geometry, &body_))
      throw std::runtime_error("Simulation safety gate requires the real robot footprint");
    body_.padding = 0.0;
    sweep_ = body_;
    sweep_.padding = 0.02;
    private_.param("prediction_time", horizon_, 0.8);
    private_.param("max_vel_x", max_v_, 0.5);
    private_.param("max_vel_theta", max_w_, 0.75);
    if (!std::isfinite(max_v_) || max_v_ <= 0 ||
        !std::isfinite(max_w_) || max_w_ <= 0 ||
        !std::isfinite(horizon_) || horizon_ <= 0)
      throw std::runtime_error("Simulation velocity limits and prediction time must be finite and positive");
    double body_radius = 0;
    for (const auto& vertex : body_.vertices)
      body_radius = std::max(body_radius, std::hypot(vertex.x, vertex.y));
    // Retain every return that can intersect the predicted swept body.
    // At 3 m/s the old fixed 2 m filter truncated the 0.8 s prediction.
    observation_radius_ = std::max(2.0, max_v_ * horizon_ + body_radius + sweep_.padding + 0.03);
    std::string input, output, scan;
    private_.param<std::string>("input_topic", input, "/simulation/nav_cmd_vel");
    private_.param<std::string>("output_topic", output, "/simulation/cmd_vel");
    private_.param<std::string>("scan_topic", scan, "/scan_obstacles");
    command_ = node_.subscribe(input, 1, &SafetyGate::command, this);
    scan_ = node_.subscribe(scan, 1, &SafetyGate::scan, this);
    output_ = node_.advertise<geometry_msgs::Twist>(output, 1);
    stop_ = node_.advertise<std_msgs::Bool>("/simulation/safety_stop", 1, true);
    reason_ = node_.advertise<std_msgs::String>("/simulation/safety_reason", 1, true);
    timer_ = node_.createTimer(ros::Duration(0.02), &SafetyGate::tick, this);
  }

 private:
  void command(const geometry_msgs::Twist::ConstPtr& message) {
    requested_ = *message;
    command_stamp_ = ros::Time::now();
  }
  void scan(const sensor_msgs::LaserScan::ConstPtr& message) {
    if (message->header.frame_id != "base_laser_link" && message->header.frame_id != "laser") {
      scan_stamp_ = ros::Time();
      return;
    }
    scan_stamp_ = message->header.stamp;
    points_.clear();
    front_ = std::numeric_limits<double>::infinity();
    bool have_observation = false;
    for (size_t i=0; i<message->ranges.size(); ++i) {
      const double r = message->ranges[i];
      // +inf is a genuine clear ray; NaN is an invalid/occluded ray.
      if (std::isinf(r) && r>0) have_observation = true;
      if (!std::isfinite(r) || r < message->range_min || r >= message->range_max) continue;
      have_observation = true;
      const double angle = message->angle_min+i*message->angle_increment;
      const stm32_bridge::SafetyPoint point{0.315+r*std::cos(angle), r*std::sin(angle)};
      if (stm32_bridge::pointInsideFootprint(point, {0,0}, 0, body_)) continue;
      if (std::hypot(point.x, point.y)<observation_radius_) points_.push_back(point);
      if (std::fabs(point.y) <= body_.half_width+sweep_.padding) {
        double rear, front;
        stm32_bridge::longitudinalBodyLimits(body_,point.y,&rear,&front);
        if (point.x>=front) front_=std::min(front_,point.x-front);
      }
    }
    if (!have_observation) scan_stamp_=ros::Time();
  }
  void tick(const ros::TimerEvent&) {
    const auto now=ros::Time::now();
    geometry_msgs::Twist safe=requested_;
    std::string reason="clear";
    bool stopped=false;
    auto reject=[&](const std::string& why) {safe=geometry_msgs::Twist();reason=why;stopped=true;};
    const double scan_age=(now-scan_stamp_).toSec();
    const double command_age=(now-command_stamp_).toSec();
    if (scan_stamp_.isZero() || scan_age<0 || scan_age>0.30) reject("lidar_timeout_stop");
    else if (command_stamp_.isZero() || command_age<0 || command_age>0.50) reject("command_timeout_stop");
    else if (!std::isfinite(safe.linear.x) || !std::isfinite(safe.angular.z)) reject("invalid_command_stop");
    else if (safe.linear.x < -1e-6) reject("navigation_reverse_stop");
    else {
      safe.linear.x=std::max(0.0,safe.linear.x);
      safe.linear.y=safe.linear.z=safe.angular.x=safe.angular.y=0;
      // Apply a common scale if a command exceeds either configured limit,
      // preserving the planned curvature before the collision check.
      double limit_scale = 1.0;
      if (safe.linear.x > max_v_) limit_scale = std::min(limit_scale, max_v_ / safe.linear.x);
      if (std::fabs(safe.angular.z) > max_w_)
        limit_scale = std::min(limit_scale, max_w_ / std::fabs(safe.angular.z));
      safe.linear.x *= limit_scale;
      safe.angular.z *= limit_scale;
      if (safe.linear.x>1e-3 && front_<=0.08) reject("front_clearance_stop");
      else {
        if (safe.linear.x>1e-3 && front_<0.30) {
          const double scale=std::max(0.0,std::min(1.0,(front_-0.08)/(0.30-0.08)));
          safe.linear.x*=scale;
          safe.angular.z*=scale;  // Preserve the planned arc when slowing.
          reason="front_slow";
        }
        if ((std::fabs(safe.linear.x)+std::fabs(safe.angular.z)>1e-6) &&
            stm32_bridge::trajectoryIntersects(points_,sweep_,safe.linear.x,safe.angular.z,horizon_,0.03))
          reject("footprint_prediction_stop");
      }
    }
    output_.publish(safe);
    std_msgs::Bool stop; stop.data=stopped; stop_.publish(stop);
    std_msgs::String state; state.data=reason; reason_.publish(state);
  }
  ros::NodeHandle node_,private_;
  ros::Subscriber command_,scan_;
  ros::Publisher output_,stop_,reason_;
  ros::Timer timer_;
  geometry_msgs::Twist requested_;
  ros::Time command_stamp_,scan_stamp_;
  stm32_bridge::SafetyFootprint body_,sweep_;
  std::vector<stm32_bridge::SafetyPoint> points_;
  double front_=std::numeric_limits<double>::infinity(),horizon_,max_v_,max_w_,observation_radius_;
};

int main(int argc,char** argv) {
  ros::init(argc,argv,"simulation_safety_gate");
  try {SafetyGate gate;ros::spin();}
  catch(const std::exception& error) {ROS_FATAL("%s",error.what());return 1;}
  return 0;
}
