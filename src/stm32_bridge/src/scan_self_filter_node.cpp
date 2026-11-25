#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt32.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include "stm32_bridge/footprint_parameters.hpp"

// Removes lidar returns produced by the chassis and calibrated on-board parts.
// GMapping does not know the robot footprint; feeding those returns directly to
// it permanently paints the robot into the saved static map. The same filtered
// scan is also used by AMCL and both costmaps.
class ScanSelfFilterNode {
public:
  ScanSelfFilterNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("input_topic", input_topic_, "/scan_raw");
    pnh_.param<std::string>("output_topic", output_topic_, "/scan");
    pnh_.param<std::string>("obstacle_output_topic", obstacle_output_topic_, "/scan_obstacles");
    pnh_.param<double>("laser_x", laser_x_, 0.315);
    pnh_.param<double>("laser_y", laser_y_, 0.0);
    pnh_.param<double>("laser_yaw", laser_yaw_, 0.0);
    pnh_.param<double>("vehicle_front_x", vehicle_front_x_, 0.3483);
    pnh_.param<double>("vehicle_rear_x", vehicle_rear_x_, -0.1517);
    pnh_.param<double>("vehicle_half_width", vehicle_half_width_, 0.24);
    pnh_.param<double>("padding", padding_, 0.02);

    double fixed_echo_bearing_deg = 0.0;
    const bool have_echo_bearing =
        pnh_.getParam("fixed_self_echo_bearing_deg", fixed_echo_bearing_deg);
    const bool have_echo_range = pnh_.getParam("fixed_self_echo_range_m", fixed_echo_range_);
    if (have_echo_bearing && have_echo_range &&
        std::fabs(fixed_echo_bearing_deg) > 90.5 && std::fabs(fixed_echo_bearing_deg) <= 180.0 &&
        std::isfinite(fixed_echo_range_) && fixed_echo_range_ > 0.01) {
      fixed_echo_bearing_ = fixed_echo_bearing_deg * M_PI / 180.0;
      fixed_echo_enabled_ = true;
      ROS_INFO("Calibrated self echo at %.2f degrees, %.3f m",
               fixed_echo_bearing_deg, fixed_echo_range_);
    } else if (have_echo_bearing || have_echo_range) {
      ROS_WARN("Invalid fixed self echo calibration; using the chassis filter only");
    }

    footprint_.front=vehicle_front_x_;
    footprint_.rear=vehicle_rear_x_;
    footprint_.half_width=vehicle_half_width_;
    footprint_.padding=padding_;
    if (!stm32_bridge::readFootprintParam(pnh_,&footprint_)) {
      ros::shutdown();
      return;
    }
    vehicle_front_x_=footprint_.front;
    vehicle_rear_x_=footprint_.rear;
    vehicle_half_width_=footprint_.half_width;

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, 10);
    obstacle_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(obstacle_output_topic_, 10);
    count_pub_ = nh_.advertise<std_msgs::UInt32>("/scan_self_filtered_count", 10);
    scan_sub_ = nh_.subscribe(input_topic_, 10, &ScanSelfFilterNode::scanCb, this);

    ROS_INFO("Scan self-filter: %s -> %s, body x=[%.3f, %.3f], |y|<=%.3f, padding=%.3f",
             input_topic_.c_str(), output_topic_.c_str(), vehicle_rear_x_,
             vehicle_front_x_, vehicle_half_width_, padding_);
    ROS_INFO("Scan self-filter polygon vertices: %zu",footprint_.vertices.size());
    ROS_INFO("Obstacle observations: %s (self-occluded rays stay invalid)",
             obstacle_output_topic_.c_str());
  }

private:
  bool matchesFixedSelfEcho(double angle, double range) const {
    // Match a known part within half a degree and a centimeter in range.
    return fixed_echo_enabled_ &&
           std::fabs(std::remainder(angle - fixed_echo_bearing_, 2.0 * M_PI)) <=
               0.5 * M_PI / 180.0 &&
           std::fabs(range - fixed_echo_range_) <= 0.01;
  }

  void scanCb(const sensor_msgs::LaserScan::ConstPtr& input) {
    sensor_msgs::LaserScan output = *input;
    sensor_msgs::LaserScan obstacle_output = *input;
    uint32_t filtered = 0;

    for (size_t i = 0; i < output.ranges.size(); ++i) {
      const double range = output.ranges[i];
      if (!std::isfinite(range) || range <= 0.0) continue;

      const double angle = output.angle_min +
          static_cast<double>(i) * output.angle_increment + laser_yaw_;
      const double x_base = laser_x_ + range * std::cos(angle);
      const double y_base = laser_y_ + range * std::sin(angle);
      const bool inside_robot=stm32_bridge::pointInsideFootprint(
          stm32_bridge::SafetyPoint{x_base,y_base},
          stm32_bridge::SafetyPoint{0.0,0.0},0.0,footprint_);

      if (!inside_robot && !matchesFixedSelfEcho(angle, range)) continue;
      output.ranges[i] = std::numeric_limits<float>::infinity();
      // A chassis return means the ray was occluded, not that all space
      // behind it is free. Costmap inf_is_valid must not clear that shadow.
      // Keep the established SLAM/AMCL /scan stream unchanged.
      obstacle_output.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      if (i < output.intensities.size()) output.intensities[i] = 0.0f;
      if (i < obstacle_output.intensities.size()) obstacle_output.intensities[i] = 0.0f;
      ++filtered;
    }

    scan_pub_.publish(output);
    obstacle_scan_pub_.publish(obstacle_output);
    std_msgs::UInt32 count;
    count.data = filtered;
    count_pub_.publish(count);
    ROS_INFO_THROTTLE(5.0, "Scan self-filter removed %u self returns", filtered);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber scan_sub_;
  ros::Publisher scan_pub_;
  ros::Publisher obstacle_scan_pub_;
  ros::Publisher count_pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string obstacle_output_topic_;
  double laser_x_{0.315};
  double laser_y_{0.0};
  double laser_yaw_{0.0};
  double vehicle_front_x_{0.3483};
  double vehicle_rear_x_{-0.1517};
  double vehicle_half_width_{0.24};
  stm32_bridge::SafetyFootprint footprint_;
  double padding_{0.02};
  bool fixed_echo_enabled_{false};
  double fixed_echo_bearing_{0.0};
  double fixed_echo_range_{0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_self_filter");
  ScanSelfFilterNode node;
  ros::spin();
  return 0;
}
