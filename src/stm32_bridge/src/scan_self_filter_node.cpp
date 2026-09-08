#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt32.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

// Removes lidar returns produced by brackets/chassis inside the robot body.
// GMapping does not know the robot footprint; feeding those returns directly to
// it permanently paints the robot into the saved static map. The same filtered
// scan is also used by AMCL and both costmaps.
class ScanSelfFilterNode {
public:
  ScanSelfFilterNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("input_topic", input_topic_, "/scan_raw");
    pnh_.param<std::string>("output_topic", output_topic_, "/scan");
    pnh_.param<double>("laser_x", laser_x_, 0.315);
    pnh_.param<double>("laser_y", laser_y_, 0.0);
    pnh_.param<double>("laser_yaw", laser_yaw_, 0.0);
    pnh_.param<double>("vehicle_front_x", vehicle_front_x_, 0.3483);
    pnh_.param<double>("vehicle_rear_x", vehicle_rear_x_, -0.1517);
    pnh_.param<double>("vehicle_half_width", vehicle_half_width_, 0.24);
    pnh_.param<double>("padding", padding_, 0.02);

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, 10);
    count_pub_ = nh_.advertise<std_msgs::UInt32>("/scan_self_filtered_count", 10);
    scan_sub_ = nh_.subscribe(input_topic_, 10, &ScanSelfFilterNode::scanCb, this);

    ROS_INFO("Scan self-filter: %s -> %s, body x=[%.3f, %.3f], |y|<=%.3f, padding=%.3f",
             input_topic_.c_str(), output_topic_.c_str(), vehicle_rear_x_,
             vehicle_front_x_, vehicle_half_width_, padding_);
  }

private:
  void scanCb(const sensor_msgs::LaserScan::ConstPtr& input) {
    sensor_msgs::LaserScan output = *input;
    uint32_t filtered = 0;

    for (size_t i = 0; i < output.ranges.size(); ++i) {
      const double range = output.ranges[i];
      if (!std::isfinite(range) || range <= 0.0) continue;

      const double angle = output.angle_min +
          static_cast<double>(i) * output.angle_increment + laser_yaw_;
      const double x_base = laser_x_ + range * std::cos(angle);
      const double y_base = laser_y_ + range * std::sin(angle);
      const bool inside_robot =
          x_base >= vehicle_rear_x_ - padding_ &&
          x_base <= vehicle_front_x_ + padding_ &&
          std::fabs(y_base) <= vehicle_half_width_ + padding_;

      if (!inside_robot) continue;
      output.ranges[i] = std::numeric_limits<float>::infinity();
      if (i < output.intensities.size()) output.intensities[i] = 0.0f;
      ++filtered;
    }

    scan_pub_.publish(output);
    std_msgs::UInt32 count;
    count.data = filtered;
    count_pub_.publish(count);
    ROS_INFO_THROTTLE(5.0, "Scan self-filter removed %u chassis returns", filtered);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber scan_sub_;
  ros::Publisher scan_pub_;
  ros::Publisher count_pub_;
  std::string input_topic_;
  std::string output_topic_;
  double laser_x_{0.315};
  double laser_y_{0.0};
  double laser_yaw_{0.0};
  double vehicle_front_x_{0.3483};
  double vehicle_rear_x_{-0.1517};
  double vehicle_half_width_{0.24};
  double padding_{0.02};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_self_filter");
  ScanSelfFilterNode node;
  ros::spin();
  return 0;
}
