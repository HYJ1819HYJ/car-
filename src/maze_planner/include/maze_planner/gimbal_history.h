#pragma once

#include <ros/time.h>
#include <sensor_msgs/JointState.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>

namespace maze_planner {

class GimbalHistory {
 public:
  void add(const sensor_msgs::JointState& joint) {
    Sample sample{joint.header.stamp, 0.0, 0.0};
    if (sample.time.isZero() || (!samples_.empty() && sample.time <= samples_.back().time))
      return;
    bool pan = false;
    for (std::size_t i = 0; i < joint.name.size() && i < joint.position.size(); ++i) {
      if (joint.name[i] == "camera_pan_joint") {
        sample.pan = joint.position[i];
        pan = true;
      }
      if (joint.name[i] == "camera_tilt_joint") sample.tilt = joint.position[i];
    }
    if (!pan || !std::isfinite(sample.pan) || !std::isfinite(sample.tilt)) return;
    samples_.push_back(sample);
    while (samples_.size() > 1 && (sample.time - samples_.front().time).toSec() > 2.0)
      samples_.pop_front();
  }

  bool angles(const ros::Time& stamp, double* pan, double* tilt,
              double max_skew = .15) const {
    if (stamp.isZero() || samples_.empty()) return false;
    const auto upper = std::lower_bound(samples_.begin(), samples_.end(), stamp,
        [](const Sample& sample, const ros::Time& time) { return sample.time < time; });
    if (upper == samples_.begin() || upper == samples_.end()) {
      const auto& sample = upper == samples_.end() ? samples_.back() : *upper;
      if (std::abs((sample.time - stamp).toSec()) > max_skew) return false;
      *pan = sample.pan;
      *tilt = sample.tilt;
      return true;
    }
    const auto& a = *std::prev(upper);
    const auto& b = *upper;
    const double left = (stamp - a.time).toSec(), right = (b.time - stamp).toSec();
    if (std::min(left, right) > max_skew) return false;
    if (std::max(left, right) > max_skew) {
      const auto& nearest = left < right ? a : b;
      *pan = nearest.pan;
      *tilt = nearest.tilt;
      return true;
    }
    const double weight = (stamp - a.time).toSec() / (b.time - a.time).toSec();
    *pan = a.pan + weight * (b.pan - a.pan);
    *tilt = a.tilt + weight * (b.tilt - a.tilt);
    return true;
  }

 private:
  struct Sample {
    ros::Time time;
    double pan, tilt;
  };
  std::deque<Sample> samples_;
};

}  // namespace maze_planner
