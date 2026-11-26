#pragma once

#include <algorithm>
#include <vector>

#include <ros/time.h>
#include <maze_planner/wall_parking.h>

namespace maze_planner {

class FinishTargetVerification {
 public:
  enum class Result { Waiting, Seen, Missing, FalseTarget };
  static constexpr double kAssociationRadius = .55;

  void reset() {
    last_seen_ = arrived_ = ros::Time();
    misses_ = 0;
  }

  void seen(const ros::Time& stamp) { last_seen_ = std::max(last_seen_, stamp); }
  bool seenAfter(const ros::Time& stamp) const { return last_seen_ > stamp; }

  void arrive(const ros::Time& now) { arrived_ = now; }
  void leaveEntrance() { arrived_ = ros::Time(); }
  bool arrived() const { return !arrived_.isZero(); }
  bool needsAlternateView() const { return misses_ != 0; }

  Result result(const ros::Time& now, double scan_seconds) {
    if (!arrived() || (now - arrived_).toSec() < scan_seconds) return Result::Waiting;
    const bool seen_during_scan = last_seen_ >= arrived_;
    if (!seen_during_scan && (now - last_seen_).toSec() < 4.0) return Result::Waiting;
    leaveEntrance();
    if (seen_during_scan) {
      misses_ = 0;
      return Result::Seen;
    }
    return ++misses_ >= 2 ? Result::FalseTarget : Result::Missing;
  }

 private:
  ros::Time last_seen_, arrived_;
  int misses_{0};
};

class FalseFinishTargets {
 public:
  void clear() { entries_.clear(); }
  void add(const Point2& target, const ros::Time& now) {
    entries_.push_back({target, now + ros::Duration(45.0)});
  }

  bool blocks(const Point2& target, double contour_ratio, const ros::Time& now) {
    entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
        [&](const Entry& e) { return e.expires <= now; }), entries_.end());
    return contour_ratio + 1e-7 < .03 &&
        std::any_of(entries_.begin(), entries_.end(), [&](const Entry& e) {
          return norm(target - e.point) <= .90;
        });
  }

 private:
  struct Entry {
    Point2 point;
    ros::Time expires;
  };
  std::vector<Entry> entries_;
};

}  // namespace maze_planner
