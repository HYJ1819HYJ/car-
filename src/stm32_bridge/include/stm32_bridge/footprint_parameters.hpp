#pragma once

#include <ros/ros.h>
#include <XmlRpcValue.h>
#include "stm32_bridge/safety_geometry.hpp"

namespace stm32_bridge {

// Optional private parameter. A malformed polygon must fail before hardware is
// opened; it must never silently fall back to a smaller or different outline.
inline bool readFootprintParam(ros::NodeHandle& nh, SafetyFootprint* footprint) {
  if (!nh.hasParam("footprint")) return true;
  XmlRpc::XmlRpcValue raw;
  if (!nh.getParam("footprint",raw) ||
      raw.getType()!=XmlRpc::XmlRpcValue::TypeArray) {
    ROS_FATAL("~footprint must be an array of [x,y] vertices in metres");
    return false;
  }
  std::vector<SafetyPoint> vertices;
  for (int i=0; i<raw.size(); ++i) {
    auto& pair=raw[i];
    if (pair.getType()!=XmlRpc::XmlRpcValue::TypeArray || pair.size()!=2) {
      ROS_FATAL("~footprint vertex %d must contain exactly x and y",i);
      return false;
    }
    double xy[2];
    for (int j=0;j<2;++j) {
      if (pair[j].getType()==XmlRpc::XmlRpcValue::TypeDouble)
        xy[j]=static_cast<double>(pair[j]);
      else if (pair[j].getType()==XmlRpc::XmlRpcValue::TypeInt)
        xy[j]=static_cast<int>(pair[j]);
      else {ROS_FATAL("~footprint coordinates must be numeric");return false;}
    }
    vertices.push_back(SafetyPoint{xy[0],xy[1]});
  }
  if (!normalizeConvexFootprint(&vertices)) {
    ROS_FATAL("~footprint must be a finite, nondegenerate convex polygon with "
              "3..64 vertices containing the base origin");
    return false;
  }
  footprint->vertices=vertices;
  footprint->front=vertices.front().x;
  footprint->rear=vertices.front().x;
  footprint->half_width=0.0;
  for (const auto& p:vertices) {
    footprint->front=std::max(footprint->front,p.x);
    footprint->rear=std::min(footprint->rear,p.x);
    footprint->half_width=std::max(footprint->half_width,std::fabs(p.y));
  }
  footprint->polygon_bounds_valid=true;
  return true;
}

}  // namespace stm32_bridge
