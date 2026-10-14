#include <maze_smooth_planner/smoother.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/cost_values.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace maze_smooth_planner {
namespace {
double distance(const Pose& a, const Pose& b) { return std::hypot(a.x-b.x,a.y-b.y); }
double wrap(double a) { return std::atan2(std::sin(a),std::cos(a)); }
Pose blend(const Pose& a,const Pose& b,double t) {
  return {a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.yaw+t*wrap(b.yaw-a.yaw)};
}
bool intersects(const std::vector<geometry_msgs::Point>& polygon,double cx,double cy,double h) {
  // Separating-axis test for the convex chassis polygon and a whole cell.
  for (size_t i=0;i<polygon.size()+2;++i) {
    double ax=1,ay=0;
    if (i==polygon.size()+1) {ax=0;ay=1;}
    else if(i<polygon.size()) {
      const auto& a=polygon[i]; const auto& b=polygon[(i+1)%polygon.size()];
      ax=-(b.y-a.y);ay=b.x-a.x;
    }
    double lo=std::numeric_limits<double>::infinity(),hi=-lo;
    for(const auto& p:polygon) {double z=ax*p.x+ay*p.y;lo=std::min(lo,z);hi=std::max(hi,z);}
    double mid=ax*cx+ay*cy,r=h*(std::abs(ax)+std::abs(ay));
    if(hi<mid-r-1e-10 || lo>mid+r+1e-10) return false;
  }
  return true;
}
void sampleLine(Pose a,Pose b,double step,std::vector<Pose>& out) {
  int n=std::max(1,static_cast<int>(std::ceil(distance(a,b)/step)));
  for(int i=1;i<=n;++i)out.push_back(blend(a,b,double(i)/n));
}
std::vector<Pose> tangentPath(std::vector<Pose> p,const Pose& start,const Pose& goal) {
  for(size_t i=0;i+1<p.size();++i) p[i].yaw=std::atan2(p[i+1].y-p[i].y,p[i+1].x-p[i].x);
  if(p.size()>1)p.back().yaw=p[p.size()-2].yaw;
  if(!p.empty()){p.front().yaw=start.yaw;p.back().yaw=goal.yaw;}
  return p;
}
double lineDeviation(const Pose& p,const Pose& a,const Pose& b) {
  double dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
  double t=den>1e-12?std::max(0.,std::min(1.,((p.x-a.x)*dx+(p.y-a.y)*dy)/den)):0.;
  return std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
}
}
static int collisionCells(const Pose& pose,const costmap_2d::Costmap2D& map,
              const std::vector<geometry_msgs::Point>& body,double padding) {
  if(body.size()<3 || !std::isfinite(pose.x+pose.y+pose.yaw)) return 10000;
  auto poly=body;costmap_2d::padFootprint(poly,padding);
  double xmin=1e30,ymin=1e30,xmax=-1e30,ymax=-1e30,c=std::cos(pose.yaw),s=std::sin(pose.yaw);
  for(auto& p:poly){double x=p.x,y=p.y;p.x=pose.x+c*x-s*y;p.y=pose.y+s*x+c*y;
    xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);}
  unsigned int x0,y0,x1,y1;
  if(!map.worldToMap(xmin,ymin,x0,y0)||!map.worldToMap(xmax,ymax,x1,y1))return 10000;
  int blocked=0;
  for(unsigned int y=y0;y<=y1;++y)for(unsigned int x=x0;x<=x1;++x){
    // 253 is a center-based inflation cost, not another physical obstacle.
    if(map.getCost(x,y)<costmap_2d::LETHAL_OBSTACLE)continue;
    double wx,wy;map.mapToWorld(x,y,wx,wy);
    if(intersects(poly,wx,wy,map.getResolution()/2))++blocked;
  }
  return blocked;
}
bool poseFree(const Pose& pose,const costmap_2d::Costmap2D& map,
              const std::vector<geometry_msgs::Point>& body,double padding) {
  return collisionCells(pose,map,body,padding)==0;
}
bool pathFree(const std::vector<Pose>& path,const costmap_2d::Costmap2D& map,
              const std::vector<geometry_msgs::Point>& body,double padding,Pose* failed_pose) {
  if(path.empty())return false;
  if(!poseFree(path.front(),map,body,padding)){if(failed_pose)*failed_pose=path.front();return false;}
  double radius=0;for(const auto& p:body)radius=std::max(radius,std::hypot(p.x,p.y)+2*padding);
  for(size_t i=1;i<path.size();++i){
    double sweep=distance(path[i-1],path[i])+radius*std::abs(wrap(path[i].yaw-path[i-1].yaw));
    int n=std::max(1,static_cast<int>(std::ceil(sweep/(map.getResolution()/2))));
    for(int j=1;j<=n;++j){auto p=blend(path[i-1],path[i],double(j)/n);if(!poseFree(p,map,body,padding)){if(failed_pose)*failed_pose=p;return false;}}
  }
  return true;
}
bool smooth(const std::vector<Pose>& raw,const costmap_2d::Costmap2D& map,
            const std::vector<geometry_msgs::Point>& body,const Config& cfg,
            std::vector<Pose>& out,Statistics& stats) {
  const auto begin=std::chrono::steady_clock::now();stats=Statistics();out.clear();
  auto elapsed=[&](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();};
  if(raw.size()<2 || cfg.sample_spacing<=0 || body.size()<3)return false;
  std::vector<Pose> clean;
  for(const auto& p:raw){if(!std::isfinite(p.x+p.y+p.yaw))return false;
    if(clean.empty()||distance(clean.back(),p)>1e-5)clean.push_back(p);}
  if(clean.size()<2)return false;
  clean.front()=raw.front();clean.back()=raw.back();
  // Grid traceback appends the exact goal after its final grid point. That
  // short overshoot must not seed a 170-degree tangent reversal near the goal.
  while(clean.size()>2 && distance(clean[clean.size()-2],clean.back())<2*cfg.sample_spacing)
    clean.erase(clean.end()-2);
  while(clean.size()>2 && distance(clean.front(),clean[1])<cfg.sample_spacing)
    clean.erase(clean.begin()+1);
  // Shortcuts stay near the original route, retaining its obstacle-side choice.
  std::vector<Pose> anchors{clean.front()};size_t i=0;
  while(i+1<clean.size()) {
    size_t far=i+1;double length=0;
    while(far+1<clean.size() && (length+=distance(clean[far-1],clean[far]))<cfg.shortcut_length)++far;
    size_t selected=i+1;
    for(size_t j=far;j>i+1;--j) {
      bool near=true;for(size_t k=i+1;k<j;++k)if(lineDeviation(clean[k],clean[i],clean[j])>cfg.shortcut_deviation){near=false;break;}
      if(!near)continue;
      Pose a=clean[i],b=clean[j];a.yaw=b.yaw=std::atan2(b.y-a.y,b.x-a.x);
      if(pathFree({a,b},map,body,cfg.collision_padding)){selected=j;break;}
      if(elapsed()>cfg.max_compute_ms)break;
    }
    anchors.push_back(clean[selected]);i=selected;
    if(elapsed()>cfg.max_compute_ms){anchors.insert(anchors.end(),clean.begin()+i+1,clean.end());break;}
  }
  // Give the terminal curve a feasible approach tangent instead of forcing a
  // sharp orientation change in its last few centimeters. Large final heading
  // reversals remain intentional in-place goal alignment for this diff-drive.
  if(anchors.size()>1 && cfg.goal_tangent_length>0){
    const Pose goal=clean.back();const Pose prev=anchors[anchors.size()-2];
    double incoming=std::atan2(goal.y-prev.y,goal.x-prev.x);
    double change=std::abs(wrap(goal.yaw-incoming));
    if(change>.15 && change<1.65){
      for(double length=cfg.goal_tangent_length;length>=.075;length*=.75){
        Pose approach{goal.x-length*std::cos(goal.yaw),goal.y-length*std::sin(goal.yaw),goal.yaw};
        if(!pathFree({approach,goal},map,body,cfg.collision_padding))continue;
        anchors.pop_back();
        while(anchors.size()>1 && distance(anchors.back(),goal)<1.6*length)anchors.pop_back();
        anchors.push_back(approach);anchors.push_back(goal);break;
      }
    }
  }
  out.push_back(anchors.front());
  for(size_t k=1;k+1<anchors.size();++k){
    Pose a=anchors[k-1],b=anchors[k],c=anchors[k+1];
    double l1=distance(a,b),l2=distance(b,c);
    double ux=(b.x-a.x)/l1,uy=(b.y-a.y)/l1,vx=(c.x-b.x)/l2,vy=(c.y-b.y)/l2;
    double angle=std::acos(std::max(-1.,std::min(1.,ux*vx+uy*vy)));
    bool rounded=false;
    if(angle>.10 && angle<2.60 && elapsed()<cfg.max_compute_ms){
      double trim=std::min({cfg.preferred_radius*std::tan(angle/2),.45*l1,.45*l2});
      for(int trial=0;trial<6 && trim>.015;++trial,trim*=.75){
        Pose entry{b.x-trim*ux,b.y-trim*uy,std::atan2(uy,ux)},exit{b.x+trim*vx,b.y+trim*vy,std::atan2(vy,vx)};
        double radius=trim/std::tan(angle/2),handle=4./3.*radius*std::tan(angle/4);
        Pose p1{entry.x+handle*ux,entry.y+handle*uy,0},p2{exit.x-handle*vx,exit.y-handle*vy,0};
        std::vector<Pose> curve;int count=std::max(3,static_cast<int>(std::ceil(radius*angle/cfg.sample_spacing)));
        for(int n=0;n<=count;++n){double t=double(n)/count,z=1-t;
          double x=z*z*z*entry.x+3*z*z*t*p1.x+3*z*t*t*p2.x+t*t*t*exit.x;
          double y=z*z*z*entry.y+3*z*z*t*p1.y+3*z*t*t*p2.y+t*t*t*exit.y;
          double dx=3*z*z*(p1.x-entry.x)+6*z*t*(p2.x-p1.x)+3*t*t*(exit.x-p2.x);
          double dy=3*z*z*(p1.y-entry.y)+6*z*t*(p2.y-p1.y)+3*t*t*(exit.y-p2.y);
          curve.push_back({x,y,std::atan2(dy,dx)});
        }
        if(pathFree(curve,map,body,cfg.smoothing_clearance)){
          sampleLine(out.back(),entry,cfg.sample_spacing,out);out.insert(out.end(),curve.begin()+1,curve.end());
          rounded=true;++stats.rounded_corners;break;
        }
        if(elapsed()>cfg.max_compute_ms)break;
      }
    }
    if(!rounded)sampleLine(out.back(),b,cfg.sample_spacing,out);
  }
  sampleLine(out.back(),anchors.back(),cfg.sample_spacing,out);
  out=tangentPath(out,raw.front(),raw.back());
  if(!pathFree(out,map,body,cfg.collision_padding,&stats.failed_pose) &&
      poseFree(out.front(),map,body,cfg.collision_padding) && poseFree(out.back(),map,body,cfg.collision_padding)) {
    // A center-safe grid route can still clip a wall with its rotating corners.
    // Move a bounded, broad section of the curve rather than shrinking the body.
    const auto reference=out;
    std::vector<double> arc(out.size(),0);
    for(size_t j=1;j<out.size();++j)arc[j]=arc[j-1]+distance(out[j-1],out[j]);
    auto score=[&](const std::vector<Pose>& path){double value=0;
      for(size_t j=0;j<path.size();++j){double pad=(j==0||j+1==path.size())?cfg.collision_padding:cfg.smoothing_clearance;
        value+=100.*collisionCells(path[j],map,body,pad);
        if(j)value+=.2*std::pow(wrap(path[j].yaw-path[j-1].yaw),2);
      }return value;};
    double value=score(out);
    for(int iteration=0;iteration<20 && elapsed()<cfg.max_compute_ms;++iteration) {
      if(pathFree(out,map,body,cfg.collision_padding,&stats.failed_pose))break;
      size_t center=0;double nearest=1e30;
      for(size_t j=1;j+1<out.size();++j){double d=distance(out[j],stats.failed_pose);if(d<nearest){nearest=d;center=j;}}
      std::vector<Pose> best;double best_value=value;
      for(int direction=0;direction<8 && elapsed()<cfg.max_compute_ms;++direction){
        auto candidate=out;double a=direction*M_PI/4;
        for(size_t j=1;j+1<out.size();++j){double ds=std::abs(arc[j]-arc[center]);if(ds>.65)continue;
          double scale=.5*(1+std::cos(M_PI*ds/.65))*std::min({1.,arc[j]/.25,(arc.back()-arc[j])/.25});
          candidate[j].x+=.025*scale*std::cos(a);candidate[j].y+=.025*scale*std::sin(a);
          if(distance(candidate[j],reference[j])>cfg.max_lateral_adjustment){candidate[j]=out[j];}
        }
        candidate=tangentPath(candidate,raw.front(),raw.back());double cost=score(candidate);
        if(cost<best_value-1e-8){best_value=cost;best=std::move(candidate);}
      }
      if(best.empty())break;
      out=std::move(best);value=best_value;++stats.repair_steps;
    }
  }
  if(!pathFree(out,map,body,cfg.collision_padding,&stats.failed_pose)){
    stats.fallback=true;out=tangentPath(clean,raw.front(),raw.back());
    if(!pathFree(out,map,body,cfg.collision_padding)){out.clear();stats.elapsed_ms=elapsed();return false;}
  }
  stats.elapsed_ms=elapsed();return true;
}
}
