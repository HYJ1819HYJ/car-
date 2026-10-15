#include <maze_smooth_planner/smoother.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

namespace maze_smooth_planner {
bool footprintReplan(const Pose& start, const Pose& goal,
                     const costmap_2d::Costmap2D& map,
                     const std::vector<geometry_msgs::Point>& body,
                     const Config& cfg, std::vector<Pose>& out) {
  out.clear();
  if (!poseFree(start,map,body,cfg.collision_padding) ||
      !poseFree(goal,map,body,cfg.collision_padding)) return false;
  const auto begin=std::chrono::steady_clock::now();
  const double step=2*map.getResolution();
  const int nx=(map.getSizeInCellsX()+1)/2, ny=(map.getSizeInCellsY()+1)/2;
  if (nx<1 || ny<1 || !std::isfinite(cfg.replan_compute_ms) || cfg.replan_compute_ms<=0) return false;
  const size_t count=static_cast<size_t>(nx)*ny*8;
  // Bound allocations on accidentally enormous maps as well as search time.
  if (count>4000000) return false;
  const double ox=map.getOriginX()+map.getResolution()/2;
  const double oy=map.getOriginY()+map.getResolution()/2;
  const int dx[8]={1,1,0,-1,-1,-1,0,1};
  const int dy[8]={0,1,1,1,0,-1,-1,-1};
  auto index=[&](int x,int y,int h){return (y*nx+x)*8+h;};
  auto pose=[&](int id){int h=id%8,cell=id/8;return Pose{ox+(cell%nx)*step,oy+(cell/nx)*step,h*M_PI/4};};
  auto inside=[&](int x,int y){return x>=0 && y>=0 && x<nx && y<ny;};
  auto elapsed=[&](){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();};
  // Chebyshev distance to occupied/unknown cells is a conservative lower
  // bound on Euclidean clearance. It permits cheap whole-circle proofs in
  // open space; near walls the exact polygon/sweep checks still run.
  const int mw=map.getSizeInCellsX(),mh=map.getSizeInCellsY();
  std::vector<int> clearance(static_cast<size_t>(mw)*mh,mw+mh);
  std::queue<int> wave;
  for(int y=0;y<mh;++y)for(int x=0;x<mw;++x){
    if(x==0 || y==0 || x==mw-1 || y==mh-1 || map.getCost(x,y)>=254){
      int id=y*mw+x;clearance[id]=0;wave.push(id);
    }
  }
  while(!wave.empty()){
    int id=wave.front();wave.pop();int x=id%mw,y=id/mw;
    for(int h=0;h<8;++h){int xx=x+dx[h],yy=y+dy[h];
      if(xx<0 || yy<0 || xx>=mw || yy>=mh)continue;
      int next=yy*mw+xx;
      if(clearance[next]>clearance[id]+1){clearance[next]=clearance[id]+1;wave.push(next);}
    }
  }
  double radius=0;
  for(const auto& p:body)radius=std::max(radius,std::hypot(p.x,p.y)+2*cfg.collision_padding);
  auto room=[&](const Pose& p){unsigned int x,y;
    return map.worldToMap(p.x,p.y,x,y)?clearance[y*mw+x]*map.getResolution()-map.getResolution()*std::sqrt(2.)/2:0.;};
  std::vector<double> distance(count,std::numeric_limits<double>::infinity());
  std::vector<int> previous(count,-2);
  std::vector<signed char> free_cache(count,-1);
  auto free=[&](int id){
    if(free_cache[id]<0) free_cache[id]=(room(pose(id))>radius || poseFree(pose(id),map,body,cfg.collision_padding))?1:0;
    return free_cache[id]==1;
  };
  using Item=std::pair<double,int>;
  std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
  auto heuristic=[&](int id){auto p=pose(id);return std::hypot(p.x-goal.x,p.y-goal.y);};
  int sx=std::lround((start.x-ox)/step),sy=std::lround((start.y-oy)/step);
  int sh=(static_cast<int>(std::lround(start.yaw/(M_PI/4)))%8+8)%8;
  // Connect the exact robot pose; never quantize its published start away.
  for(int y=sy-1;y<=sy+1;++y)for(int x=sx-1;x<=sx+1;++x){
    if(!inside(x,y))continue;
    int id=index(x,y,sh);auto p=pose(id);
    if(free(id) && pathFree({start,p},map,body,cfg.collision_padding)){
      distance[id]=std::hypot(p.x-start.x,p.y-start.y);previous[id]=-1;
      open.push({distance[id]+heuristic(id),id});
    }
  }
  int finish=-1;
  while(!open.empty() && elapsed()<cfg.replan_compute_ms){
    auto item=open.top();open.pop();int id=item.second;
    if(item.first>distance[id]+heuristic(id)+1e-8)continue;
    auto a=pose(id);int h=id%8,cell=id/8,x=cell%nx,y=cell/nx;
    if(heuristic(id)<=1.5*step && pathFree({a,goal},map,body,cfg.collision_padding)){
      finish=id;break;
    }
    // Differential-drive primitives: forward in the current heading, or
    // rotate in place. No reverse or side-step is admitted by this search.
    for(int move=0;move<3;++move){
      int xx=x,yy=y,hh=h;
      if(move==0){xx+=dx[h];yy+=dy[h];}
      else hh=(h+(move==1?1:7))%8;
      if(!inside(xx,yy))continue;
      int next=index(xx,yy,hh);if(!free(next))continue;
      auto b=pose(next);
      double edge=move==0?std::hypot(b.x-a.x,b.y-a.y):.12;
      // Retain the costmap preference for wider corridors as a soft cost.
      unsigned int mx,my;
      if(move==0 && map.worldToMap(b.x,b.y,mx,my))
        edge*=1+static_cast<double>(std::min<int>(map.getCost(mx,my),252))/126;
      double candidate=distance[id]+edge;
      if(candidate>=distance[next]-1e-9)continue;
      // Check every translation and rotational sweep, not only node poses.
      if(room(a)<=radius+std::hypot(b.x-a.x,b.y-a.y) &&
         !pathFree({a,b},map,body,cfg.collision_padding))continue;
      distance[next]=candidate;previous[next]=id;
      open.push({candidate+heuristic(next),next});
    }
  }
  if(finish<0)return false;
  for(int id=finish;id>=0;id=previous[id])out.push_back(pose(id));
  std::reverse(out.begin(),out.end());out.insert(out.begin(),start);out.push_back(goal);
  // A final independent complete-path check also covers exact connectors.
  if(!pathFree(out,map,body,cfg.collision_padding)){out.clear();return false;}
  return true;
}
}  // namespace maze_smooth_planner
