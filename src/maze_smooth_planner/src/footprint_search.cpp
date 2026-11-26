#include <maze_smooth_planner/smoother.h>
#include <costmap_2d/cost_values.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>

namespace maze_smooth_planner {
namespace {
// Interpolate each candidate at the same maximum sweep step as pathFree.
bool paddingEscape(const Pose& start, const costmap_2d::Costmap2D& map,
                   const std::vector<geometry_msgs::Point>& body, const Config& cfg,
                   Pose& escape, double& reverse_used) {
  double radius = 0.0;
  for (const auto& p : body)
    radius = std::max(radius, std::hypot(p.x, p.y) + 2 * cfg.collision_padding);
  double best = std::numeric_limits<double>::infinity();
  const int initial = collisionCells(start, map, body, cfg.collision_padding);
  const auto consider = [&](double ds, double dyaw) {
    const double sweep = std::abs(ds) + radius * std::abs(dyaw);
    if (sweep >= best || (ds < 0 && (!cfg.allow_reverse || -ds > cfg.max_reverse_distance))) return;
    const double dx = ds * std::cos(start.yaw), dy = ds * std::sin(start.yaw);
    const Pose end{start.x + dx, start.y + dy, start.yaw + dyaw};
    if (!poseFree(end, map, body, cfg.collision_padding) ||
        !pathFree({start, end}, map, body, 0.0)) return;
    const int steps = std::max(1, static_cast<int>(std::ceil(sweep / (map.getResolution() / 2))));
    int previous = initial;
    for (int i = 1; i <= steps; ++i) {
      const double t = static_cast<double>(i) / steps;
      const int hits = collisionCells({start.x + t * dx, start.y + t * dy, start.yaw + t * dyaw},
                                      map, body, cfg.collision_padding);
      if (hits > previous) return;
      previous = hits;
    }
    best = sweep;
    escape = end;
    reverse_used = std::max(0.0, -ds);
  };
  // A differential-drive vehicle can translate only along its heading.
  for (double ds : {.05, .10, .15, -.05, -.10, -.15, -.20, -.25}) consider(ds, 0.0);
  for (int degrees : {10, -10, 20, -20, 30, -30}) consider(0.0, degrees * M_PI / 180);
  return std::isfinite(best);
}
}  // namespace

bool footprintReplan(const Pose& start, const Pose& goal,
                     const costmap_2d::Costmap2D& map,
                     const std::vector<geometry_msgs::Point>& body,
                     const Config& cfg, std::vector<Pose>& out) {
  out.clear();
  if (!std::isfinite(cfg.replan_compute_ms) || cfg.replan_compute_ms <= 0 ||
      !poseFree(goal, map, body, cfg.collision_padding)) return false;
  const auto begin=std::chrono::steady_clock::now();
  Pose search_start = start;
  double initial_reverse = 0.0;
  const bool start_escaped = !poseFree(start, map, body, cfg.collision_padding);
  if (start_escaped &&
      (!cfg.allow_start_padding_escape || !poseFree(start, map, body, 0.0) ||
       !paddingEscape(start, map, body, cfg, search_start, initial_reverse))) return false;
  const double step=2*map.getResolution();
  const int nx=(map.getSizeInCellsX()+1)/2, ny=(map.getSizeInCellsY()+1)/2;
  if (nx<1 || ny<1) return false;
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
  std::vector<int> coarse_clearance(static_cast<size_t>(nx)*ny,nx+ny);
  std::queue<int> wave;
  for (int y=0; y<ny; ++y) {
    if (elapsed() >= cfg.replan_compute_ms) return false;
    for (int x=0; x<nx; ++x) {
      bool blocked = x==0 || y==0 || x==nx-1 || y==ny-1;
      for (int yy=2*y; !blocked && yy<std::min(2*y+2,mh); ++yy)
        for (int xx=2*x; xx<std::min(2*x+2,mw); ++xx)
          blocked |= map.getCost(xx,yy) >= costmap_2d::LETHAL_OBSTACLE;
      if (blocked) {
        const int id = y*nx+x;
        coarse_clearance[id] = 0;
        wave.push(id);
      }
    }
  }
  std::size_t wave_steps = 0;
  while(!wave.empty()){
    if ((wave_steps++ & 255) == 0 && elapsed() >= cfg.replan_compute_ms) return false;
    int id=wave.front();wave.pop();int x=id%nx,y=id/nx;
    for(int h=0;h<8;++h){int xx=x+dx[h],yy=y+dy[h];
      if(!inside(xx, yy))continue;
      int next=yy*nx+xx;
      if(coarse_clearance[next]>coarse_clearance[id]+1){
        coarse_clearance[next]=coarse_clearance[id]+1;wave.push(next);
      }
    }
  }
  double radius=0;
  for(const auto& p:body)radius=std::max(radius,std::hypot(p.x,p.y)+2*cfg.collision_padding);
  auto room=[&](const Pose& p){unsigned int x,y;
    return map.worldToMap(p.x,p.y,x,y) ? (coarse_clearance[(y/2)*nx+x/2]-std::sqrt(2.))*step : 0.;};
  std::vector<double> distance(count,std::numeric_limits<double>::infinity());
  std::vector<int> previous(count,-2);
  std::vector<bool> settled(count, false);
  std::vector<double> reverse_used(cfg.allow_reverse ? count : 0, 0.0);
  std::vector<signed char> free_cache(count,-1);
  auto free=[&](int id){
    if(free_cache[id]<0) free_cache[id]=(room(pose(id))>radius || poseFree(pose(id),map,body,cfg.collision_padding))?1:0;
    return free_cache[id]==1;
  };
  using Item=std::pair<double,int>;
  std::priority_queue<Item,std::vector<Item>,std::greater<Item>> open;
  // A bounded reverse Dijkstra supplies a wall-aware guide to the heading
  // search. The guide knows only the center cell; every accepted path still
  // needs the original footprint checks.
  const int unreachable = std::numeric_limits<int>::max();
  std::vector<int> reverse_distance(count / 8, unreachable);
  using GridItem = std::pair<int, int>;
  std::priority_queue<GridItem, std::vector<GridItem>, std::greater<GridItem>> reverse_open;
  const int gx = std::lround((goal.x - ox) / step);
  const int gy = std::lround((goal.y - oy) / step);
  const auto open_cell = [&](int x, int y) {
    return inside(x, y) && map.getCost(2 * x, 2 * y) < costmap_2d::LETHAL_OBSTACLE;
  };
  for (int y = gy - 1; y <= gy + 1; ++y) {
    for (int x = gx - 1; x <= gx + 1; ++x) {
      if (!open_cell(x, y)) continue;
      const int cell = y * nx + x;
      const auto p = pose(cell * 8);
      const double proximity = std::hypot(p.x - goal.x, p.y - goal.y);
      if (proximity > 1.5 * step) continue;
      reverse_distance[cell] = std::lround(proximity * 10 / step);
      reverse_open.emplace(reverse_distance[cell], cell);
    }
  }
  const double guide_deadline = elapsed() + std::min(8.0, .12 * cfg.replan_compute_ms);
  std::size_t guide_steps = 0;
  while (!reverse_open.empty()) {
    if ((guide_steps++ & 127) == 0 &&
        elapsed() >= std::min(guide_deadline, cfg.replan_compute_ms)) break;
    const auto item = reverse_open.top();
    reverse_open.pop();
    const int cost = item.first, cell = item.second;
    if (cost != reverse_distance[cell]) continue;
    const int x = cell % nx, y = cell / nx;
    for (int h = 0; h < 8; ++h) {
      const int xx = x + dx[h], yy = y + dy[h];
      if (!open_cell(xx, yy)) continue;
      const bool diagonal = dx[h] && dy[h];
      if (diagonal && (!open_cell(xx, y) || !open_cell(x, yy))) continue;
      const int next = yy * nx + xx;
      const int candidate = cost + (diagonal ? 14 : 10);
      if (candidate >= reverse_distance[next]) continue;
      reverse_distance[next] = candidate;
      reverse_open.emplace(candidate, next);
    }
  }
  const auto distance_to_goal = [&](int id) {
    const auto p = pose(id);
    return std::hypot(p.x - goal.x, p.y - goal.y);
  };
  const auto heuristic = [&](int id) {
    const int guide = reverse_distance[id / 8];
    const double direct = distance_to_goal(id);
    return 1.25 * (guide == unreachable ? direct : std::max(direct, .1 * step * guide));
  };
  const auto motion = [](const Pose& a, const Pose& b) {
    return (b.x-a.x)*std::cos(a.yaw)+(b.y-a.y)*std::sin(a.yaw);
  };
  const auto bounded = [&](double used, double ds) {
    return ds >= -1e-9 || (cfg.allow_reverse && used - ds <= cfg.max_reverse_distance + 1e-6);
  };
  int sx=std::lround((search_start.x-ox)/step),sy=std::lround((search_start.y-oy)/step);
  int sh=(static_cast<int>(std::lround(search_start.yaw/(M_PI/4)))%8+8)%8;
  // Connect the exact robot pose; never quantize its published start away.
  for(int y=sy-1;y<=sy+1;++y)for(int x=sx-1;x<=sx+1;++x){
    if(!inside(x,y))continue;
    int id=index(x,y,sh);auto p=pose(id);
    const double length = std::hypot(p.x-search_start.x,p.y-search_start.y);
    const double projection = motion(search_start, p);
    const double ds = projection < -1e-9 ? -length : projection;
    if ((cfg.allow_reverse || start_escaped) && !bounded(initial_reverse, ds)) continue;
    if(free(id) && pathFree({search_start,p},map,body,cfg.collision_padding)){
      distance[id]=length;
      if (cfg.allow_reverse) {
        if (ds < -1e-9) distance[id] *= cfg.reverse_cost_multiplier;
        reverse_used[id] = ds > 1e-9 ? 0.0 : initial_reverse + std::max(0.0, -ds);
      }
      previous[id]=-1;
      open.push({distance[id]+heuristic(id),id});
    }
  }
  int finish=-1;
  while(!open.empty() && elapsed()<cfg.replan_compute_ms){
    auto item=open.top();open.pop();int id=item.second;
    if(settled[id] || item.first>distance[id]+heuristic(id)+1e-8)continue;
    settled[id] = true;
    auto a=pose(id);int h=id%8,cell=id/8,x=cell%nx,y=cell/nx;
    const double used = cfg.allow_reverse ? reverse_used[id] : 0.0;
    const double goal_distance = distance_to_goal(id);
    const double goal_motion = motion(a, goal) < -1e-9 ? -goal_distance : 0.0;
    if(goal_distance<=1.5*step &&
       ((!cfg.allow_reverse && !start_escaped) || bounded(used, goal_motion)) &&
       pathFree({a,goal},map,body,cfg.collision_padding)){
      finish=id;break;
    }
    const int moves = cfg.allow_reverse && cfg.max_reverse_distance >= step ? 4 : 3;
    for(int move=0;move<moves;++move){
      int xx=x,yy=y,hh=h;
      if(move==0){xx+=dx[h];yy+=dy[h];}
      else if(move==3){xx-=dx[h];yy-=dy[h];}
      else hh=(h+(move==1?1:7))%8;
      if(!inside(xx,yy))continue;
      int next=index(xx,yy,hh);if(settled[next])continue;
      auto b=pose(next);
      const double length = std::hypot(b.x-a.x,b.y-a.y);
      if (move==3 && !bounded(used,-length)) continue;
      double edge = move==3 ? length*cfg.reverse_cost_multiplier : move==0 ? length : .12;
      // Retain the costmap preference for wider corridors as a soft cost.
      unsigned int mx,my;
      if(move==0 && map.worldToMap(b.x,b.y,mx,my))
        edge*=1+static_cast<double>(std::min<int>(map.getCost(mx,my),252))/126;
      double candidate=distance[id]+edge;
      if(candidate>=distance[next]-1e-9 || !free(next))continue;
      // Check every translation and rotational sweep, not only node poses.
      if(room(a)<=radius+std::hypot(b.x-a.x,b.y-a.y) &&
         !pathFree({a,b},map,body,cfg.collision_padding))continue;
      distance[next]=candidate;previous[next]=id;
      if (cfg.allow_reverse) reverse_used[next] = move==0 ? 0.0 : used+(move==3 ? length : 0.0);
      open.push({candidate+heuristic(next),next});
    }
  }
  if(finish<0)return false;
  for(int id=finish;id>=0;id=previous[id])out.push_back(pose(id));
  std::reverse(out.begin(),out.end());out.insert(out.begin(),search_start);out.push_back(goal);
  // A final independent complete-path check also covers exact connectors.
  if(!pathFree(out,map,body,cfg.collision_padding)){out.clear();return false;}
  if (start_escaped) {
    if (!pathFree({start, search_start}, map, body, 0.0)) {out.clear();return false;}
    out.insert(out.begin(), start);
  }
  return true;
}
}  // namespace maze_smooth_planner
