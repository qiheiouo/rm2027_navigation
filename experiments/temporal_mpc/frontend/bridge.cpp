// Copyright 2026 RM Navigation. SPDX-License-Identifier: MIT
// Offline bridge to the already migrated T-DT libraries. No velocity output.
#include "YAstar/yastar.hpp"
#include "MinimumSnapOsqp/sfcSquare.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>

int main() {
  int w, h; float r, ox, oy, sx, sy, gx, gy;
  if (!(std::cin >> w >> h >> r >> ox >> oy >> sx >> sy >> gx >> gy) ||
      w < 3 || h < 3 || w > 1000 || h > 1000 || w*h > 100000 ||
      !std::isfinite(ox) || !std::isfinite(oy) || !std::isfinite(sx) || !std::isfinite(sy) ||
      !std::isfinite(gx) || !std::isfinite(gy) || !(r >= .01f && r <= .2f)) return 2;
  std::vector<u_char> raw(w*h);
  for (auto &v : raw) {int cost; if (!(std::cin >> cost) || cost<0 || cost>255) return 2;
    v = cost >= 253 ? 0 : 255; } // Unknown stays blocked.
  for(int y=0;y<h;y++) for(int x=0;x<w;x++)
    if(x==0 || y==0 || x==w-1 || y==h-1) raw[y*w+x]=0;
  YAstar field(w,h,r,ox,oy); field.setMap(w,h,raw.data()); auto sdf=field.getSDF();
  // Configuration-space centres: complete padded circumscribed footprint,
  // clearance, and both cell half diagonals. Never expose raw-free corridors.
  const float reach=std::hypot(.355f,.330f)+.02f+std::sqrt(2.f)*r;
  std::vector<u_char> free(w*h);
  for(int i=0;i<w*h;i++) free[i]=raw[i] && sdf.data()[i]>reach+1e-6f ? 255 : 0;
  YAstar planner(w,h,r,ox,oy); planner.setMap(w,h,free.data());
  planner.setCostWeight(.5f); planner.initCostMap();
  auto begin=std::chrono::steady_clock::now();
  auto path=planner.search({sx,sy},{gx,gy},[&](){return
    std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count()>.25;});
  if(path.empty()) return 3;
  path=planner.simplifyPath(path,.1f);
  path.front()={sx,sy};path.back()={gx,gy};
  for(size_t i=1;i<path.size();i++) if(planner.lineInObsticle(path[i-1],path[i])) return 4;
  // SfcSquare's frozen offset convention differs from YAstar's map origin.
  // Keep its input in local grid coordinates and explicitly translate output.
  SfcSquare corridor(w,h,free.data(),r,{0.f,0.f});
  std::cout<<std::setprecision(9)<<"{\"path\":[";
  for(size_t i=0;i<path.size();i++) {
    if(i) std::cout<<',';std::cout<<'['<<path[i].x()<<','<<path[i].y()<<']';
  }
  std::cout<<"],\"anchors\":["; bool first=true;
  for(size_t i=1;i<path.size();i++) {
    const auto delta=path[i]-path[i-1];int n=std::max(1,(int)std::ceil(delta.norm()/.1f));
    for(int k=(i==1?0:1);k<=n;k++) {
      Eigen::Vector2f p=path[i-1]+delta*((float)k/n);
      auto b=corridor.getBound(p.x()-ox,p.y()-oy,2.f,.01f);
      b[0]+=ox;b[2]+=ox;b[1]+=oy;b[3]+=oy;
      if(!(b[0]<p.x() && p.x()<b[2] && b[1]<p.y() && p.y()<b[3])) return 5;
      if(!first) std::cout<<',';first=false;
      std::cout<<"{\"position\":["<<p.x()<<','<<p.y()<<"],\"centre_bounds\":["
        <<b[0]<<','<<b[2]<<','<<b[1]<<','<<b[3]<<"]}";
    }
  }
  std::cout<<"],\"inflation_reach_m\":"<<reach<<"}\n";
}
