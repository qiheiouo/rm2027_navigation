#include "sfcSquare.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <vector>
int main(int argc,char**argv){
 std::ifstream f(argv[1],std::ios::binary);std::string magic;int w,h,max;f>>magic>>w>>h>>max;f.get();std::vector<unsigned char> pgm(w*h);f.read(reinterpret_cast<char*>(pgm.data()),pgm.size());
 const int pad=4,W=w+2*pad,H=h+2*pad;std::vector<unsigned char> map(W*H,0);
 for(int y=0;y<h;++y)for(int x=0;x<w;++x)map[(y+pad)*W+x+pad]=pgm[(h-1-y)*w+x]==254?255:0;
 SfcSquare provider(W,H,map.data(),.05f);auto raw=provider.getBound(2.f+.175f,4.f+.175f,2.f,0.f);
 double b[4]={raw[0]-2.-.175,raw[1]-4.-.175,raw[2]-2.-.175,raw[3]-4.-.175};
 std::cout<<std::setprecision(17)<<"{\"raw_world_bounds_xyxy\":["<<b[0]<<","<<b[1]<<","<<b[2]<<","<<b[3]<<"],\"occupied_in_closed_rectangle\":[";
 bool first=true;for(int y=std::floor((b[1]+4.-1e-6)/.05);y<=std::floor((b[3]+4.+1e-6)/.05);++y)for(int x=std::floor((b[0]+2.-1e-6)/.05);x<=std::floor((b[2]+2.+1e-6)/.05);++x)if(pgm[(h-1-y)*w+x]!=254){if(!first)std::cout<<",";first=false;std::cout<<"["<<x<<","<<y<<"]";}
 std::cout<<"]}\n";
}
