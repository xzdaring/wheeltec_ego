// 回归：负方向栅格取整不得把空闲起点平移进墙；搜索结果仍必须绕墙。
#include <path_searching/dyn_a_star.h>
#include <cassert>
#include <iostream>
int main(){
 auto grid=std::make_shared<GridMap2D>(.1,Eigen::Vector2i(10,10));grid->setCurPose(0,0);
 grid->setOccupancyQuery([](const Eigen::Vector2d&p){return p.x()>=.12 && p.x()<=.79 && std::abs(p.y())<.45;});
 AStar search;search.initGridMap(grid,Eigen::Vector2i(100,100));
 assert(search.AstarSearch(.1,Eigen::Vector2d(0,0),Eigen::Vector2d(.9,0)));
 auto path=search.getPath();assert(!path.empty());assert(path.front().norm()<.1);
 for(const auto&p:path)assert(!grid->getInflateOccupancy(p));
 std::cout<<"PASS negative-grid rounding and Astar detour, points="<<path.size()<<std::endl;
}
