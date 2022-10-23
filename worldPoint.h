#ifndef _WORLD_POINT_H_
#define _WORLD_POINT_H_
#include"types.h"
#include<unordered_map>

//世界点
class WorldPoint
{
public:
    //世界点的坐标
    Eigen::Vector3d coord_;
    //能观察到这个点的所有相机
    std::unordered_map<unsigned,Eigen::Vector2d> observations_;
};

#endif
