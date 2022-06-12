#ifndef _INTR_CAMPOINT_EDGE_H_
#define _INTR_CAMPOINT_EDGE_H_
#include"IntrinsicNode.h"
#include"cameraPoint.h"
#include<g2o/core/base_binary_edge.h>
#include<array>

class IntrCampointEdge : public g2o::BaseBinaryEdge<2,double,IntrinsicVertex,
        CameraPointVertex>
{
protected:
    //点在像素坐标系下的投影点
    const Eigen::Vector2d pixelProjection_;

public:
    //约束边的构造函数，记录投影点
    IntrCampointEdge(const Eigen::Vector2d& pixelProjection) : pixelProjection_(pixelProjection){}

    //计算误差的函数，属于是最核心函数了
    void computeError();

    //优化边的读写相关函数，留空
    virtual bool read(std::istream &is){return true;}
    virtual bool write(std::ostream &os) const{return true;}
};

//根据内参把点投影到像素坐标系下
Eigen::Vector2d getPixelProjection(const Eigen::Vector3d& intrinsic,
                                   const Eigen::Vector2d& camPoint)
{
    //初始化用于返回的结果变量
    Eigen::Vector2d projection;
    //计算投影的位置
    projection<<camPoint[0]*intrinsic[0]+intrinsic[1],
            camPoint[1]*intrinsic[0]+intrinsic[2];
    //返回投影结果
    return projection;
}

//计算误差的函数，属于是最核心函数了
void IntrCampointEdge::computeError()
{
    //获取内参节点
    const IntrinsicVertex* intrPtr=(const IntrinsicVertex*)(_vertices[0]);
    //相机坐标系下的节点
    const CameraPointVertex* camPointPtr=(const CameraPointVertex*)(_vertices[1]);
    //获取点在相机坐标系下的投影
    Eigen::Vector2d pixelProjectionEsti=getPixelProjection(
                intrPtr->estimate(),camPointPtr->estimate());
    //根据投影位置分别计算误差
    _error(0)=std::abs(pixelProjectionEsti[0]-pixelProjection_[0]);
    _error(1)=std::abs(pixelProjectionEsti[1]-pixelProjection_[1]);
}

#endif
