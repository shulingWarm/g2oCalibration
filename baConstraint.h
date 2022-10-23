#ifndef _BA_CONSTRAINT_H_
#define _BA_CONSTRAINT_H_
#include"totalIntrinsic.h"
#include"externalVertex.h"
#include<g2o/core/base_binary_edge.h>
#include"extrinsicTranspose.h"
#include"intrinsicTranspose.h"

//对所有的点做重投影误差时使用的误差函数
class BAConstraint : public g2o::BaseBinaryEdge<2,double,TotalIntrinsic,ExternalVertex>
{
    //点在相机坐标系下的坐标
    const Eigen::Vector2d pixPoint_;
    //标定板上点的坐标
    const Eigen::Vector3d worldPoint_;
public:
    BAConstraint(const Eigen::Vector2d& pixPoint,
                 const Eigen::Vector3d& worldPoint) : pixPoint_(pixPoint),worldPoint_(worldPoint){}

    //计算误差的函数，使用重投影误差
    void computeError()
    {
        //获得内参节点
        const TotalIntrinsic* intrVertex=(const TotalIntrinsic*)(_vertices[0]);
        //获得外参节点
        const ExternalVertex* exterinsicVertex=(const ExternalVertex*)(_vertices[1]);
        //临时的print调试，打印世界点坐标
        //std::cout<<"world point "<<worldPoint_<<std::endl;
        //把三维世界点转换到相机坐标系下
        Eigen::Vector3d camPoint=transWorldToCamera(worldPoint_,
                                                    exterinsicVertex->estimate());
        //为了测试是不是约束函数写错了，临时把error写成固定期望值
//        _error[0]=intrVertex->estimate()[0]-10;
//        _error[1]=intrVertex->estimate()[1]-20;
//        return;
        //如果是无穷远点，就把误差改到最大
        if(camPoint[2]==0)
        {
            //std::cerr<<"infinate point"<<std::endl;
            _error[0]=1e10;
            _error[1]=1e10;
        }
        else
        {
            //把相机坐标系下的点转换到世界坐标系下
            Eigen::Vector2d pixProjection=
                    intrinsicTranspose(intrVertex->estimate(),camPoint);
            //根据投影的位置计算绝对的误差
            _error(0)=(pixProjection[0]-pixPoint_[0]);
            _error(1)=(pixProjection[1]-pixPoint_[1]);
        }
    }

    //优化边的读写相关函数，留空
    virtual bool read(std::istream &is){return true;}
    virtual bool write(std::ostream &os) const{return true;}
};

#endif
