#ifndef _CAMERA_POINT_H_
#define _CAMERA_POINT_H_
#include<iostream>
#include<g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include<Eigen/Core>
#include<math.h>
#include<vector>

//每个相机坐标系下的坐标只有两个维度
class CameraPointVertex : public g2o::BaseVertex<2,Eigen::Vector2d>
{
protected:
    //点坐标的初始值
    Eigen::Vector2d initPoint_;
public:
    //待优化变量的初始值，这里其实是一个相对固定的模板，格式上的是固定的，名字也不能随便改
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl()
    {
        //把待估计变量设置为0
        _estimate=initPoint_;
    }

    //待优化变量的更新函数，这个选项在ceres那边是似乎是没有的
    virtual void oplusImpl(const double* update)
    {
        //把准备用来更新的变量增加到默认的变量上面
        _estimate+=Eigen::Vector2d(update);
    }

    //关于存盘和读盘的函数,因为这两个函数实际上是不会被用到的，所以在这里不写实现
    virtual bool read(std::istream &in){return true;}
    virtual bool write(std::ostream &out) const{return true;}

    //构造函数，输出一个初值，初始化的时候会使用这个值
    CameraPointVertex(const Eigen::Vector2d& initPoint):initPoint_(initPoint)
    {
        setToOriginImpl();
    }

    //空的构造函数
    CameraPointVertex()
    {
        initPoint_<<0,0;
    }
};

#endif
