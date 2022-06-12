#ifndef _INTRINSIC_NODE_H_
#define _INTRINSIC_NODE_H_
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

//内参的节点
class IntrinsicVertex : public g2o::BaseVertex<3,Eigen::Vector3d>
{
public:
    //给变量设置的初值
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl()
    {
        //设置待估计的变量
        _estimate<<10,10,10;
    }

    //待优化变量的更新函数，这个选项在ceres那边是似乎是没有的
    virtual void oplusImpl(const double* update)
    {
        //把准备用来更新的变量增加到默认的变量上面
        _estimate+=Eigen::Vector3d(update);
    }

    //关于存盘和读盘的函数,因为这两个函数实际上是不会被用到的，所以在这里不写实现
    virtual bool read(std::istream &in){return true;}
    virtual bool write(std::ostream &out) const{return true;}
};


#endif
