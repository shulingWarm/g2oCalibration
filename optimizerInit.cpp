#include<iostream>
#include"optimizerInit.h"

//对g2o优化器的初始化
//初始化优化器
void initG2oOptimizer(g2o::SparseOptimizer& optimizer)
{
    //指定待优化的图的类型
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> Block;
        //根据图优化类型，定义线性方程求解器,根据g2o的调用规则，这里需要使用智能指针
        std::unique_ptr<Block::LinearSolverType> linearSolver(
                    new g2o::LinearSolverDense<Block::PoseMatrixType>());
        //根据线性方程求解器定义矩阵块求解器
        std::unique_ptr<Block>
                matSolver(new Block(std::move(linearSolver)));
        //选取优化过程中使用的梯度下降算法,类似于ceres里面的loss function的那个函数的选择
        g2o::OptimizationAlgorithmLevenberg* gradientSolver=
                new g2o::OptimizationAlgorithmLevenberg(std::move(matSolver));
        //指定优化问题的求解器
        optimizer.setAlgorithm(gradientSolver);
        //打开调试输出信息
        optimizer.setVerbose(true);
}
