#include<iostream>
#include"totalIntrEdge.h"
#include<memory>

//初始化优化器
void initOptimizer(g2o::SparseOptimizer& optimizer)
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

int main()
{
    //新建被优化的内参
    auto intrPtr=std::shared_ptr<TotalIntrinsic>(new  TotalIntrinsic());
    //设置内参节点的标号
    intrPtr->setId(0);
    EigenVec5d intrInitValue;
    intrInitValue<<1,2,3,4,5;
    //给内参节点赋值初值
    intrPtr->setEstimate(intrInitValue);
    //设置用于优化这个节点的边
    auto intrEdge=std::shared_ptr<TotalIntrEdge>(new TotalIntrEdge());
    //设置边的标号
    intrEdge->setId(0);
    //设置边的节点
    intrEdge->setVertex(0,intrPtr.get());
    intrEdge->setInformation(Eigen::Matrix<double,5,5>::Identity());
    //新建优化器
    g2o::SparseOptimizer optimizer;
    //给优化器添加节点 一定要先添加节点
    optimizer.addVertex(intrPtr.get());
    //给优化器添加边
    optimizer.addEdge(intrEdge.get());
    //初始化优化器
    initOptimizer(optimizer);
    optimizer.initializeOptimization();
    //开始优化
    optimizer.optimize(100);
    //输出内参节点的值
    std::cout<<intrPtr->estimate()<<std::endl;
	return 0;
}
