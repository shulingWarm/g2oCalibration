#include<iostream>
#include"homographyConstraint.h"

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
    //随便生成一个单应矩阵
    Eigen::Matrix3d homoTest;
    //给单应矩阵随便写一个初值
    homoTest<<1,1,2,
            1,0,4,
            3,1,0;
    //用单应矩阵构建一个单应矩阵形成的约束
    HomographyEdge* homoConstraint=new HomographyEdge(homoTest,100,100);
    //设置边的标号
    homoConstraint->setId(0);
    //设置边的信息矩阵
    homoConstraint->setInformation(Eigen::Matrix4d::Identity());
    //新建一个优化器
    g2o::SparseOptimizer optimizer;
    //初始化优化器
    initOptimizer(optimizer);
    //新建待优化的内参节点
    IntrinsicVertex* intrVertexPtr=new IntrinsicVertex();
    //设置内参节点的标号
    intrVertexPtr->setId(0);
    //设置内参的初始值
    Eigen::Vector3d intrInitValue;
    intrInitValue<<100,100,100;
    intrVertexPtr->setEstimate(intrInitValue);
    //给目标边设置内参节点
    homoConstraint->setVertex(0,intrVertexPtr);
    //给优化器添加点
    optimizer.addVertex(intrVertexPtr);
    //给优化器添加边
    optimizer.addEdge(homoConstraint);
    //准备开始优化
    optimizer.initializeOptimization();
    optimizer.optimize(50);
    //输出内参的优化结果
    std::cout<<intrVertexPtr->estimate()<<std::endl;
	return 0;
}
