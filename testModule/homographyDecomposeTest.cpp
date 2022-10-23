#include<iostream>
#include"homographyDecompose.h"

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
    //新建用于测试的内参
    Eigen::Vector3d testIntr;
    testIntr<<95,50,50;
    //转换成内参矩阵
    Eigen::Matrix3d intrMat;
    intrMat<<testIntr(0),0,testIntr(1),
            0,testIntr(0),testIntr(2),
            0,0,1;
    //新建若干用于测试的外参
    std::vector<std::array<double,6>> testExtrinsics={
        {0.1,0.2,0.3,0,0,0},
        {0.2,0.3,0.4,0,1,2},
        {0.3,0.2,0.1,3,3,3}
    };
    //新建单应性变换矩阵的列表
    std::vector<Eigen::Matrix3d> homographyVec;
    homographyVec.reserve(testExtrinsics.size());
    for(std::array<double,6>& eachExtrinsic : testExtrinsics)
    {
        //旋转向量
        Eigen::Vector3d rotVector;
        rotVector<<eachExtrinsic[0],eachExtrinsic[1],eachExtrinsic[2];
        //生成一个旋转轴
        Eigen::AngleAxisd axis(rotVector.norm(),rotVector.normalized());
        //把旋转轴转换成旋转矩阵
        Eigen::Matrix3d rotMat=axis.matrix();
        //把第3行替换成平移量
        rotMat.col(2)<<eachExtrinsic[3],eachExtrinsic[4],eachExtrinsic[5];
        //记录当前的单应性变换
        homographyVec.push_back(intrMat*rotMat);
    }
    //新建一个用于测试的内参初值
    Eigen::Vector3d testIntrInit=testIntr;
    testIntrInit(0)=testIntrInit(1);
    //优化器
    g2o::SparseOptimizer optimizer;
    initOptimizer(optimizer);
    //对内参和外参做估计
    std::vector<std::array<double,6>> extrinsicEstimate;
    decomposeHomography(homographyVec,
                        testIntrInit,extrinsicEstimate,optimizer);
    //打印估计的结果
    std::cout<<testIntrInit<<std::endl;
    for(auto& eachExtr : extrinsicEstimate)
    {
        for(auto eachValue : eachExtr)
        {
            std::cout<<eachValue<<" ";
        }
        std::cout<<std::endl;
    }
}
