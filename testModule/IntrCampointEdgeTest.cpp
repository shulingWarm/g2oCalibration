#include<iostream>
#include"IntrCamerapointEdge.h"

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

//测试优化过程
//这里面把所有变量都开在栈上了，导致double free
void testOptimize()
{
    //新建一个优化器
    g2o::SparseOptimizer optimizer;
    initOptimizer(optimizer);
    //待优化的内参节点
    IntrinsicVertex intrVertex;
    //设置内参节点的标号
    intrVertex.setId(0);
    //设置约束边
    Eigen::Vector2d projPoint;
    projPoint<<2,2;
    IntrCampointEdge edge(projPoint);
    //相机坐标系下的节点
    projPoint<<4,4;
    CameraPointVertex camPoint(projPoint);
    //设置节点标号
    camPoint.setId(1);
    //设置边的标号
    edge.setId(0);
    //给边设置内参节点
    edge.setVertex(0,&intrVertex);
    //给边设置相机坐标系下的坐标点
    edge.setVertex(1,&camPoint);
    //设置边上两个误差维度的关联关系
    edge.setInformation(Eigen::Matrix2d::Identity());
    //把相机坐标系下的点设置成定值
    camPoint.setFixed(true);
    //给优化器添加两个节点
    optimizer.addVertex(&intrVertex);
    optimizer.addVertex(&camPoint);
    //添加约束边
    optimizer.addEdge(&edge);
    //添加另一个相机坐标系下的点
    projPoint<<10,9.7;//略微加了一些误差
    CameraPointVertex camPoint2(projPoint);
    //设置节点的标号
    camPoint2.setId(2);
    //新建这个相机坐标系下的点对应的像素坐标系的投影
    projPoint<<5,5.2;
    IntrCampointEdge edge2(projPoint);
    //设置边的标号
    edge2.setId(1);
    //设置内参节点
    edge2.setVertex(0,&intrVertex);
    //设置相机坐标系下的点
    edge2.setVertex(1,&camPoint2);
    //设置第2个边的信息矩阵
    edge2.setInformation(Eigen::Matrix2d::Identity());
    //添加另外一组边和相机坐标系下的点
    optimizer.addVertex(&camPoint2);
    //另一个相机坐标系下的点也设置成固定值
    camPoint2.setFixed(true);
    //添加另一个约束边
    optimizer.addEdge(&edge2);
    //初始化优化过程
    optimizer.initializeOptimization();
    //开始进行优化过程，迭代10次
    optimizer.optimize(10);
    //输出估计结果
    std::cout<<intrVertex.estimate()<<std::endl;
}

//为了防止double free 这个函数会尽量使用指针
//暂时不考虑内存泄漏的问题
void testOptimizePtr()
{
    //新建一个优化器
    g2o::SparseOptimizer optimizer;
    initOptimizer(optimizer);
    //待优化的内参节点
    IntrinsicVertex* intrVertex=new IntrinsicVertex();
    //设置内参节点的标号
    intrVertex->setId(0);
    //设置约束边
    Eigen::Vector2d projPoint;
    projPoint<<2,2;
    IntrCampointEdge* edge=new IntrCampointEdge(projPoint);
    //相机坐标系下的节点
    projPoint<<4,4;
    CameraPointVertex* camPoint=new CameraPointVertex(projPoint);
    //设置节点标号
    camPoint->setId(1);
    //设置边的标号
    edge->setId(0);
    //给边设置内参节点
    edge->setVertex(0,intrVertex);
    //给边设置相机坐标系下的坐标点
    edge->setVertex(1,camPoint);
    //设置边上两个误差维度的关联关系
    edge->setInformation(Eigen::Matrix2d::Identity());
    //把相机坐标系下的点设置成定值
    camPoint->setFixed(true);
    //给优化器添加两个节点
    optimizer.addVertex(intrVertex);
    optimizer.addVertex(camPoint);
    //添加约束边
    optimizer.addEdge(edge);
    //添加另一个相机坐标系下的点
    projPoint<<10,9.7;//略微加了一些误差
    CameraPointVertex* camPoint2=new CameraPointVertex(projPoint);
    //设置节点的标号
    camPoint2->setId(2);
    //新建这个相机坐标系下的点对应的像素坐标系的投影
    projPoint<<5,5.2;
    IntrCampointEdge* edge2=new IntrCampointEdge(projPoint);
    //设置边的标号
    edge2->setId(1);
    //设置内参节点
    edge2->setVertex(0,intrVertex);
    //设置相机坐标系下的点
    edge2->setVertex(1,camPoint2);
    //设置第2个边的信息矩阵
    edge2->setInformation(Eigen::Matrix2d::Identity());
    //添加另外一组边和相机坐标系下的点
    optimizer.addVertex(camPoint2);
    //另一个相机坐标系下的点也设置成固定值
    camPoint2->setFixed(true);
    //添加另一个约束边
    optimizer.addEdge(edge2);
    //初始化优化过程
    optimizer.initializeOptimization();
    //开始进行优化过程，迭代10次
    optimizer.optimize(30);
    //输出估计结果
    std::cout<<intrVertex->estimate()<<std::endl;
}

int main()
{
    testOptimizePtr();
    std::cout<<"do other things"<<std::endl;
	return 0;
}
