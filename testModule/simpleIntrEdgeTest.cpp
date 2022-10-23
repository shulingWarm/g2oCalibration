#include<iostream>
#include"IntrinsicNode.h"
#include<memory>
#include<cmath>

//简单的单参数约束边
class TotalIntrEdge : public g2o::BaseUnaryEdge<3,double,IntrinsicVertex>
{
public:

    //计算误差的函数，直接强制处理
    void computeError()
    {
        //获得内参节点
        const IntrinsicVertex* intrVertex=(const IntrinsicVertex*)(_vertices[0]);
        for(unsigned idError=0;idError<3;++idError)
        {
            _error(idError)=std::abs(intrVertex->estimate()(idError)-2*idError);
        }
    }

    //优化边的读写相关函数，留空
    virtual bool read(std::istream &is){return true;}
    virtual bool write(std::ostream &os) const{return true;}
};

//初始化优化器
void initOptimizer(g2o::SparseOptimizer& optimizer)
{
    //指定待优化的图的类型
        typedef g2o::BlockSolver<g2o::BlockSolverX> Block;
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
    auto intrPtr=new  IntrinsicVertex();
    //设置内参节点的标号
    intrPtr->setId(0);
    Eigen::Vector3d intrInitValue;
    intrInitValue<<1,2,3;
    //给内参节点赋值初值
    intrPtr->setEstimate(intrInitValue);
    //设置用于优化这个节点的边
    auto intrEdge=new TotalIntrEdge();
    //设置边的标号
    intrEdge->setId(0);
    intrEdge->setInformation(Eigen::Matrix<double,3,3>::Identity());
    //新建优化器
    g2o::SparseOptimizer optimizer;
    initOptimizer(optimizer);
    //设置边的节点
    intrEdge->setVertex(0,intrPtr);
    //给优化器添加边
    optimizer.addEdge(intrEdge);
    //给优化器添加节点
    optimizer.addVertex(intrPtr);
    std::cout<<"add end"<<std::endl;
    //初始化优化器
    optimizer.initializeOptimization();
    //开始优化
    optimizer.optimize(100);
    //输出内参节点的值
    std::cout<<intrPtr->estimate()<<std::endl;
    return 0;
}
