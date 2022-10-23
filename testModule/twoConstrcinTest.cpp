#include<iostream>
#include"IntrinsicNode.h"
#include<g2o/core/base_binary_edge.h>
#include<cmath>

//同时约束两个节点的边
//double表示的是观测值_measurement的数据类型
class TwoLinkEdge : public g2o::BaseBinaryEdge<2,double,
        IntrinsicVertex,IntrinsicVertex>
{
    //6个用来约束的数值，分别对应两个内参
    std::array<double,6> excepValue_;

    //从指定的位置开始计算三个数字的误差
    double getSingleError(Eigen::Vector3d estimate,
                          unsigned cmpOffset)
    {
        //初始化求和结果
        double sum=0;
        for(unsigned idValue=0;idValue<3;++idValue)
        {
            sum+=std::abs(estimate[idValue]-excepValue_[idValue+cmpOffset]);
        }
        //返回误差值
        return sum;
    }
public:
    TwoLinkEdge()
    {
        //给6个内参数值做初始化
        excepValue_={1,2,3,4,5,6};
    }

    //计算误差的函数
    void computeError()
    {
        //获取第1个内参节点
        const IntrinsicVertex* intrPtr1=(const IntrinsicVertex*)(_vertices[0]);
        //再获取第2个内参节点
        const IntrinsicVertex* intrPtr2=(const IntrinsicVertex*)(_vertices[1]);
        //计算误差值
        _error(0)=getSingleError(intrPtr1->estimate(),0);
        _error(1)=getSingleError(intrPtr2->estimate(),3);
        _measurement;
    }

    //优化边的读写相关函数，留空
    virtual bool read(std::istream &is){return true;}
    virtual bool write(std::ostream &os) const{return true;}
};

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
    //声明一个有两条约束的边
    TwoLinkEdge testEdge;
    //设置当前边的标号
    testEdge.setId(0);
    //初始化两个用于约束的内参节点
    IntrinsicVertex vertex1,vertex2;
    //设置节点的标号
    vertex1.setId(0);
    vertex2.setId(1);
    //设置第1个连接点
    testEdge.setVertex(0,&vertex1);
    //设置第2个连接点
    testEdge.setVertex(1,&vertex2);
    //观测值直接放在自定义的属性里面了，所以没有用到
    //信息矩阵表达的可能是不同维度之间的误差的关联程度
    testEdge.setInformation(Eigen::Matrix2d::Identity());
    //新建一个优化器
    g2o::SparseOptimizer optimizer;
    //初始化优化器
    initOptimizer(optimizer);
    //添加待优化的节点
    optimizer.addVertex(&vertex1);
    optimizer.addVertex(&vertex2);
    //添加约束边
    optimizer.addEdge(&testEdge);
    //初始化优化过程
    optimizer.initializeOptimization();
    //开始优化
    optimizer.optimize(10);
    //获取两个内参的估计结果
    std::cout<<vertex1.estimate()<<std::endl;
    std::cout<<vertex2.estimate()<<std::endl;
	return 0;
}
