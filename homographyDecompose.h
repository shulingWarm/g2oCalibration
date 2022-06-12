#ifndef _HOMOGRAPHY_DECOMPOSE_H_
#define _HOMOGRAPHY_DECOMPOSE_H_
#include<Eigen/Core>
#include<vector>
#include<array>
#include<list>
#include"homographyConstraint.h"
#include"initExtrinsic.h"

//把单应性变换分解成内参和旋转平移
void decomposeHomography(
        const std::vector<Eigen::Matrix3d>& homographyVec, //单应性变换的向量
        Eigen::Vector3d& dstIntrInfo,//算出来的内参会存储在这里面，传入的时候后两个位置需要写上主点坐标
        std::vector<std::array<double,6>>& extrinsicVec, //最后算出来的外参序列都会存储在这里
        g2o::SparseOptimizer& optimizer,//已经写入过算法的优化器
        unsigned iterateTime=1000 //优化次数
)
{
    //新建内参节点
    IntrinsicVertex* intrVertex=new IntrinsicVertex();
    //把内参的焦距设置成两个主点位置的平均值
    dstIntrInfo(0)=(dstIntrInfo(1)+dstIntrInfo(2))/2;
    //设置内参节点的估计值
    intrVertex->setEstimate(dstIntrInfo);
    //设置节点的标号
    intrVertex->setId(0);
    //给优化器加入节点
    optimizer.addVertex(intrVertex);
    //用于存储指针的容器
    std::list<HomographyEdge*> edgePtrList;
    //每个单应性变换形成一个内参约束
    for(unsigned idHomo=0;idHomo<homographyVec.size();++idHomo)
    {
        //新建单应性变换的约束边
        edgePtrList.push_back(new HomographyEdge(homographyVec[idHomo],
                                dstIntrInfo(1),dstIntrInfo(2)));
        //设置约束边的编号
        edgePtrList.back()->setId(idHomo);
        //设置节点
        edgePtrList.back()->setVertex(0,intrVertex);
        //设置信息矩阵
        edgePtrList.back()->setInformation(Eigen::Matrix4d::Identity());
        //给优化器加入当前的约束边
        optimizer.addEdge(edgePtrList.back());
    }
    //初始化优化器
    optimizer.initializeOptimization();
    //开始优化
    optimizer.optimize(iterateTime);
    //释放所有的边的内存
    for(auto eachPtr : edgePtrList)
    {
        delete eachPtr;
    }
    //记录内参结果
    dstIntrInfo=intrVertex->estimate();
    //记录外参的结果
    extrinsicVec.reserve(homographyVec.size());
    for(unsigned idHomo=0;idHomo<homographyVec.size();++idHomo)
    {
        extrinsicVec.push_back(getExtrinsic(
                                   homographyVec[idHomo],dstIntrInfo));
    }
    //释放内参节点
    delete intrVertex;
}


#endif
