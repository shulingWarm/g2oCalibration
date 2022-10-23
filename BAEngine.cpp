#include"totalIntrinsic.h"
#include"externalVertex.h"
#include"scene.h"
#include<vector>
#include<memory>
#include"baConstraint.h"
#include"BAEngine.h"

//输入输入的scene
void BAEngine::setInputScene(Scene& inputScene)
{
    corresScenePtr_=&inputScene;
}

//获取场景的scene 如果发现scene是空指针就抛出错误
Scene& BAEngine::getScene()
{
    //如果scene是空的要报错
    if(corresScenePtr_==nullptr)
    {
        std::cerr<<"the scene of BAEngine has not been initialized"<<std::endl;
        throw -1;
    }
    //正常情况下返回引用
    return *corresScenePtr_;
}

//根据所有的节点和边开始优化
void BAEngine::beginOptimize(ExtrinsicMap& extMap,ExtEdgeMap& extEdgeList,
                   TotalIntrinsic* intrPtr)
{
    //新建优化器
    g2o::SparseOptimizer optimizer;
    BAInitOptimizer(optimizer);
    //添加内参节点
    optimizer.addVertex(intrPtr);
    //添加每个外参节点
    for(auto& eachExt : extMap)
    {
        optimizer.addVertex(eachExt.second);
    }
    //添加约束边
    for(auto& eachEdge : extEdgeList)
    {
        optimizer.addEdge(eachEdge);
    }
    //初始化优化器
    optimizer.initializeOptimization();
    //开始优化
    optimizer.optimize(iterTimes_);
}

//初始化优化器
void BAEngine::BAInitOptimizer(g2o::SparseOptimizer& optimizer)
{
    //指定待优化的图的类型
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>> Block;
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

//获得内参和外参之间的约束边
void BAEngine::getExtrinsicEdge(ExtrinsicMap& extrList,TotalIntrinsic* intrPtr,
                      ExtEdgeMap& dstEdgeMap)
{
    //遍历场景里面的每个点
    for(auto& eachPoint : getScene().pointList_)
    {
        //遍历每个观察点
        for(auto& eachObs : eachPoint.observations_)
        {
            //新建一个约束边
            dstEdgeMap.push_back(
                        new BAConstraint(eachObs.second,eachPoint.coord_));
            //设置约束边的标号
            dstEdgeMap.back()->setId(dstEdgeMap.size());
            //设置约束边的两个节点
            dstEdgeMap.back()->setVertex(0,intrPtr);
            dstEdgeMap.back()->setVertex(1,extrList[eachObs.first]);
            //设置边的信息矩阵
            dstEdgeMap.back()->setInformation(Eigen::Matrix2d::Identity());
        }
    }
}

//根据scene获得一系列的外参节点
//索引的第一个键值表示对应的相机标号
void BAEngine::getExtrinsicList(ExtrinsicMap& dstExtrList)
{
    //遍历外参的数值
    for(auto& eachExt : getScene().poses_)
    {
        //新建一个外参
        dstExtrList[eachExt.first]=new ExternalVertex();
        //更新外参的初值
        dstExtrList[eachExt.first]->setEstimate(eachExt.second);
        //更新外参的id
        dstExtrList[eachExt.first]->setId(dstExtrList.size());
    }
}

//根据scene获得待优化的内参节点
TotalIntrinsic* BAEngine::getIntrPtr()
{
    //新建一个内参节点
    TotalIntrinsic* intrPtr(new TotalIntrinsic());
    //把节点的估计值设置成scene里面的内参值
    intrPtr->setEstimate(getScene().camIntrinsic_);
    //设置节点的标号，固定是0
    intrPtr->setId(0);
    //返回新建的内参节点
    return intrPtr;
}

//开始运行
void BAEngine::beginOptimize()
{
    //获得待优化的内参节点
    auto intrPtr=getIntrPtr();
    //获得外参节点
    ExtrinsicMap extrMap;
    getExtrinsicList(extrMap);
    //根据内参和外参获得约束边
    ExtEdgeMap edgeList;
    getExtrinsicEdge(extrMap,intrPtr,edgeList);
    //根据节点和边开始优化
    beginOptimize(extrMap,edgeList,intrPtr);
    //优化完成后把优化结果重新记录到原始的scene里面
    saveVertexToScene(intrPtr,extrMap);
}

//把优化的结果重新记录到scene里面
void BAEngine::saveVertexToScene(TotalIntrinsic* intrPtr,
                       ExtrinsicMap& extrList)
{
    //记录内参
    getScene().camIntrinsic_=intrPtr->estimate();
    //依次记录外参
    for(auto& eachExtr : extrList)
    {
        getScene().poses_[eachExtr.first]=eachExtr.second->estimate();
    }
}
