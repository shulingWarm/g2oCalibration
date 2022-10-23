#ifndef _BA_ENGINE_H_
#define _BA_ENGINE_H_
#include"totalIntrinsic.h"
#include"externalVertex.h"
#include"scene.h"
#include<vector>
#include<memory>
#include"baConstraint.h"

//专门用来做标定过程中全局优化的引擎的
class BAEngine
{
protected:
    //外参的map
    using ExtrinsicMap=std::map<unsigned,ExternalVertex*>;
    //外参和内参约束边的map
    using ExtEdgeMap=std::list<BAConstraint*>;

    //优化时候的迭代次数
    unsigned iterTimes_=1000;

    //需要被优化的scene的指针
    Scene* corresScenePtr_=nullptr;

    //获取场景的scene 如果发现scene是空指针就抛出错误
    Scene& getScene();

    //根据scene获得待优化的内参节点
    TotalIntrinsic* getIntrPtr();

    //根据scene获得一系列的外参节点
    void getExtrinsicList(ExtrinsicMap& dstExtrList);

    //获得内参和外参之间的约束边
    void getExtrinsicEdge(ExtrinsicMap& extrList,TotalIntrinsic* intrPtr,
                          ExtEdgeMap& dstEdgeMap);

    //根据所有的节点和边开始优化
    void beginOptimize(ExtrinsicMap& extMap,ExtEdgeMap& extEdgeList,
                       TotalIntrinsic* intrPtr);

    //初始化优化器
    void BAInitOptimizer(g2o::SparseOptimizer& optimizer);

    //把优化的结果重新记录到scene里面
    void saveVertexToScene(TotalIntrinsic* intrPtr,
                           ExtrinsicMap& extrList);

public:
    //输入输入的scene
    void setInputScene(Scene& inputScene);

    //开始运行
    void beginOptimize();

    //设置迭代次数
    void setItertime(unsigned iterTime)
    {
        iterTimes_=iterTime;
    }
};

#endif
