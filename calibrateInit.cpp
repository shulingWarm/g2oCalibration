#include"calibrateInit.h"
#include"homographyConstraint.h"
#include"optimizerInit.h"

//输入单应性变换的列表，输出用张氏标定法得到的内参
//内参上需要预先给一个主点的初值
void InitCalibrator::solveIntrinsic(const std::vector<Eigen::Matrix3d>& homoMat,
                    Eigen::Vector3d& intrinsics)
{
    //新建一个优化器
    g2o::SparseOptimizer optimizer;
    //约束边的编号
    unsigned id=0;
    initG2oOptimizer(optimizer);
    //新建待优化的内参节点
    IntrinsicVertex* intrVertexPtr=new IntrinsicVertex();
    //设置内参节点的标号
    intrVertexPtr->setId(0);
    //设置内参的初始值
    intrVertexPtr->setEstimate(intrinsics);
    //把节点添加到优化器里面
    optimizer.addVertex(intrVertexPtr);
    //遍历每个单应矩阵，把它们都加到优化器里面
    for(const auto& eachHomo : homoMat)
    {
        //新建一个临时的单应矩阵的约束边
        auto homoConsPtr=new HomographyEdge(eachHomo,intrinsics[1],intrinsics[2]);
        //给约束边设置编号
        homoConsPtr->setId(id);
        ++id;
        //设置边的内参矩阵
        homoConsPtr->setVertex(0,intrVertexPtr);
        //设置边的信息矩阵
        homoConsPtr->setInformation(Eigen::Matrix4d::Identity());
        //把边添加到优化器里面
        optimizer.addEdge(homoConsPtr);
    }
    //准备开始优化
    optimizer.initializeOptimization();
    optimizer.optimize(iterateTims_);
}
