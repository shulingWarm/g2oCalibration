#include<iostream>
#include"homographyConstraint.h"


//构造函数，传入一个单应性矩阵和内参的两个主点坐标
HomographyEdge::HomographyEdge(const Eigen::Matrix3d& homography,
               double priX,double priY)
{
    //记录两个列矩阵
    homoColumn_[0]=homography.col(0);
    homoColumn_[1]=homography.col(1);
    //记录主点坐标
    priPoint_[0]=priX;
    priPoint_[1]=priY;
}

//根据一个h向量和内参计算相乘后的结果向量
Eigen::Vector3d HomographyEdge::getCompareVector(const Eigen::Vector3d& hColumn,
                                 const Eigen::Vector3d& intrInfo)
{
    //把内参信息整理成k的逆矩阵
    Eigen::Matrix3d invKMat=getIntrinsicInverse(intrInfo[0],intrInfo[1],intrInfo[2]);
    //返回矩阵和向量的相乘结果
    return invKMat*hColumn;
}


//根据内参计算正交性误差
double HomographyEdge::angleError(const Eigen::Vector3d& intrInfo)
{
    //计算待比较的两个向量
    Eigen::Vector3d cmpVector1=getCompareVector(homoColumn_[0],intrInfo);
    Eigen::Vector3d cmpVector2=getCompareVector(homoColumn_[1],intrInfo);
    //两个向量应当是正交的
    return std::abs(cmpVector1.transpose()*cmpVector2)/cmpVector1.norm()/cmpVector2.norm();
}

//根据内参计算单应矩阵形成的等长度误差
double HomographyEdge::getEqualDistanceError(const Eigen::Vector3d& intrInfo)
{
    //计算待比较的两个向量
    Eigen::Vector3d cmpVector1=getCompareVector(homoColumn_[0],intrInfo);
    Eigen::Vector3d cmpVector2=getCompareVector(homoColumn_[1],intrInfo);
    //两个向量的长度应该相等
    return std::abs(cmpVector1.norm()-cmpVector2.norm());
}

//计算误差的函数，一个单应性矩阵对内参形成的两个约束
//4个误差项依次是正交约束，长度约束，主点x误差和主点y误差
void HomographyEdge::computeError()
{
    //获取内参节点
    const IntrinsicVertex* intrVertex=(const IntrinsicVertex*)(_vertices[0]);
    //分别计算4个误差
    _error(0)=angleError(intrVertex->estimate());
    _error(1)=getEqualDistanceError(intrVertex->estimate());
    _error(2)=std::abs(intrVertex->estimate()[1]-priPoint_[0]);
    _error(3)=std::abs(intrVertex->estimate()[2]-priPoint_[1]);
}
