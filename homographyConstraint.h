#ifndef _HOMOGRAPHY_CONSTRAINT_H_
#define _HOMOGRAPHY_CONSTRAINT_H_
#include"IntrinsicNode.h"
#include"cameraPoint.h"
#include<g2o/core/base_binary_edge.h>
#include<array>
#include"intrinsicInverse.h"

//单应性变换对内参的约束形成的约束边
//这个类会固定相机的主点，通常只是用于初始化
class HomographyEdge : public g2o::BaseUnaryEdge<4,double,IntrinsicVertex>
{
    //两个单应性矩阵的列向量 h1 h2
    std::array<Eigen::Vector3d,2> homoColumn_;
    //两个主点坐标xy
    Eigen::Vector2d priPoint_;

    //根据内参计算单应矩阵形成的等长度误差
    double getEqualDistanceError(const Eigen::Vector3d& intrInfo);
    //根据内参计算正交性误差
    double angleError(const Eigen::Vector3d& intrInfo);
    //根据一个h向量和内参计算相乘后的结果向量
    Eigen::Vector3d getCompareVector(const Eigen::Vector3d& hColumn,
                                     const Eigen::Vector3d& intrInfo);
public:
    //构造函数，传入一个单应性矩阵和内参的两个主点坐标
    HomographyEdge(const Eigen::Matrix3d& homography,
                   double priX,double priY);

    //计算误差的函数，一个单应性矩阵对内参形成的两个约束
    //4个误差项依次是正交约束，长度约束，主点x误差和主点y误差
    void computeError();

    //优化边的读写相关函数，留空
    virtual bool read(std::istream &is){return true;}
    virtual bool write(std::ostream &os) const{return true;}
};

#endif
