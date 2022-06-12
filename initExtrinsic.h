#ifndef _INIT_EXTRINSIC_H_
#define _INIT_EXTRINSIC_H_
#include<array>
#include<Eigen/Core>
#include"intrinsicInverse.h"
#include<Eigen/Geometry>

//根据已知的单应性变换和内参矩阵初始化外参
//返回结果的前3个是旋转向量，后3个是平移量
std::array<double,6> getExtrinsic(const Eigen::Matrix3d& homographyMat,
                                  const Eigen::Vector3d& intrInfo //内参信息，焦距和两个主点
)
{
    //内参矩阵的逆矩阵
    Eigen::Matrix3d inverseKMat=getIntrinsicInverse(intrInfo[0],intrInfo[1],intrInfo[2]);
    //计算第1个旋转矩阵
    Eigen::Vector3d rotColumn1=inverseKMat*homographyMat.col(0);
    //计算第2个列向量
    Eigen::Vector3d rotColumn2=inverseKMat*homographyMat.col(1);
    //旋转向量被缩放的幅度，这会作为后面平移向量缩放幅度的参考
    double lambdaScale=rotColumn1.norm();
    //用两个列向量推出第3个列向量
    Eigen::Vector3d rotColumn3=rotColumn1.cross(rotColumn2);
    //根据三个列向量新建一个旋转矩阵
    Eigen::Matrix3d rotMat;
    rotMat.col(0)=rotColumn1.normalized();
    rotMat.col(1)=rotColumn2.normalized();
    rotMat.col(2)=rotColumn3.normalized();
    //把旋转矩阵转换成旋转向量
    Eigen::AngleAxisd rotVector(rotMat);
    //把旋转向量存储到数组里面
    Eigen::Vector3d axis=rotVector.axis();
    axis*=rotVector.angle();
    std::array<double,6> extrinsicInfo;
    for(unsigned dimCount=0;dimCount<3;++dimCount)
    {
        extrinsicInfo[dimCount]=axis[dimCount];
    }
    //计算平移向量
    Eigen::Vector3d transVector=inverseKMat*homographyMat.col(2)/lambdaScale;
    //用平移向量给外参信息赋值
    for(unsigned dimCount=0;dimCount<3;++dimCount)
    {
        extrinsicInfo[dimCount+3]=transVector[dimCount];
    }
    return extrinsicInfo;
}

#endif
