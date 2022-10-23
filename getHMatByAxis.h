#ifndef _GET_H_MAT_BY_EXT_
#define _GET_H_MAT_BY_EXT_
#include"types.h"

//通过外参信息获得H矩阵
Eigen::Matrix3d getHMayByExt(const EigenVec6d& extPar)
{
    Eigen::Vector3d angleaxis;
    angleaxis<<extPar[0],extPar[1],extPar[2];
    double angle=angleaxis.norm();
    angleaxis/=angle;
    angle=std::fmod(angle,M_PI);
    //构造一个旋转角
    Eigen::AngleAxisd eigenAxis(angle,angleaxis);
    //生成对应的旋转矩阵
    Eigen::Matrix3d rotMat=eigenAxis.matrix();
    return rotMat;
}


#endif
