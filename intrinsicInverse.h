#ifndef _INTRINSIC_INVERSE_H_
#define _INTRINSIC_INVERSE_H_
#include<Eigen/Core>

//获取内参矩阵的逆矩阵，传入的分别是焦距和两个主点坐标
Eigen::Matrix3d getIntrinsicInverse(double focal,double priX,double priY)
{
    //新建逆内参矩阵
    Eigen::Matrix3d invKMat;
    invKMat<<1.f/focal,0,-priX/focal,
            0,1.f/focal,-priY/focal,
            0,0,1;
    return invKMat;
}

#endif
