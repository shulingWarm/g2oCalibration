#ifndef _INTRINSIC_INVERSE_H_
#define _INTRINSIC_INVERSE_H_
#include<Eigen/Core>

//获取内参矩阵的逆矩阵，传入的分别是焦距和两个主点坐标
Eigen::Matrix3d getIntrinsicInverse(double focal,double priX,double priY);

#endif
