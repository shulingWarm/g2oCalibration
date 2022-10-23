#ifndef _EXTRINSIC_TRANSPOSE_H_
#define _EXTRINSIC_TRANSPOSE_H_
#include<Eigen/Core>
#include"types.h"
#include<cmath>

//一个函数，根据外参和输入的点把点从世界坐标转换到相机坐标系下
Eigen::Vector3d transWorldToCamera(const Eigen::Vector3d& worldPoint,
                   const EigenVec6d& extrinsics);

#endif
