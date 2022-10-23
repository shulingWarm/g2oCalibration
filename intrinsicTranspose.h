#ifndef _INTRINSIC_TRANSPOSE_H_
#define _INTRINSIC_TRANSPOSE_H_
#include"types.h"

//把5参数的内参数转换成内参矩阵
Eigen::Matrix3d getIntrMat(const EigenVec5d& intrinsics);

//输入一个点和内参，把相机坐标系下的点转换到像素坐标系下
//不会判断传入的是不是无穷远点
Eigen::Vector2d intrinsicTranspose(const EigenVec5d& intrinsics,
                        Eigen::Vector3d& cameraPoint);

#endif
