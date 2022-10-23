#ifndef _CALIBRATE_INIT_H_
#define _CALIBRATE_INIT_H_

#include"types.h"
#include<vector>

//标定过程的初始化
//主要负责通过单应矩阵的列表求相机内参
class InitCalibrator
{
public:
    //使用张氏标定法求解单应矩阵
    void solveIntrinsic(const std::vector<Eigen::Matrix3d>& homoMat,
                        Eigen::Vector3d& intrinsics);

    //迭代的次数，默认是50
    unsigned iterateTims_=50;
};

#endif
