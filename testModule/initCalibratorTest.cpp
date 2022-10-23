#include<iostream>
#include"calibrateInit.h"
#include"types.h"
#include<Eigen/Geometry>
#include"getHMatByAxis.h"

int main()
{
    std::vector<Eigen::Matrix3d> matList;
    EigenVec6d ext;
    ext<<0.2,2,3,1,2,3;
    matList.push_back(getHMayByExt(ext));
    ext<<3,-2,3,1,0,3;
    matList.push_back(getHMayByExt(ext));
    ext<<1,1,3,-3,0,3;
    matList.push_back(getHMayByExt(ext));
    //做一个初始化的内参
    Eigen::Vector3d intr;
    intr<<1,0,0;
    //新建一个标定初始化求解器
    InitCalibrator calibrator;
    //求解内参
    calibrator.solveIntrinsic(matList,intr);
    std::cout<<intr<<std::endl;
	return 0;
}
