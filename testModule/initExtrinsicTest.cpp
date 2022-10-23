#include<iostream>
#include"initExtrinsic.h"
#include<cmath>

int main()
{
    //随便新建一个内参信息
    Eigen::Vector3d intrInfo;
    intrInfo<<100,100,100;
    //假设的外参信息
    Eigen::Vector3d rotAxis;
    rotAxis<<0.267261,
             0.534522,
             0.801784;
    //std::cout<<rotAxis.norm()<<std::endl;
    //std::cout<<rotAxis.normalized()<<std::endl;
    //旋转角
    Eigen::AngleAxisd rotVector(2,rotAxis);
    //新建单应矩阵
    Eigen::Matrix3d homoMat=rotVector.matrix();
    //把第3行转换成旋转
    homoMat.col(2)<<1,2,3;
    //建立内参矩阵
    Eigen::Matrix3d intrMat;
    intrMat<<intrInfo[0],0,intrInfo[1],
            0,intrInfo[0],intrInfo[2],
            0,0,1;
    //把内参矩阵乘到单应矩阵上
    homoMat=intrMat*homoMat;
    //获得外参信息
    std::array<double,6> extrInfo=getExtrinsic(homoMat,intrInfo);
    for(auto eachValue : extrInfo)
    {
        std::cout<<eachValue<<std::endl;
    }
	return 0;
}
