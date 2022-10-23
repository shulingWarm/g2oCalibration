#include"types.h"
#include"intrinsicTranspose.h"

//把5参数的内参数转换成内参矩阵
Eigen::Matrix3d getIntrMat(const EigenVec5d& intrinsics)
{
    Eigen::Matrix3d intrMat;
    intrMat<<intrinsics(0),intrinsics(4),intrinsics(2),
            0,intrinsics(1),intrinsics(3),
            0,0,1;
    //返回转换过的内参矩阵
    return intrMat;
}

//输入一个点和内参，把相机坐标系下的点转换到像素坐标系下
//不会判断传入的是不是无穷远点
Eigen::Vector2d intrinsicTranspose(const EigenVec5d& intrinsics,
                        Eigen::Vector3d& cameraPoint)
{
    //把内参转换成内参矩阵
    Eigen::Matrix3d intrMat=getIntrMat(intrinsics);
    //把相机坐标系下的点转换到像素坐标系下
    Eigen::Vector3d pixHomo=intrMat*cameraPoint;
    //把齐次坐标转换到二维坐标
    Eigen::Vector2d pixelLocal;
    pixelLocal<<pixHomo(0)/pixHomo(2),pixHomo(1)/pixHomo(2);
    //返回二维坐标
    return pixelLocal;
}
