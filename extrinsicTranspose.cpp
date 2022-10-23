
#include<Eigen/Core>
#include"types.h"
#include<cmath>
#include<Eigen/Geometry>
#include"extrinsicTranspose.h"

//一个函数，根据外参和输入的点把点从世界坐标转换到相机坐标系下
Eigen::Vector3d transWorldToCamera(const Eigen::Vector3d& worldPoint,
                   const EigenVec6d& extrinsics)
{
    //把前三个数字拿出来转换成旋转轴
    Eigen::Vector3d rotAxis;
    rotAxis<<extrinsics(0),extrinsics(1),extrinsics(2);
    //把旋转轴转换成旋转矩阵
    float rotAngle=rotAxis.norm();
    rotAngle=fmod(rotAngle,M_PI);
    //对旋转轴单位化
    rotAxis.normalize();
    //生成旋转轴
    Eigen::AngleAxisd eigenAxis(rotAngle,rotAxis);
    //生成对应的旋转矩阵
    Eigen::Matrix3d rotMat=eigenAxis.matrix();
    //平移量
    Eigen::Vector3d transVector;
    transVector<<extrinsics(3),extrinsics(4),extrinsics(5);
    //对世界点先做旋转再做平移
    return rotMat*worldPoint+transVector;
}
