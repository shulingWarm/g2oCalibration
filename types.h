#ifndef _TYPES_H_
#define _TYPES_H_
#include<Eigen/Core>

using EigenVec6d=Eigen::Matrix<double,6,1>;

//作为内参的时候分别对应fx fy px py s
using EigenVec5d=Eigen::Matrix<double,5,1>;

#endif
