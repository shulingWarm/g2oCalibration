#声明最小版本
cmake_minimum_required(VERSION 3.2)
#添加c++11标准
add_compile_options(-std=c++17)
#工程名称
project(g2oCalibration)
#设置为debug模式
#set(CMAKE_BUILD_TYPE "Debug")
#g2o的include路径
set(g2oInclude "/home/cvlab/usrApp/g2o/include")
include_directories(${g2oInclude})
#元编程里面的一些基础程序操作
include_directories("/media/cvlab/data/workSpace/mainProject/code20220412/patchHeaderFile/src")
#读取g2o的相关库文件
file(GLOB g2oLibs "/home/cvlab/usrApp/g2o/lib/libg2o*.so")
#寻找eigen3的位置
find_package(Eigen3 REQUIRED)
#添加eigen的路径
include_directories(${EIGEN3_INCLUDE_DIRS})
#寻找opencv的位置
find_package(OpenCV REQUIRED)
#把opencv添加到include目录
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIRS})
#库目录同时还需要包含当前目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#求内参矩阵的逆矩阵所用的函数，静态函数不能用这样的方式放在.h里面来实现
add_library(intrinsicInverse intrinsicInverse.cpp)
#原始的张氏标定法对应的估计，似乎直接把函数实现写在.h文件里面它是无法通过编译的
add_library(homographyConstraint homographyConstraint.cpp)
#链接到g2o相关的库
target_link_libraries(homographyConstraint ${g2oLibs} intrinsicInverse)
#把点从世界转换到相机坐标系的库
add_library(extrinsicTranspose extrinsicTranspose.cpp)
#把点从相机坐标系转换到像素坐标
add_library(intrinsicTranspose intrinsicTranspose.cpp)
#BA优化的库
add_library(BAEngine BAEngine.cpp)
target_link_libraries(BAEngine ${g2oLibs} intrinsicTranspose extrinsicTranspose)
#添加opencv的角点读取程序的实现文件
add_library(cvCornerRead cvCornerRead.cpp)
target_link_libraries(cvCornerRead ${OpenCV_LIBS})
#添加可执行文件
add_executable(main main.cpp)
#添加动态链接库
target_link_libraries(main ${g2oLibs})
#添加opencv的链接库
target_link_libraries(main ${OpenCV_LIBS})
#优化器的初始化
add_library(optimizerInit optimizerInit.cpp)
target_link_libraries(optimizerInit ${g2oLibs})
#opencv计算单应性变换的库
add_library(cvFindHomography cvFindHomography.cpp)
target_link_libraries(cvFindHomography ${OpenCV_LIBS})
#标定的准备程序，其实就是对张氏标定法的封装
add_library(calibrateInit calibrateInit.cpp)
target_link_libraries(calibrateInit ${g2oLibs} optimizerInit homographyConstraint)
#标定器的实现
add_library(calibrator calibrator.cpp)
target_link_libraries(calibrator ${g2oLibs} cvFindHomography cvCornerRead BAEngine
        optimizerInit calibrateInit homographyConstraint)
#添加测试模块
add_subdirectory(testModule)

