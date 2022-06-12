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
#添加测试模块
add_subdirectory(testModule)
#添加可执行文件
add_executable(main main.cpp)
#添加动态链接库
target_link_libraries(main ${g2oLibs})
#添加opencv的链接库
target_link_libraries(main ${OpenCV_LIBS})
