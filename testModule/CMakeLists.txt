#添加内参节点的测试程序
add_executable(testIntrinsic testIntrinsic.cpp)
#链接g2o的库
target_link_libraries(testIntrinsic ${g2oLibs})
#添加外参节点的测试程序
add_executable(externalVertexTest externalVertexTest.cpp)
target_link_libraries(externalVertexTest ${g2oLibs})
#一个边和两个约束节点相关联的测试
add_executable(twoLinkConstrin twoConstrcinTest.cpp)
target_link_libraries(twoLinkConstrin ${g2oLibs})
#点在相机坐标系下的坐标
#一个边和两个约束节点相关联的测试
add_executable(camPointTest camPointTest.cpp)
target_link_libraries(camPointTest ${g2oLibs})
#相机坐标系下的点和内参的约束关系
add_executable(IntrCampointEdgeTest IntrCampointEdgeTest.cpp)
target_link_libraries(IntrCampointEdgeTest ${g2oLibs})
#由单应性变换形成的对内参的约束
add_executable(homographyConstraintTest homographyConstraintTest.cpp)
target_link_libraries(homographyConstraintTest ${g2oLibs})
#测试已知内参的情况下初始化外参
add_executable(initExtrinsicTest initExtrinsicTest.cpp)
target_link_libraries(initExtrinsicTest ${g2oLibs})
#单应性变换分解出内参和旋转平移
add_executable(homographyDecomposeTest homographyDecomposeTest.cpp)
target_link_libraries(homographyDecomposeTest ${g2oLibs})
#完整参数的内参节点的测试
add_executable(testToalIntrinsic testToalIntrinsic.cpp)
target_link_libraries(testToalIntrinsic ${g2oLibs})
#重投影误差约束的测试
add_executable(baConstraintTest baConstraintTest.cpp)
target_link_libraries(baConstraintTest ${g2oLibs} extrinsicTranspose intrinsicTranspose BAEngine)
#链接ba engine的编译测试
add_executable(baEngineCompileTest baEngineCompileTest.cpp)
target_link_libraries(baEngineCompileTest BAEngine)
#用于测试完全参数的内参参与优化时的情形
add_executable(totalIntrEdgeTest totalIntrEdgeTest.cpp)
target_link_libraries(totalIntrEdgeTest ${g2oLibs})
#5参数的内参测不明白，现在开始测3参数的内参
add_executable(simpleIntrEdgeTest simpleIntrEdgeTest.cpp)
target_link_libraries(simpleIntrEdgeTest ${g2oLibs})
#从图片里面读取角点程序的测试
add_executable(cornerReadTest cornerReadTest.cpp)
target_link_libraries(cornerReadTest cvCornerRead ${OpenCV_LIBS})
#完整的标定过程的测试
add_executable(calibrateTest calibrateTest.cpp)
target_link_libraries(calibrateTest cvCornerRead BAEngine calibrator)
#计算单应性变换矩阵的算法测试
add_executable(findHomographyTest findHomographyTest.cpp)
target_link_libraries(findHomographyTest cvFindHomography)
#内参初始化器的测试代码
add_executable(initCalibratorTest initCalibratorTest.cpp)
target_link_libraries(initCalibratorTest calibrateInit)
