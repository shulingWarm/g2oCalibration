# g2oCalibration
该项目是基于g2o实现的相机标定

该项目的输入要求是同一相机拍摄的若干张棋盘格的图片
程序先通过张氏标定法计算相机内参和外参的初值，然后使用重投影误差精确估计相机的内参
