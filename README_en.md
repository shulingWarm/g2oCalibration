#g2oCalibration

The project is used for camera calibration. It is implemented based on g2o.
The calibration process can be considered as a nonlinear optimization problem about the intrinsic parameters of the camera. The initial value of the variables in the optimization problem is obtained by the method from Z. Zhange in this link:
https://ieeexplore.ieee.org/iel5/34/19223/00888718.pdf?casa_token=5E2aeDpPJQIAAAAA:g8PXnnF8f5sfX0SA2eW9Zy9HJcN90jRNX4hY0qRvLF8JZCSfe6gUKxFcFGekbhrOlhVWmGHloQ

The constraint function is the bundle adjustment.