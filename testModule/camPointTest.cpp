#include<iostream>
#include"cameraPoint.h"

int main()
{
    Eigen::Vector2d camPoint;
    camPoint<<100,100;//设置点坐标
    CameraPointVertex cameraVertex(camPoint);//相机坐标系下的节点
    cameraVertex.setId(0);//设置标号
    std::cout<<cameraVertex.estimate()<<std::endl;
	return 0;
}
