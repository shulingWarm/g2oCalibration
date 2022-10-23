#include<iostream>
#include"IntrinsicNode.h"

int main()
{
    //声明一个内参节点
    IntrinsicVertex intrVertex;
    //设置用来估计的值
    Eigen::Vector3d testValue(9,10,11);
    //设置内参节点的初始值
    intrVertex.setEstimate(testValue);
    //输出节点的估计结果
    std::cout<<intrVertex.estimate()<<std::endl;
	return 0;
}
