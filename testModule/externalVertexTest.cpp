#include<iostream>
#include"externalVertex.h"

int main()
{
    //新建一个外参节点
    ExternalVertex externalVtx;
    //设置外参的初值
    EigenVec6d vec6;
    vec6<<1,2,3,4,5,6;
    externalVtx.setEstimate(vec6);
    //输出外参节点的估计值
    std::cout<<externalVtx.estimate()<<std::endl;
	return 0;
}
