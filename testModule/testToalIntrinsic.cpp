#include<iostream>
#include"totalIntrinsic.h"

int main()
{
    //声明一个全参数的内参
    TotalIntrinsic totalNode;
    EigenVec5d vec5;
    vec5<<1,2,3,4,5;
    totalNode.setEstimate(vec5);
    std::cout<<totalNode.estimate()<<std::endl;
	return 0;
}
