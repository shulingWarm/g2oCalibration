#include<iostream>
#include"findHomography.h"
#include"cvFindHomography.h"

int main()
{
    //新建cv的计算
    CvHomographyFinder cvFinder;
    std::vector<std::array<double,4>> pointPairs={{1,2,2,3},{3,6,4,7},{9,10,11,12},
                                                  {30,12,31,13},{44,2,43,3}};
    //随便写一些特征点
    cvFinder.find(pointPairs);
    //输出单应矩阵
    for(unsigned idData=0;idData<9;++idData)
    {
        std::cout<<cvFinder.homoMat_[idData]<<" ";
        if((idData+1)%3==0)
        {
            std::cout<<std::endl;
        }
    }
	return 0;
}
