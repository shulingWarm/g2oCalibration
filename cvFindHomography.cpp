#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include"cvFindHomography.h"

//输入点列表，根据点的映射列表生成单应性变换的结果
//pointPair里面每4个数字的前两个表示原始的位置，后两个值表示变换之后的位置点
void CvHomographyFinder::find(const std::vector<std::array<double,4>>& pointPairs,
                              std::array<double,9>& homoMat_)
{
    if(pointPairs.size()<4)
    {
        std::cout<<"the point pairs should be 4 at least"<<std::endl;
        return;
    }
    //把点对整理成opencv要求的形式
    std::vector<cv::Point2f> srcPoints;
    auto dstPoints=srcPoints;
    srcPoints.reserve(pointPairs.size());
    dstPoints.reserve(pointPairs.size());
    //遍历每个点对
    for(const auto& eachPoint : pointPairs)
    {
        srcPoints.push_back(cv::Point2f(eachPoint[0],eachPoint[1]));
        dstPoints.push_back(cv::Point2f(eachPoint[2],eachPoint[3]));
    }
    //调用单应性变换的计算
    cv::Mat hMat=cv::findHomography(srcPoints,dstPoints);
    //把计算结果保存到homo矩阵里面
    for(unsigned idData=0;idData<9;++idData)
    {
        homoMat_[idData]=((double*)hMat.data)[idData];
    }
}
