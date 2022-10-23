#include<iostream>
#include<algorithm>
#include<opencv2/opencv.hpp>
#include<vector>
#include"cvCornerReader.h"

//构造函数，opencv的实现接口要求输入标定板的行数和列数
//默认colNum是长边，rowNum是短边
CvCornerReader::CvCornerReader(unsigned rowNum,unsigned colNum)
{
    rowNum_=std::max(rowNum,colNum);
    colNum_=std::min(rowNum,colNum);
}

//体现核心功能的函数，输入一个图片，函数运行完成后结果会更新在属性里面
void CvCornerReader::computeCornerPoints(void* imgData)
{
    //opencv的矩阵
    cv::Mat& img=*((cv::Mat*)imgData);
    //记录图片的长和宽
    imgWidth_=img.cols;
    imgHeight_=img.rows;
    //把行列数保存成opencv的形式
    cv::Size boardSize(colNum_,rowNum_);
    //提前准备一个用于存储结果的角点列表
    std::vector<cv::Point2f> cornerList;
    //调用opencv的函数寻找角点
    int findFlag= cv::findChessboardCorners(img,boardSize,cornerList);
    //如果并不是所有的角点都被找到了，抛出异常
    if(findFlag==0)
    {
        std::cerr<<"not all corner points are found"<<std::endl;
        throw 0;
    }
    //根据行数开辟空间
    cornerPointMap_.resize(rowNum_);
    //把找到的角点保存在数据结构里面
    for(unsigned idRow=0;idRow<rowNum_;++idRow)
    {
        //当前的访问偏移量
        unsigned idOffset=idRow*colNum_;
        //要访问的行内容
        auto& currRow=cornerPointMap_[idRow];
        //初始化当前行
        currRow.resize(colNum_);
        //遍历当前行的每一列
        for(unsigned idCol=0;idCol<colNum_;++idCol)
        {
            //当前位置的点
            auto& currPoint=cornerList[idOffset+idCol];
            currRow[idCol]={currPoint.x,currPoint.y};
        }
    }
}

//通过传入图片路径来处理的获得角点的方案
void CvCornerReader::computeCornerPoints(const std::string& imgPath)
{
    //读取图片
    cv::Mat cvImg=cv::imread(imgPath);
    //如果读取失败要报错的
    if(cvImg.empty())
    {
        std::cerr<<"the image "<<imgPath<<" cannot be read"<<std::endl;
        throw 0;
    }
    //调用计算角点的程序
    computeCornerPoints(&cvImg);
}
