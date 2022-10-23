#include<iostream>
#include"cornerReadInterface.h"
#include"cvCornerReader.h"
#include<opencv2/opencv.hpp>
#include<string>

int main()
{
    //标定板的图片
    std::string imgPath="/media/cvlab/data/privateSpace/workSpace/22cvChessBoardCornerBoardDetect/chessBoard.jpeg";
    //新建角点读取器
    CvCornerReader reader(11,8);
    //读取角点
    reader.computeCornerPoints(imgPath);
    //读取图片，把读取到的点画上去
    cv::Mat tempImg=cv::imread(imgPath);
    //遍历第3行的每个角点
    for(const auto& eachPoint : reader.getCornerPoints()[3])
    {
        //在角点对应的位置上画圈
        cv::circle(tempImg,cv::Point2i((int)eachPoint[0],(int)eachPoint[1]),3,cv::Scalar(0,0,255));
        //打印画圈的坐标
        std::cout<<eachPoint[0]<<" "<<eachPoint[1]<<std::endl;
    }
    //保存画好的图片
    cv::imwrite("circle.jpg",tempImg);
	return 0;
}
