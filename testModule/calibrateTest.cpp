#include<iostream>
#include"cvCornerReader.h"
#include"calibrator.h"
#include"calibrateInit.h"
#include<string>


int main()
{
    //图片的根目录
    std::string imgRoot="/media/cvlab/data/privateSpace/workSpace/22pcCalibrateBoard";
    //图片的个数
    unsigned imgNum=28;
    //图片格点的数量
    unsigned boardColNum=8;
    unsigned boardRowNum=6;
    //图片路径的列表
    std::vector<std::string> imgPathList;
    imgPathList.resize(imgNum);
    //记录每个图片的名称
    for(unsigned idImg=0;idImg<imgNum;++idImg)
    {
        imgPathList[idImg]=imgRoot+"/"+std::to_string(idImg+1)+".jpg";
    }
    std::swap(imgPathList.front(),imgPathList.back());
    //新建标定器
    //记录标定器的长宽
    std::array<unsigned,2> boardSize={boardRowNum,boardColNum};
    Calibrator calibrator(imgPathList,boardSize);
    calibrator.calibrate();
    //打印运行结果的内参
    std::cout<<calibrator.scene_.camIntrinsic_<<std::endl;
	return 0;
}
