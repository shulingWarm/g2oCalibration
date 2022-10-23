#ifndef _CV_CORNER_READER_H_
#define _CV_CORNER_READER_H_
#include"cornerReadInterface.h"

//利用opencv实现的角点读取接口
class CvCornerReader : public CornerReadInterface
{
protected:
    //标定板的行数和列数，行数对应短边
    unsigned rowNum_=0;
    unsigned colNum_=0;
public:
    //体现核心功能的函数，输入一个图片，函数运行完成后结果会更新在属性里面
    //注意在这里面只能传入cv::Mat
    virtual void computeCornerPoints(void* imgData) override;

    //通过传入图片路径来处理的获得角点的方案
    virtual void computeCornerPoints(const std::string& imgPath) override;

    //构造函数，opencv的实现接口要求输入标定板的行数和列数
    //默认colNum是长边，rowNum是短边
    CvCornerReader(unsigned rowNum,unsigned colNum);
};


#endif
