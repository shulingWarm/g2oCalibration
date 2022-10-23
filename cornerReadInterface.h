#ifndef _CORNER_READ_INTERFACE_H_
#define _CORNER_READ_INTERFACE_H_
#include<vector>
#include<array>
#include<string>

//从图片里面读取角点的基本接口，不对任何类型的数据做具体化
class CornerReadInterface
{
protected:
    //标定板上的点列表，按照行顺序存储
    std::vector<std::vector<std::array<double,2>>> cornerPointMap_;
public:

    //相机的长和宽
    unsigned imgWidth_=0;
    unsigned imgHeight_=0;

    //获得标定板的点列表
    const std::vector<std::vector<std::array<double,2>>> & getCornerPoints() const
    {
        return cornerPointMap_;
    }

    //获取标定板上点的行数
    unsigned getRowNum() const
    {
        return cornerPointMap_.size();
    }

    //通过传入图片路径来处理的获得角点的方案
    virtual void computeCornerPoints(const std::string& imgPath)=0;

    //体现核心功能的函数，输入一个图片，函数运行完成后结果会更新在属性里面
    virtual void computeCornerPoints(void* imgData)=0;
};

#endif
