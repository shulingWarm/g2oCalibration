#ifndef _CALIBRATOR_H_
#define _CALIBRATOR_H_
#include<vector>
#include<string>
#include"findHomography.h"
#include"cornerReadInterface.h"
#include"types.h"
#include<memory>
#include"scene.h"

//相机标定器
class Calibrator
{
protected:
    //寻找单应性变换使用的方法
    std::unique_ptr<HomographyFinder> homoFinder_;

    //提取标定板角点的方法
    std::unique_ptr<CornerReadInterface> cornerReader_;

    //所有图片的名称列表
    std::vector<std::string> imgList_;
public:
    //标定板上的角点间距
    //如果有需求可以在启动calibrate之前修改
    double gridSize_=1.0;

    //最后里面的内参外参信息可以从这里面找
    Scene scene_;

    //传入所有图片的名称列表
    Calibrator(const std::vector<std::string>& imgList,
              const std::array<unsigned,2>& boardSize);

    //设置单应矩阵的计算器
    void setHomographyFinder(std::unique_ptr<HomographyFinder> homoFinder);

    //设置图片角点的读取器
    void setCornerReader(std::unique_ptr<CornerReadInterface> cornerReader);

    //开启标定流程
    void calibrate();
};

#endif
