#ifndef _SCENE_H_
#define _SCENE_H_
#include"worldPoint.h"
#include<list>

//一个用于三维重建的场景
class  Scene
{
public:
    //世界点的列表
    std::list<WorldPoint> pointList_;
    //相机的外参的哈希表
    //第一个键值表示相机的标号
    std::unordered_map<unsigned,EigenVec6d> poses_;
    //相机的内参 因为只考虑标定问题，因此每个场景里面只有一个相机
    EigenVec5d camIntrinsic_;

    //清空scene里面的内容
    void clear()
    {
        pointList_.clear();
        poses_.clear();
    }

    //后续如果想扩展相机的畸变模型可以再添加新的属性

};

#endif
