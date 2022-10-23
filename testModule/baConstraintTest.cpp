#include<iostream>
#include"baConstraint.h"
#include"BAEngine.h"

//把点投影到每个相机上
//仅仅是为了写测试用例
void projectWorldPoint(WorldPoint& dstPoint,
        std::unordered_map<unsigned,EigenVec6d>& poses,
            EigenVec5d& intrInfo)
{
    //遍历点的每个pose
    for(auto& eachPose : poses)
    {
        //把三维点转换到相机坐标系下
        Eigen::Vector3d camPoint=transWorldToCamera(
                    dstPoint.coord_,eachPose.second);
        //把相机下的点转换到像素下并且记录到观察点里面
        dstPoint.observations_[eachPose.first]=
                intrinsicTranspose(intrInfo,camPoint);
        //临时的print调试，因为debug的时候看不到
        //std::cout<<"obs point: "<<dstPoint.observations_[eachPose.first]<<std::endl;
    }
}

int main()
{
    //新建一个scene
    Scene tempScene;
    tempScene.camIntrinsic_<<100,100,100,100,0;
    //随便写4个外参
    EigenVec6d tempPose;
    tempPose<<0.1,0.2,0.3,2,2,2;
    tempScene.poses_[0]=tempPose;
    tempScene.poses_.at(0)=tempPose;
    tempPose<<-0.1,0.2,0.3,3,4,5;
    tempScene.poses_[1]=tempPose;
    tempPose<<0.5,1,-0.2,3,1,4;
    tempScene.poses_[2]=tempPose;
    tempPose<<0.1,0.2,0.3,1,2,1;
    tempScene.poses_[3]=tempPose;
    //遍历每个pose,依次打印，这是因为debug的时候显示的数据不正常
//    for(auto& eachPose : tempScene.poses_)
//    {
//        std::cout<<eachPose.second<<std::endl;
//    }
    //新建几个点列表
    tempScene.pointList_.push_back(WorldPoint());
    tempScene.pointList_.back().coord_<<0,0,0;
    tempScene.pointList_.push_back(WorldPoint());
    tempScene.pointList_.back().coord_<<-1,-2,-3;
    tempScene.pointList_.push_back(WorldPoint());
    tempScene.pointList_.back().coord_<<-1,0,-1;
    tempScene.pointList_.push_back(WorldPoint());
    tempScene.pointList_.back().coord_<<-2,-1,0;
    tempScene.pointList_.push_back(WorldPoint());
    tempScene.pointList_.back().coord_<<-2,-1,0;
    tempScene.pointList_.push_back(WorldPoint());
    tempScene.pointList_.back().coord_<<-2,-5,0;
    tempScene.pointList_.push_back(WorldPoint());
    tempScene.pointList_.back().coord_<<-3,-5.2,0;
    //把点依次投影到每个相机上
    for(auto& eachPoint : tempScene.pointList_)
    {
        projectWorldPoint(eachPoint,tempScene.poses_,tempScene.camIntrinsic_);
    }
    //微微调整内参
    tempScene.camIntrinsic_<<98,103,101,98,0.5;
    //新建一个BA的工具
    BAEngine baEngine;
    //设置输入的scene
    baEngine.setInputScene(tempScene);
    //开始优化
    baEngine.beginOptimize();
    //输出内参的结果
        std::cout<<"intrinsic: "<<tempScene.camIntrinsic_<<std::endl;
        for(auto& eachPose : tempScene.poses_)
        {
            std::cout<<eachPose.first<<": "<<eachPose.second<<std::endl;
        }
    return 0;
}
