#include<iostream>
#include"calibrator.h"
#include"cvFindHomography.h"
#include"cvCornerReader.h"
#include"scene.h"
#include"BAEngine.h"
#include"optimizerInit.h"
#include"calibrateInit.h"
#include"homographyDecompose.h"
#include"BAEngine.h"

//构造函数
Calibrator::Calibrator(const std::vector<std::string>& imgList,
          const std::array<unsigned,2>& boardSize)
{
    //计算单应矩阵的方法
    setHomographyFinder(std::unique_ptr<CvHomographyFinder>(
                            new CvHomographyFinder()));
    //计算角点的方法
    setCornerReader(std::unique_ptr<CvCornerReader>(
                        new CvCornerReader(boardSize[0],boardSize[1])));
    //记录所有图片的名称列表
    imgList_=imgList;
}

//设置单应矩阵的计算器
void Calibrator::setHomographyFinder(std::unique_ptr<HomographyFinder> homoFinder)
{
    homoFinder_=std::move(homoFinder);
}

//设置图片角点的读取器
void Calibrator::setCornerReader(std::unique_ptr<CornerReadInterface> cornerReader)
{
    cornerReader_=std::move(cornerReader);
}

//用网格单元信息初始化scene里面的世界点
void addGridPointsToScene(unsigned rowNum,unsigned colNum,double gridSize,
            Scene& targetScene //这个scene会被初始化
)
{
    //遍历每个网格点
    for(unsigned idRow=0;idRow<rowNum;++idRow)
    {
        //当前行的行坐标
        double y=idRow*gridSize;
        //遍历当前行的每一列
        for(unsigned idCol=0;idCol<colNum;++idCol)
        {
            //向列表里面添加一个点
            targetScene.pointList_.push_back(WorldPoint());
            //记录坐标
            targetScene.pointList_.back().coord_<<(idCol*gridSize),y,0;
        }
    }
}

//从标定板信息里面初始化scene
void initSceneByBoardCorners(const std::vector<std::vector<std::array<double,2>>>& cornerMap,
            Scene& targetScene, //这个scene会被初始化
            double gridSize //网格单元的大小
)
{
    //清空scene里面的所有内容
    targetScene.clear();
    //根据网格单元信息初始化scene里面的点
    addGridPointsToScene(cornerMap.size(),cornerMap.front().size(),
                            gridSize,targetScene);
}

//给一个scene添加观察图片
//仅适用于标定板的情形
void addObsToScene(const std::vector<std::vector<std::array<double,2>>>& cornerMap,
                   Scene& targetScene)
{
    //保证观察点的数量和scene里面世界点的数量是相同的
    if(targetScene.pointList_.size()!=cornerMap.size()*cornerMap.front().size())
    {
        std::cerr<<"found corner points number invalid"<<std::endl;
        throw 0;
    }
    //即将添加的观察者标号
    unsigned idObs=targetScene.poses_.size();
    //添加一个外参，用于表示当前的图片
    targetScene.poses_[idObs]=EigenVec6d();
    //初始化当前访问的行列
    unsigned currRow=0,currCol=0;
    //遍历所有的点来添加观察信息到scene里面
    for(auto& eachPoint : targetScene.pointList_)
    {
        //记录投影映射
        eachPoint.observations_[idObs]=Eigen::Vector2d();
        //要添加的点
        auto& obsCoord=cornerMap[currRow][currCol];
        eachPoint.observations_[idObs]<<obsCoord[0],obsCoord[1];
        //列增加
        ++currCol;
        if(currCol>=cornerMap[currRow].size())
        {
            ++currRow;
            currCol=0;
        }
    }
}

//从scene里面获取指定相机的对应点列表
//由于是仅针对标定板的情况，所以三维世界点也只记录XY
void getPointMapByView(std::vector<std::array<double,4>>& pointPairs,
                       unsigned idView,const Scene& targetScene)
{
    //遍历点列表里面的每个点
    for(auto& eachPoint : targetScene.pointList_)
    {
        //判断目标view是否存在
        if(eachPoint.observations_.count(idView)==0)
        {
            std::cerr<<idView<<" cannot found in "<<eachPoint.coord_;
            throw -1;
        }
        //当前位置的投影点
        auto& proj=eachPoint.observations_.at(idView);
        //记录当前的点在目标view上的像素位置
        std::array<double,4> worldMatch={eachPoint.coord_[0],eachPoint.coord_[1],
                                      proj[0],proj[1]};
        //把匹配点记录到完整的匹配列表里面
        pointPairs.push_back(worldMatch);
    }
}

//把相机外参数转换到适用于scnen的数据结构里面
void convertExtParams(std::vector<std::array<double,6>>& extArray,
                      std::unordered_map<unsigned,EigenVec6d>& dstExtVec)
{
    //遍历每个pose
    for(unsigned id=0;id<extArray.size();++id)
    {
        //如果没这个数据就算了
        if(dstExtVec.count(id)==0)
        {
            continue;
        }
        //记录向量数据
        auto& eigenExt=dstExtVec[id];
        for(unsigned idDim=0;idDim<extArray[id].size();++idDim)
        {
            eigenExt[idDim]=extArray[id][idDim];
        }
    }
}

//开启标定流程
void Calibrator::calibrate()
{
    //把scene清空
    scene_.clear();
    //如果没有记录图片列表就直接结束
    if(imgList_.size()==0)
    {
        return;
    }
    //拿出来第1个图片提取角点
    cornerReader_->computeCornerPoints(imgList_.front());
    //用提到的角点初始化scene
    initSceneByBoardCorners(cornerReader_->getCornerPoints(),
                            scene_,gridSize_);
    //还是用这个角点往里面添加观察信息
    addObsToScene(cornerReader_->getCornerPoints(),scene_);
    //遍历其它的图片
    for(unsigned idImg=1;idImg<imgList_.size();++idImg)
    {
        //对当前图片提取角点
        cornerReader_->computeCornerPoints(imgList_[idImg]);
        //把提取到的角点添加到scene里面
        addObsToScene(cornerReader_->getCornerPoints(),scene_);
    }
    //用scene里面的点云关系去准备单应性变换的列表
    std::vector<Eigen::Matrix3d> homoList;
    homoList.resize(imgList_.size());
    //遍历所有的变换，准备每个单应性变换矩阵
    for(unsigned idView=0;idView<homoList.size();++idView)
    {
        //当前view到标定板的所有匹配点
        std::vector<std::array<double,4>> viewPairList;
        getPointMapByView(viewPairList,idView,scene_);
        //从匹配点信息里面计算单应性变换
        std::array<double,9> arrayMat;
        homoFinder_->find(viewPairList,arrayMat);
        //依次记录矩阵的9个数字
        for(unsigned idRow=0;idRow<3;++idRow)
        {
            for(unsigned idCol=0;idCol<3;++idCol)
            {
                homoList[idView](idRow,idCol)=arrayMat[idRow*3+idCol];
            }
        }
    }

    //用单应性变换的列表生成内参
    Eigen::Vector3d intrEigen;
    //把主点初始化为图片的中点
    intrEigen<<cornerReader_->imgHeight_/2.0,cornerReader_->imgWidth_/2.0,
            cornerReader_->imgHeight_/2;

    std::cout<<"init intrinsic"<<std::endl;

    //准备调用内参的初始化代码
    InitCalibrator initSolver;
    initSolver.solveIntrinsic(homoList,intrEigen);
    //准备一个优化器给分解外参的时候使用
    g2o::SparseOptimizer optimizer;
    //初始化优化器
    initG2oOptimizer(optimizer);
    //最终的外参存储
    std::vector<std::array<double,6>> tempExtParameters;

    //用已知的内参分解每个外参
    decomposeHomography(homoList,
                        intrEigen,tempExtParameters,optimizer);
    //把列表形式的外参转换到scene里面的map形式
    convertExtParams(tempExtParameters,scene_.poses_);

    //准备进入最终的非线性优化的标定流程
    BAEngine engine;
    engine.setInputScene(scene_);
    //开始非线性优化版本的标定
    engine.beginOptimize();
}

