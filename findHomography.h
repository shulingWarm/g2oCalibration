#ifndef _FIND_HOMOGRAPHY_H_
#define _FIND_HOMOGRAPHY_H_
#include<array>
#include<vector>

class HomographyFinder
{
public:
    //输入点列表，根据点的映射列表生成单应性变换的结果
    //pointPair里面每4个数字的前两个表示原始的位置，后两个值表示变换之后的位置点
    virtual void find(const std::vector<std::array<double,4>>& pointPairs,
                      std::array<double,9>& homoMat_)=0;
};

#endif
