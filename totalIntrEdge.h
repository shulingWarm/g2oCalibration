#ifndef _TOTAL_INTR_EDGE_
#define _TOTAL_INTR_EDGE_
#include"totalIntrinsic.h"
#include<g2o/core/base_unary_edge.h>

//一个约束边，这个约束边会试图约束内参节点里面的所有变量
class TotalIntrEdge : public g2o::BaseUnaryEdge<5,double,TotalIntrinsic>
{
public:

    //计算误差的函数，直接强制处理
    void computeError()
    {
        //获得内参节点
        const TotalIntrinsic* intrVertex=(const TotalIntrinsic*)(_vertices[0]);
        for(unsigned idError=0;idError<5;++idError)
        {
            _error(idError)=intrVertex->estimate()(idError)-2*idError;
        }
    }

    //优化边的读写相关函数，留空
    virtual bool read(std::istream &is){return true;}
    virtual bool write(std::ostream &os) const{return true;}
};

#endif
