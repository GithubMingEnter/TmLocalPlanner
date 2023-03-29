#ifndef CDCBD068_AC38_4D7C_B984_4404F3A1332C
#define CDCBD068_AC38_4D7C_B984_4404F3A1332C
#include<eigen3/Eigen/Core>
#include<map>
#include<memory>
#include"common.h"

class sceneBase{
public:
    //TODO 2d artifital distance field 
    virtual double dist_field(const double x1,const double y1,Vec* grad=nullptr)=0;
    virtual ~sceneBase() {}




};






#endif /* CDCBD068_AC38_4D7C_B984_4404F3A1332C */
