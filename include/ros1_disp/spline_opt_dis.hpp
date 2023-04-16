#ifndef SPLINE_OPT_DIS_01
#define SPLINE_OPT_DIS_01

#include "rviz_dis.hpp"
#include "topp/spline_opt.hpp"
#include <fstream>
struct splineOptDis{
  cubicSplineOpt& cso;
  rviz1DisSimp& r1ds;
  splineOptDis(rviz1DisSimp&_r1ds, cubicSplineOpt& _cso) : 
                 cso(_cso), r1ds(_r1ds){}

  int add_lbfgs_path_points(){
    std::fstream file;
    file.open("/home/ming/ros1_workspace/testws/src/topp2/data/dtt1_yaw.csv");
    for(int i=0;i<cso.heading1.rows();i++){
      file<<cso.heading1(i)<<"\n";
    }
    file.close();

    return r1ds.add_scattered2d(r1ds.id0++, cso.points1, 0.08, 0.0, {0.0,1.0,1.0,1.0});
  }

  int add_lbfgs_path(){
    return r1ds.add_path_strip2d(r1ds.id0++, cso.points1, 0.02, 0.1, {1.0,1.0,0.5,1.0});
  }
};

#endif // SPLINE_OPT_DIS_01
