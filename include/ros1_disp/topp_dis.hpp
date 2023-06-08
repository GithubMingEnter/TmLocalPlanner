#ifndef TOPP_DIS_01
#define TOPP_DIS_01

#include <fstream>

#include "rviz_dis.hpp"
#include "topp/conic_alm_topp_opt.hpp"

struct conicAlmToppDis {
  conicALMTOPP2& topp2;
  rviz1DisSimp& r1ds;
  conicAlmToppDis(rviz1DisSimp& _r1ds, conicALMTOPP2& _topp2) : topp2(_topp2), r1ds(_r1ds) {}

  int add_topp_trajectory_points(double v_min, double v_max) {
    Vec a, b, c, d;
    topp2.get_result(a, b, c, d);
    Vec v = b.cwiseSqrt();
    std::cout<<"v = "<<v<<std::endl;
    std::ofstream file;
    file.open("/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/data/dtt1_q.csv");
    for(int k = 0; k < (int)topp2.q.rows(); ++k){
      file << topp2.q(k,0)<<" , "<<topp2.q(k,1)<<"\n";
    }
    file.close();
    file.open("/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/data/dtt1_qv.csv");
    for(int k = 0; k < (int)topp2.qv.rows(); ++k){
      file << topp2.qv(k,0)<<" , "<<topp2.qv(k,1)<<"\n";
    }
    file.close();
    file.open("/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/data/dtt1_qa.csv");
    for(int k = 0; k < (int)topp2.qa.rows(); ++k){
      file << topp2.qa(k,0)<<" , "<<topp2.qa(k,1)<<"\n";
    }
    file.close();
    file.open("/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/data/dtt1_s.csv");
    for(int k = 0; k < (int)topp2.s.size(); ++k){
      file << topp2.s(k)<<"\n";
    }
    file.close();
    file.open("/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/data/dtt1_a.csv");
    for(int k = 0; k < (int)a.rows(); ++k){
      for(int j = 0; j < (int)a.cols()-1; ++j){
        file << a(k,j)<<"  ";
      }
      file << a(k,a.cols()-1)<<"\n";
    }
    file.close();
    file.open("/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/data/dtt1_b.csv");
    for(int k = 0; k < (int)b.rows(); ++k){
      for(int j = 0; j < (int)b.cols()-1; ++j){
        file << b(k,j)<<"  ";
      }
      file << b(k,b.cols()-1)<<"\n";
    }
    file.close();
    file.open("/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/data/dtt1_c.csv");
    for(int k = 0; k < (int)c.rows(); ++k){
      for(int j = 0; j < (int)c.cols()-1; ++j){
        file << c(k,j)<<"  ";
      }
      file << c(k,c.cols()-1)<<"\n";
    }
    file.close();
    file.open("/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/data/dtt1_d.csv");
    for(int k = 0; k < (int)d.rows(); ++k){
      for(int j = 0; j < (int)d.cols()-1; ++j){
        file << d(k,j)<<"  ";
      }
      file << d(k,d.cols()-1)<<"\n";
    }
    file.close();
    
    return r1ds.add_colored_path_strip2d(r1ds.id0++, topp2.q, v, v_min, v_max, 0.02, 0.1);
  }
};

#endif  // TOPP_DIS_01