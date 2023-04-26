#ifndef AA49BA2B_3151_4644_BA98_2AA265969667
#define AA49BA2B_3151_4644_BA98_2AA265969667
#include"common.h"



#include "lbfgs/lbfgs_new.hpp"
#include "scene/scene_base.hpp"
#include "traj_opti/CorridorGen2D.hpp"


using SpMat=Eigen::SparseMatrix<double>;

struct cubicSplineOpt {
 // basic information
 shared_ptr<CorridorGen2D> Corridor_;
 Emx& path0;
 Emx points0;
 Emx points1;
 Emx param1;
 int segs0;
 
 //
 Eigen::ConjugateGradient<SpMat,Eigen::Upper> solver;
 SpMat A,B,C,M,E,F;
 Emx d_D_d_p,d_c_d_p,d_d_d_p;

 // 障碍物势利场参数
 double h1,h2;

 //topp
 Evx s1;
 Evx heading1;
 Emx q1,qv1,qa1;

 cubicSplineOpt(shared_ptr<CorridorGen2D> _corridor,Emx& _path,double _step,double _h1=200,double _h2=-10)
 :Corridor_(_corridor),path0(_path),step(_step),h1(_h1),h2(_h2)
 {

 }

 // generate initial cubic spline points with fixed step
 int generate_inital_cubic_spline_points(dobule s)
 {

    return 0;
 }

 int generate_static_matrices()
 {
    
 }

};


#endif /* AA49BA2B_3151_4644_BA98_2AA265969667 */
