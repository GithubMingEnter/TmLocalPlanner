#ifndef CC35913B_70A0_4399_AA07_153080BF4E8D
#define CC35913B_70A0_4399_AA07_153080BF4E8D
#ifndef SPLINE_OPT_HPP
#define SPLINE_OPT_HPP
#include <eigen3/Eigen/Sparse>
#include <iomanip>
#include <iostream>

#include <eigen3/Eigen/Sparse>
#include <iomanip>
#include <iostream>

#include "lbfgs/lbfgs_new.hpp"
#include "scene/scene_base.hpp"
#include "traj_opti/CorridorGen2D.hpp"

typedef Eigen::SparseMatrix<double> SpMat;

struct cubicSplineOpt {
  // basic information
  shared_ptr<CorridorGen2D> Corridor_;
  Mat& path0;
  double step;
  Mat points0;
  Mat points1;
  Mat param1;
  int segs0;

  // variables for spline generation
  Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Upper> solver;
  SpMat A, B, C, M, E, F;
  Mat d_D_d_p, d_c_d_p, d_d_d_p;

  // parameters for risk field
  double c0, c1;

  // variables for TOPP
  Vec s1;
  Mat q1, qv1, qa1;

  cubicSplineOpt(shared_ptr<CorridorGen2D> _scene, Mat& _path, double _step, double _c0 = 200, double _c1 = -10)
      : Corridor_(_scene), path0(_path), step(_step), c0(_c0), c1(_c1) {
    generate_inital_cubic_spline_points(step);
    generate_static_matrices();//原有形式三次样条
  }

  // generate initial cubic spline points with fixed step
  int generate_inital_cubic_spline_points(double s) {
    double len0 = 0;
    for (int k = 1; k < path0.rows(); ++k) {
      len0 += (path0.row(k) - path0.row(k - 1)).norm();//A*路径长度
      
    }
    int num_est = static_cast<int>((std::floor(len0 / s) + 1) * 1.5);//1。5？

    int num = 0;
    Mat points02(num_est, 2);
    points02.row(num++) = path0.row(0);
    double residual = 0;
    for (int k = 1; k < path0.rows(); ++k) {
      double offset = -residual;
      double edge = (path0.row(k) - path0.row(k - 1)).norm();
      residual += edge;
      while (residual >= s) {//如果大于步长
        double location = (offset + step) / edge;
        //？
        points02.row(num++) = (1 - location) * path0.row(k - 1) + location * path0.row(k);
        residual -= step;
        offset += step;
      }
    }
    if (residual < 0.67 * s) {//余数小于
      points02.row(num - 1) = path0.row(path0.rows() - 1);
    } else {
      points02.row(num++) = path0.row(path0.rows() - 1);
    }
    points0 = points02.block(0, 0, num, 2);
    segs0 = num - 1;
    return 0;
  }

  // generate static matrices for initial cubic spline points
  int generate_static_matrices() {
    A = SpMat(segs0 - 1, segs0 - 1);//SparseMatrix
    B = SpMat(segs0 - 1, segs0 - 1);
    for (int i = 0; i < segs0 - 2; ++i) {
      A.insert(i, i) = 4;
      A.insert(i, i + 1) = 1;
      A.insert(i + 1, i) = 1;
    }
    A.insert(segs0 - 2, segs0 - 2) = 4;
    for (int i = 0; i < segs0 - 2; ++i) {
      B.insert(i + 1, i) = -3;
      B.insert(i, i + 1) = 3;
    }
    C = SpMat(segs0, segs0 - 1);
    M = SpMat(segs0, segs0 - 1);
    E = SpMat(segs0, segs0 - 1);
    F = SpMat(segs0, segs0 - 1);
    for (int i = 0; i < segs0 - 1; ++i) {
      C.insert(i, i) = 3;
      C.insert(i + 1, i) = -3;
    }
    for (int i = 0; i < segs0 - 1; ++i) {
      M.insert(i, i) = -1;
      M.insert(i + 1, i) = -2;
    }
    for (int i = 0; i < segs0 - 1; ++i) {
      E.insert(i, i) = -2;
      E.insert(i + 1, i) = 2;
    }
    for (int i = 0; i < segs0 - 1; ++i) {
      F.insert(i, i) = 1;
      F.insert(i + 1, i) = 1;
    }
    solver.analyzePattern(A);//重新排序矩阵的非零元素，以便分解步骤创建更少的填充
    solver.factorize(A);//计算系数矩阵的因子/?。 每次矩阵的值发生变化时
    d_D_d_p = solver.solve(B);//$ A x = b \f$ D
    d_c_d_p = C + M * d_D_d_p;
    d_d_d_p = E + F * d_D_d_p;
    return 0;
  }

  // convert path points to cubic spline coefficients
  int path_to_coeff(const Mat& path, Vec& coeff) {
    int num = path.rows();
    coeff.resize(num * 2 - 4);
    for (int i = 1; i < num - 1; ++i) {
      coeff(i - 1) = path(i, 0);//路径点
      coeff(i + num - 3) = path(i, 1);
    }
    return 0;
  }

  // convert cubic spline coefficients to  path points
  
  int coeff_to_path(const Vec& coeff, const Mat& path0, Mat& path) {
    int num = path0.rows();
    path.resize(num, 2);
    path.row(0) = path0.row(0);
    path.row(num - 1) = path0.row(num - 1);
    for (int i = 1; i < num - 1; ++i) {
      path(i, 0) = coeff(i - 1);//1,2,3,4,5,6
      path(i, 1) = coeff(i + num - 3);
    }
    return 0;
  }

  // convert path points to cubic spline parameters
  int path_to_param(const Mat& path, Mat& param) {
    int segs = path.rows() - 1;
    Vec Dx(Vec::Zero(segs + 1));
    Vec Dy(Vec::Zero(segs + 1));
    param.resize(segs, 8);
    //可参考三次样条的构造
    Dx.segment(1, segs - 1) = 3 * solver.solve(path.block(2, 0, segs - 1, 1) -
                                               path.block(0, 0, segs - 1, 1));
    Dy.segment(1, segs - 1) = 3 * solver.solve(path.block(2, 1, segs - 1, 1) -
                                               path.block(0, 1, segs - 1, 1));
    for (int i = 0; i < segs; ++i) {
      param(i, 0) = path(i, 0);
      param(i, 1) = Dx(i);
      param(i, 2) = 3 * (path(i + 1, 0) - path(i, 0)) - 2 * Dx(i) - Dx(i + 1);
      param(i, 3) = 2 * (path(i, 0) - path(i + 1, 0)) + Dx(i) + Dx(i + 1);
      param(i, 4) = path(i, 1);
      param(i, 5) = Dy(i);
      param(i, 6) = 3 * (path(i + 1, 1) - path(i, 1)) - 2 * Dy(i) - Dy(i + 1);
      param(i, 7) = 2 * (path(i, 1) - path(i + 1, 1)) + Dy(i) + Dy(i + 1);
    }
    return 0;
  }

  // convert cubic spline parameters to interpolated points
  int param_to_interpolated(const Mat& param, Mat interp, int segs) {
    int end = segs * param.rows();
    interp.resize(end + 1, 2);
    for (int i = 0; i < param.rows(); ++i) {
      for (int j = 0; j < segs; ++j) {
        double t = j * 1.0 / segs;
        interp(j + i * segs, 0) = param(i, 0) + param(i, 1) * t +
                                  param(i, 2) * t * t + param(i, 3) * t * t * t;
        interp(j + i * segs, 1) = param(i, 4) + param(i, 5) * t +
                                  param(i, 6) * t * t + param(i, 7) * t * t * t;
      }
      interp(end, 0) = param(end, 0) + param(end, 1) + param(end, 2) + param(end, 3);
      interp(end, 1) = param(end, 4) + param(end, 5) + param(end, 6) + param(end, 7);
    }
    return 0;
  }

  
  int param_to_q(const Mat& param, Vec& s, Mat& q, Mat& qv, Mat& qa, int segs) {
    int num = param.rows();
    int pts = num * segs + 1;
    auto cubic_x = [&param](int i, double t) {
      return param(i, 0) + param(i, 1) * t + param(i, 2) * t * t +
             param(i, 3) * t * t * t;//三次样条表达式关于一维x
    };
    auto cubic_y = [&](int i, double t) {
      return param(i, 4) + param(i, 5) * t + param(i, 6) * t * t +
             param(i, 7) * t * t * t;//二维y
    };
    auto dx_dt = [&](int i, double t) {
      return param(i, 1) + 2 * param(i, 2) * t + 3 * param(i, 3) * t * t;
    };
    auto dy_dt = [&](int i, double t) {
      return param(i, 5) + 2 * param(i, 6) * t + 3 * param(i, 7) * t * t;
    };
    auto ds_dt = [&](int i, double t) {
      return std::sqrt(std::pow(dx_dt(i, t), 2) + std::pow(dy_dt(i, t), 2));
      //
    };
    auto d2x_dt2 = [&](int i, double t) {
      return 2 * param(i, 2) + 6 * param(i, 3) * t;//求导
    };
    auto d2y_dt2 = [&](int i, double t) {
      return 2 * param(i, 6) + 6 * param(i, 7) * t;
    };
    s.resize(pts);
    q.resize(pts, 2);
    qv.resize(pts, 2);
    qa.resize(pts, 2);
    double dt = 1.0 / segs;
    double current_length = 0;
    for (int i = 0; i < num; ++i) {
      for (int j = 0; j < segs; ++j) {
        double t = j * dt;
        s(j + i * segs) = current_length;
        q(j + i * segs, 0) = cubic_x(i, t);
        q(j + i * segs, 1) = cubic_y(i, t);
        // ROS_INFO("t= %f",t);
        // average by numeric integration in Simpson's rule
        double dx_dt0 =
            (dx_dt(i, t) + 4 * dx_dt(i, t + 0.5 * dt) + dx_dt(i, t + dt)) / 6;
        double dy_dt0 =
            (dy_dt(i, t) + 4 * dy_dt(i, t + 0.5 * dt) + dy_dt(i, t + dt)) / 6;
        double ds_dt0 =
            (ds_dt(i, t) + 4 * ds_dt(i, t + 0.5 * dt) + ds_dt(i, t + dt)) / 6;
        current_length += dt * ds_dt0;//路径长度于当前
        double d2x_dt20 =
            (d2x_dt2(i, t) + 4 * d2x_dt2(i, t + 0.5 * dt) + d2x_dt2(i, t + dt)) /
            6;
        double d2y_dt20 =
            (d2y_dt2(i, t) + 4 * d2y_dt2(i, t + 0.5 * dt) + d2y_dt2(i, t + dt)) /
            6;
        qv(j + i * segs, 0) = dx_dt0 / ds_dt0;
        qv(j + i * segs, 1) = dy_dt0 / ds_dt0;
        qa(j + i * segs, 0) = (d2x_dt20 * dy_dt0 - d2y_dt20 * dx_dt0) * dy_dt0 /
                              std::pow(ds_dt0, 4);
        qa(j + i * segs, 1) = (d2y_dt20 * dx_dt0 - d2x_dt20 * dy_dt0) * dx_dt0 /
                              std::pow(ds_dt0, 4);
      }
    }
    s(pts - 1) = current_length;
    qv(pts - 1, 0) = qv(pts - 2, 0);//末端
    qv(pts - 1, 1) = qv(pts - 2, 1);
    qa(pts - 1, 0) = qa(pts - 2, 0);
    qa(pts - 1, 1) = qa(pts - 2, 1);
    return 0;
  }

  // cost function for optimization methods
  static double cost_function(void* ptr, const Vec& x, Vec& g) {
    cubicSplineOpt* opt = reinterpret_cast<cubicSplineOpt*>(ptr);//强制转换三次样条
    int segs = opt->segs0;//初始化段数
    double cost_energy;
    double cost_potential;
    Vec g_energy(x.size());
    Vec g_potential(x.size());
    Mat path, param;
    //coeff👈的是三次样条所对应的路径点的x,y大小
    opt->coeff_to_path(x, opt->points0, path);

    opt->path_to_param(path, param);
    Vec cx = param.col(2);
    Vec dx = param.col(3);
    Vec cy = param.col(6);
    Vec dy = param.col(7);

    // scretch energy goal
    cost_energy = 12 * dx.dot(dx) + 12 * cx.dot(dx) + 4 * cx.dot(cx) +
                  12 * dy.dot(dy) + 12 * cy.dot(dy) + 4 * cy.dot(cy);

    Vec d_energy_d_x = (12 * dx + 8 * cx).transpose() * opt->d_c_d_p +
                       (24 * dx + 12 * cx).transpose() * opt->d_d_d_p;
    Vec d_energy_d_y = (12 * dy + 8 * cy).transpose() * opt->d_c_d_p +
                       (24 * dy + 12 * cy).transpose() * opt->d_d_d_p;
    g_energy.head(segs - 1) = d_energy_d_x;
    g_energy.tail(segs - 1) = d_energy_d_y;

    // distance risk goal
    cost_potential = 0;
    Vec dist_grad(2);
    for (int i = 1; i < segs; ++i) {
      double x1 = path(i, 0);
      double y1 = path(i, 1);
      double dist = opt->Corridor_->dist_field(x1, y1, &dist_grad);
      std::cout<<"dist= "<<dist<<endl;
      // f(x) = c0*exp(c1*x);
      if (dist < 1e-4) {
        cost_potential += 0;
        g_potential(i - 1) = g_potential(i + segs - 2) = 0;
      } else {
        cost_potential += opt->c0 * std::exp(opt->c1 * dist);
        double gain = opt->c1 * opt->c0 * std::exp(opt->c1 * dist);
        g_potential(i - 1) = gain * dist_grad(0);
        g_potential(i + segs - 2) = gain * dist_grad(1);
      }
    }

    g = g_energy + g_potential;
    // std::cout << "{{{{ step2: "
    //           << "  line0:" << path.row(0) << "  \nline1:" << path.row(1) << std::endl;
    return cost_energy + cost_potential;
  }

  // information monitoring in L-BFGS algorithm
  static int monitor_progress(void* /*instance*/, const Eigen::VectorXd& /*x*/,
                              const Eigen::VectorXd& g, const double fx,
                              const double /*step*/, const int k,
                              const int /*ls*/) {
    // std::cout << "================================\n"
    //           << "Iteration: " << k << "\n"
    //           << "Cost: " << fx << "\n"
    //           << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << std::endl;
    return 0;
  }

  // optimizing the spline by L-BFGS algorithm
  int path_opt_lbfgs() {
    double final_cost;
    Vec coeff;
    path_to_coeff(points0, coeff);
    int ret = 0;
    lbfgs::lbfgs_parameter_t lbfgs_param;
    lbfgs_param.s_curv_coeff = 0.7;
    lbfgs_param.f_dec_coeff = 1e-4;
    lbfgs_param.g_epsilon = 1.0e-6;
    lbfgs_param.past = 3;
    lbfgs_param.delta = 1.0e-6;

    std::cout << "{{{{ step1: " << points0.rows() << " " << points0.cols() << " " << coeff.size() << std::endl;
    //输出A*搜索到点的集合
    ret = lbfgs::lbfgs_optimize(coeff, final_cost, cost_function,
                                monitor_progress, this,
                                lbfgs_param);

    coeff_to_path(coeff, points0, points1);
    path_to_param(points1, param1);
  //note
    // std::cout << std::setprecision(6) << "================================"
    //           << "\n"
    //           << "L-BFGS Optimization Returned: " << ret << "\n"
    //           << "Minimized Cost: " << final_cost << std::endl;

    // std::cout << param1 << std::endl;

    return ret;
  }

  // topp preparations
  int topp_prepare(int segs) {
    return param_to_q(param1, s1, q1, qv1, qa1, segs);
  }
};
  





#endif  // SPLINE_OPT_HPP


#endif /* CC35913B_70A0_4399_AA07_153080BF4E8D */
