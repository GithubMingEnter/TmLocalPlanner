#ifndef C4D70806_4050_478C_8EF2_51128D3C208E
#define C4D70806_4050_478C_8EF2_51128D3C208E
#include <eigen3/Eigen/Core>

#include "pavel_valtr.hpp"
#include "sdqp/sdqp.hpp"
#include "common.h"


// base classes of obstacles
struct baseObstacle {
  double x;
  double y;
  double r; //diameter or 
  int type; //0 cylinderObstacle 1 convex obstacle
  virtual bool isfree(const double x1, const double y1) = 0;//1 is free
  virtual double distance(const double x1, const double y1, Evx* grad = nullptr) = 0;
  baseObstacle(int _t, double _x, double _y, double _r) : x(_x), y(_y), r(_r), type(_t) {}
  virtual ~baseObstacle() {}
};

// classes for cylinder obstacle
struct cylinderObstacle : public baseObstacle {
  cylinderObstacle(const double _x = 0, const double _y = 0, const double _r = 1) : 
  baseObstacle(0, _x, _y, _r) {}
  ~cylinderObstacle(){};
  bool isfree(const double x1, const double y1) {
    return ((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y) > r * r);
  }
  double distance(const double x1, const double y1, Evx* grad = nullptr) {
    double d1 = std::max(std::sqrt((x1 - x) * (x1 - x) + (y1 - y) * (y1 - y)), 0.01);
    double d = d1 - r;
    if (grad != nullptr) {
      grad->operator()(0) = (x1 - x) / d1;//与中心距离的比值
      grad->operator()(1) = (y1 - y) / d1;
    }
    return d;
  }
};

// classes for poly prism obstacle
struct polyObstacle : public baseObstacle {
  int m;
  Emx poly;
  Eigen::Matrix<double, 2, 2> Q;
  Eigen::Matrix<double, 2, 1> c;
  Eigen::Matrix<double, 2, 1> x;
  Emx A;
  Evx b;
  polyObstacle(const Emx& _poly, const double _x = 0, const double _y = 0, const double _r = 1)
      : baseObstacle(1, _x, _y, _r), m(_poly.rows()),
       poly(_poly), Q(Emx::Identity(2, 2) * 2), A(m, 2), b(m) {
    int j = m - 1;//尾部开始
    for (int k = 0; k < m; ++k) {
      // 边对应起始点 Ax=b
      A(k, 0) = poly(k, 1) - poly(j, 1);
      A(k, 1) = poly(j, 0) - poly(k, 0);
      b(k) = A(k, 0) * poly(j, 0) + A(k, 1) * poly(j, 1);
      j = k;
    }
    // std::cout << "[random poly object OK!]" << std::endl;
  }

  ~polyObstacle(){};
  bool isfree(const double x1, const double y1) {
    x(0) = x1;
    x(1) = y1;
    return (A * x - b).maxCoeff() > 0;
  }
  double distance(const double x1, const double y1, Evx* grad = nullptr) {
    if (isfree(x1, y1)) {
      c(0) = -2 * x1;
      c(1) = -2 * y1;
      sdqp::sdqp<2>(Q, c, A, b, x);//SDQP求解
      double d, d2 = (x1 - x(0)) * (x1 - x(0)) + (y1 - x(1)) * (y1 - x(1));
      if (d2 > 1e-6) {//有效free space 
        d = std::sqrt(d2);
        if (grad != nullptr) {
          grad->operator()(0) = (x1 - x(0)) / d;//vector 指针对应位置梯度
          grad->operator()(1) = (y1 - x(1)) / d;
        }
      } else {
        d = 0;
        if (grad != nullptr) {
          grad->operator()(0) = grad->operator()(1) = 0;
        }
      }
      // std::cout << "[x1:" << x1 << " y1:" << y1 << " x:" << x(0) << " y:" << x(1) << " d:" << d << "]" << std::endl;
      return d;
    } else {
      // double d = 0;
      // if (grad != nullptr) {
      //   grad->operator()(0) = grad->operator()(1) = 0;
      // }

      double d = 1e20;
      // 计算点到各个边的距离，比较
      for (int k = 0; k < m; ++k) {
        double la = A(k, 0);//la*x+lb*y+lc0
        double lb = A(k, 1);
        double lc = -b(k);
        double l2 = la * la + lb * lb;
        double d1 = std::abs(la * x1 + lb * y1 + lc) / std::sqrt(l2);
        if (d1 < d) {
          d = d1;
          if (grad != nullptr) {
            //点到直线的垂足到(x0,y0)对应的(x,y)分量
            double dx = (lb * (lb * x1 - la * y1) - la * lc) / l2 - x1;
            double dy = (la * (la * y1 - lb * x1) - lb * lc) / l2 - y1;
            double norm_dxdy = std::min(std::sqrt(dx * dx + dy * dy), 0.01);
            grad->operator()(0) = dx / norm_dxdy;//与中心距离的比值
            grad->operator()(1) = dy / norm_dxdy;
          }
        }
      }

      return -d;//note 内部为负梯度
    }
  }
};

// class for simple factory type1 (random generation in specific range)
//在特定范围内随机生成
struct baseObstacleFactory {
  virtual baseObstacle* create() = 0;
  virtual baseObstacle* create_const(int obscle_type) =0;
  virtual ~baseObstacleFactory() {}
};

struct randomObstacleFactory1 : public baseObstacleFactory {
  double x0;
  double y0;
  double yaw0;
  randPoly rand_poly;
  std::random_device dev;
  std::mt19937 re;
  std::uniform_real_distribution<double> rand_x;
  std::uniform_real_distribution<double> rand_y;
  std::uniform_real_distribution<double> rand_d;
  std::uniform_int_distribution<int> rand_p;
  randomObstacleFactory1(int _p_min, int _p_max, double _x_min, double _x_max,
                         double _y_min, double _y_max, double _d_min, double _d_max,
                         double _x0, double _y0, double _yaw0)
      : x0(_x0), y0(_y0), yaw0(_yaw0), re(dev()), 
      rand_x(_x_min, _x_max), rand_y(_y_min, _y_max),
      rand_d(_d_min, _d_max), rand_p(_p_min, _p_max) {}  //
  
  
  
  baseObstacle* create() {
    int p1 = rand_p(re);

    double cos0 = std::cos(yaw0);
    double sin0 = std::sin(yaw0);
    double d1 = rand_d(re);//半径
    ROS_INFO("D1= %.f",d1);

    double xr = rand_x(re);
    double yr = rand_y(re);
    double x1 = x0 + xr * cos0 - yr * sin0;
    double y1 = y0 + xr * sin0 + yr * cos0;//旋转矩阵坐标变换

    baseObstacle* bo_ptr;
    if (p1 <= 3) {
      bo_ptr = new cylinderObstacle(x1, y1, d1);
    } else {
      Emx poly1;
      rand_poly.generate(p1, d1, poly1);
      // std::cout << "[random poly OK!]" << std::endl;
      for (int k = 0; k < poly1.rows(); ++k) {
        double dx = poly1(k, 0);
        double dy = poly1(k, 1);
        poly1(k, 0) = x1 + dx * cos0 - dy * sin0;
        poly1(k, 1) = y1 + dx * sin0 + dy * cos0;
      }
      // std::cout << "[random poly formed OK!]" << std::endl;
      bo_ptr = new polyObstacle(poly1, x1, y1, d1);
    }
    return bo_ptr;
  }
  // TODO solid pos
    baseObstacle* create_const(int obscle_type) {
      // 圆柱数量
    int p1 = rand_p(re);

    double cos0 = std::cos(yaw0);
    double sin0 = std::sin(yaw0);
    double d1 = rand_d(re);

    double xr = rand_x(re);
    double yr = rand_y(re);
    double x1 = x0 + xr * cos0 - yr * sin0;
    double y1 = y0 + xr * sin0 + yr * cos0;

    baseObstacle* bo_ptr;
    if (p1 <= 3) {
      bo_ptr = new cylinderObstacle(x1, y1, d1);
    } else {
      Emx poly1;
      rand_poly.generate(p1, d1, poly1);
      // std::cout << "[random poly OK!]" << std::endl;
      for (int k = 0; k < poly1.rows(); ++k) {
        double dx = poly1(k, 0);
        double dy = poly1(k, 1);
        poly1(k, 0) = x1 + dx * cos0 - dy * sin0;
        poly1(k, 1) = y1 + dx * sin0 + dy * cos0;
      }
      // std::cout << "[random poly formed OK!]" << std::endl;
      bo_ptr = new polyObstacle(poly1, x1, y1, d1);
    }
    return bo_ptr;
  }
  ~randomObstacleFactory1() {}
};