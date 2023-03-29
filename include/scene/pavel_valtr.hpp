#ifndef B26B5C4A_CE18_4298_B201_15F2EE043752
#define B26B5C4A_CE18_4298_B201_15F2EE043752

#include "common.h"
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <random>
#include <vector>
using namespace std;

struct randPoly {
  int RND_MAX = 655369;
  std::random_device dev;
  std::mt19937 rng;
  std::uniform_int_distribution<std::mt19937::result_type> random_numer;
  std::uniform_int_distribution<std::mt19937::result_type> random_logic;

  randPoly() : rng(dev()), random_numer(0, RND_MAX), random_logic(0, 1) {}
  virtual ~randPoly() {}

  int generate(const int n, const double r0, Mat& poly) {
    auto gen = [&]() { return random_numer(rng); };
    // initialize random samples and sort them
    // int m = n / 2;
    std::cout<<" r0 ="<<r0<<std::endl;
    std::vector<int> x(n), y(n), vx(n), vy(n), idx(n);
    std::vector<double> a(n);
    std::generate(x.begin(), x.end(), gen);
    std::generate(y.begin(), y.end(), gen);
    // Create a range of sequentially increasing values.
    std::iota(idx.begin(), idx.end(), 0);
    std::sort(x.begin(), x.end());
    std::sort(y.begin(), y.end());
    // divide samples and get vector component
    //3 isolate extreme points 
    int x0 = x[0], x1 = x0;
    //4 Divide into 2chains
    for (int k = 1; k < n - 1; ++k) {
      if (random_logic(rng)) {
        //5 Extract vector components
        vx[k - 1] = x[k] - x0;
        x0 = x[k];
      } else {
        vx[k - 1] = x1 - x[k];
        x1 = x[k];
      }
    }
    vx[n - 2] = x[n - 1] - x0;
    vx[n - 1] = x1 - x[n - 1];
    int y0 = y[0], y1 = y0;
    for (int k = 1; k < n - 1; ++k) {
      if (random_logic(rng)) {
        vy[k - 1] = y[k] - y0;
        y0 = y[k];
      } else {
        vy[k - 1] = y1 - y[k];
        y1 = y[k];
      }
    }
    vy[n - 2] = y[n - 1] - y0;
    vy[n - 1] = y1 - y[n - 1];
    //6 random pair up vector components and sort by angle
    std::shuffle(vy.begin(), vy.end(), rng);
    for (int k = 0; k < n; ++k) {
      a[k] = std::atan2(vy[k], vx[k]);
    }
    //8sort by angle
    std::sort(idx.begin(), idx.end(),
              [&a](int& lhs, int& rhs) { return a[lhs] < a[rhs]; });
    
    double x_max = 0, y_max = 0, x_min = 0, y_min = 0;
    x[0] = y[0] = 0;
    for (int k = 1; k < n; ++k) {//7 form the polygon by connencting vectors
      x[k] = x[k - 1] + vx[idx[k - 1]];
      y[k] = y[k - 1] + vy[idx[k - 1]];
      if (x[k] > x_max) {//9 lay vectors end to end
        x_max = x[k];
      } else if (x[k] < x_min) {
        x_min = x[k];
      }
      if (y[k] > y_max) {
        y_max = y[k];
      } else if (y[k] < y_min) {
        y_min = y[k];
      }
    }
    // center and resize the polygon
    poly.resize(n, 2);
    //10 Move to bounding box
    double x_offset = -(x_max + x_min) / 2.0;
    double y_offset = -(y_max + y_min) / 2.0;
    double scale = 2.0 * r0 / std::max(x_max - x_min, y_max - y_min);
    for (int k = 0; k < n; ++k) {
      poly(k, 0) = scale * (x[k] + x_offset);
      poly(k, 1) = scale * (y[k] + y_offset);
    }
    return 0;
  }
};
#endif  // PAVEL_VALTR_01











#endif /* B26B5C4A_CE18_4298_B201_15F2EE043752 */
