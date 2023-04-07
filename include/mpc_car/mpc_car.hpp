#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <deque>
#include <iosqp/iosqp.hpp>

#include "arc_spline/arc_spline.hpp"
#include "lbfgs_raw.hpp"
// #define dubug_mode 
namespace mpc_car
{

  static constexpr int n = 4; // state x y phi v
  static constexpr int m = 2; // input a delta
  typedef Eigen::Matrix<double, n, n> MatrixA;
  typedef Eigen::Matrix<double, n, m> MatrixB;
  typedef Eigen::Vector4d VectorG;
  typedef Eigen::Vector4d VectorX;
  typedef Eigen::Vector2d VectorU;
  using Evx = Eigen::VectorXd;
  using E3x = Eigen::Vector3d;
  using Emx = Eigen::MatrixXd;
  class MpcCar
  {
  private:
    ros::NodeHandle nh_;
    ros::Publisher ref_pub_, traj_pub_, traj_delay_pub_;
    bool init_ = false;

    double ll_;  // 前后轮距离
    double dt_;  // 离散时间
    double rho_; // 角度变化权重
    int N_;      // 预测时域40
    double rhoN_;

    double v_max_, a_max_, delta_max_, ddelta_max_;
    double delay_;

    arc_spline::ArcSpline s_;
    double desired_v_;

    osqp::IOSQP qpSolver_;

    std::vector<VectorX> predictState_;
    std::vector<Eigen::VectorXd> reference_states_;
    std::vector<VectorU> predictInput_;
    std::deque<VectorU> historyInput_;
    int history_length_;
    VectorX x0_observe_;

    MatrixA Ad_;
    MatrixB Bd_;
    VectorG gd_;
    // x_{k+1} = Ad * x_{k} + Bd * u_k + gd

    Eigen::SparseMatrix<double> P_, q_, A_, l_, u_;
    // Eigen::SparseMatrix<double> P0_, q0_;
    Eigen::SparseMatrix<double> Cx_, lx_, ux_; // p, v constrains
    Eigen::SparseMatrix<double> Cu_, lu_, uu_; // a delta vs constrains
    Eigen::SparseMatrix<double> Qx_;

    // alm solver
    double alm_rho_, alm_beta_, alm_gamma_;
    Evx alm_mu_;

    std::vector<Eigen::Vector3d> violation_;
    //phr solver
    double phr_rho_=1.0, phr_gamma_=1.0, phr_beta_=1000.0, phr_xi_=0.1;
    Eigen::VectorXd mu_;
    double last_acc_ = 0.0, last_delta_ = 0.0;
    void linearization(const double &phi,
                       const double &v,
                       const double &delta)
    {
      // TODO: set values to Ad_, Bd_, gd_
      Ad_(0, 2) = -v * sin(phi) * dt_;
      Ad_(0, 3) = cos(phi) * dt_;
      Ad_(1, 2) = v * cos(phi) * dt_;
      Ad_(1, 3) = sin(phi) * dt_;
      Ad_(2, 3) = tan(delta) / ll_ * dt_;
      Bd_(2, 1) = v / ll_ / cos(delta) / cos(delta) * dt_;
      gd_(0) = v * sin(phi) * dt_ * phi;
      gd_(1) = -v * cos(phi) * dt_ * phi;
      gd_(2) = -v / ll_ / cos(delta) / cos(delta) * dt_ * delta;
      return;
    }

    void calLinPoint(const double &s0, double &phi, double &v, double &delta)
    {
      Eigen::Vector2d dxy = s_(s0, 1);
      Eigen::Vector2d ddxy = s_(s0, 2);
      double dx = dxy.x();
      double dy = dxy.y();
      double ddx = ddxy.x();
      double ddy = ddxy.y();
      double dphi = (ddy * dx - dy * ddx) / (dx * dx + dy * dy);
      phi = atan2(dy, dx);
      v = desired_v_;
      delta = atan2(ll_ * dphi, 1.0);
    }

    inline VectorX diff(const VectorX &state,
                        const VectorU &input) const
    {
      VectorX ds;
      double phi = state(2);
      double v = state(3);
      double a = input(0);
      double delta = input(1);
      ds(0) = v * cos(phi);
      ds(1) = v * sin(phi);
      ds(2) = v / ll_ * tan(delta);
      ds(3) = a;
      return ds;
    }

    inline void step(VectorX &state, const VectorU &input, const double dt) const
    {
      // Runge–Kutta
      VectorX k1 = diff(state, input);
      VectorX k2 = diff(state + k1 * dt / 2, input);
      VectorX k3 = diff(state + k2 * dt / 2, input);
      VectorX k4 = diff(state + k3 * dt, input);
      state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
    }

    VectorX compensateDelay(const VectorX &x0)
    {
      VectorX x0_delay = x0;
      // TODO: compensate delay
      double dt = 0.001;
      for (double t = delay_; t > 0; t -= dt)
      {
        int i = std::ceil(t / dt_);
        VectorU input = historyInput_[history_length_ - i];
        step(x0_delay, input, dt);
      }
      return x0_delay;
    }

  public:
    MpcCar(ros::NodeHandle &nh) : nh_(nh)
    {
      // load map
      std::vector<double> track_points_x, track_points_y;
      nh.getParam("track_points_x", track_points_x);
      nh.getParam("track_points_y", track_points_y);
      nh.getParam("desired_v", desired_v_);
      s_.setWayPoints(track_points_x, track_points_y);
      // load parameters
      nh.getParam("ll", ll_);
      nh.getParam("dt", dt_);
      nh.getParam("rho", rho_);
      nh.getParam("N", N_);
      nh.getParam("rhoN", rhoN_);
      nh.getParam("v_max", v_max_);
      nh.getParam("a_max", a_max_);
      nh.getParam("delta_max", delta_max_);
      nh.getParam("ddelta_max", ddelta_max_);
      nh.getParam("delay", delay_);
      history_length_ = std::ceil(delay_ / dt_);

      ref_pub_ = nh.advertise<nav_msgs::Path>("reference_path", 1);
      traj_pub_ = nh.advertise<nav_msgs::Path>("traj", 1);
      traj_delay_pub_ = nh.advertise<nav_msgs::Path>("traj_delay", 1);

      // TODO: set initial value of Ad, Bd, gd
      Ad_.setIdentity(); // Ad for instance
      Bd_.setZero();
      Bd_(3, 0) = dt_;
      gd_.setZero();
      // set size of sparse matrices
      P_.resize(m * N_, m * N_);
      q_.resize(m * N_, 1);
      Qx_.resize(n * N_, n * N_);
      // stage cost
      Qx_.setIdentity();
      for (int i = 1; i < N_; ++i)
      {
        Qx_.coeffRef(i * n - 2, i * n - 2) = rho_;
        Qx_.coeffRef(i * n - 1, i * n - 1) = 0;
      }
      Qx_.coeffRef(N_ * n - 4, N_ * n - 4) = rhoN_;
      Qx_.coeffRef(N_ * n - 3, N_ * n - 3) = rhoN_;
      Qx_.coeffRef(N_ * n - 2, N_ * n - 2) = rhoN_ * rho_;
      int n_cons = 4; // v a delta ddelta
      A_.resize(n_cons * N_, m * N_);
      l_.resize(n_cons * N_, 1);
      u_.resize(n_cons * N_, 1);
      // v constrains
      Cx_.resize(1 * N_, n * N_);
      lx_.resize(1 * N_, 1);
      ux_.resize(1 * N_, 1);
      // a delta constrains
      Cu_.resize(3 * N_, m * N_);
      lu_.resize(3 * N_, 1);
      uu_.resize(3 * N_, 1);
      // set lower and upper boundaries
      for (int i = 0; i < N_; ++i)
      {
        // TODO: set stage constraints of inputs (a, delta, ddelta)
        // -a_max <= a <= a_max for instance:
        Cu_.coeffRef(i * 3 + 0, i * m + 0) = 1;
        Cu_.coeffRef(i * 3 + 1, i * m + 1) = 1;
        Cu_.coeffRef(i * 3 + 2, i * m + 1) = 1;
        lu_.coeffRef(i * 3 + 0, 0) = -a_max_;
        uu_.coeffRef(i * 3 + 0, 0) = a_max_;
        lu_.coeffRef(i * 3 + 1, 0) = -delta_max_;
        uu_.coeffRef(i * 3 + 1, 0) = delta_max_;
        lu_.coeffRef(i * 3 + 2, 0) = -ddelta_max_ * dt_;
        uu_.coeffRef(i * 3 + 2, 0) = ddelta_max_ * dt_;
        if (i > 0)
        {
          Cu_.coeffRef(i * 3 + 2, (i - 1) * m + 1) = -1;
        }

        // TODO: set stage constraints of states (v)
        // -v_max <= v <= v_max
        Cx_.coeffRef(i, i * n + 3) = 1;
        lx_.coeffRef(i, 0) = -0.1;
        ux_.coeffRef(i, 0) = v_max_;
      }
      // set predict mats size
      predictState_.resize(N_);
      predictInput_.resize(N_);
      for (int i = 0; i < N_; ++i)
      {
        predictInput_[i].setZero();
      }
      for (int i = 0; i < history_length_; ++i)
      {
        historyInput_.emplace_back(0, 0);
      }
    }

    int solveQP(const VectorX &x0_observe)
    {
      x0_observe_ = x0_observe;
      historyInput_.pop_front();
      historyInput_.push_back(predictInput_.front());
      lu_.coeffRef(2, 0) = predictInput_.front()(1) - ddelta_max_ * dt_;
      uu_.coeffRef(2, 0) = predictInput_.front()(1) + ddelta_max_ * dt_;
      VectorX x0 =x0_observe_;// compensateDelay(x0_observe_);
      // set BB, AA, gg
      Eigen::MatrixXd BB, AA, gg;
      BB.setZero(n * N_, m * N_);
      AA.setZero(n * N_, n);
      gg.setZero(n * N_, 1);
      double s0 = s_.findS(x0.head(2));
      double phi, v, delta;
      double last_phi = x0(2);
      Eigen::SparseMatrix<double> qx;
      qx.resize(n * N_, 1);
      for (int i = 0; i < N_; ++i)
      {
        calLinPoint(s0, phi, v, delta);
        if (phi - last_phi > M_PI)
        {
          phi -= 2 * M_PI;
        }
        else if (phi - last_phi < -M_PI)
        {
          phi += 2 * M_PI;
        }
        last_phi = phi;
        if (init_)
        {
          double phii = predictState_[i](2);
          v = predictState_[i](3);
          delta = predictInput_[i](1);
          if (phii - last_phi > M_PI)
          {
            phii -= 2 * M_PI;
          }
          else if (phii - last_phi < -M_PI)
          {
            phii += 2 * M_PI;
          }
          last_phi = phii;
          linearization(phii, v, delta);
        }
        else
        {
          linearization(phi, v, delta);
        }
        // calculate big state-space matrices
        /* *                BB                AA
         * x1    /       B    0  ... 0 \    /   A \
         * x2    |      AB    B  ... 0 |    |  A2 |
         * x3  = |    A^2B   AB  ... 0 |u + | ... |x0 + gg
         * ...   |     ...  ...  ... 0 |    | ... |
         * xN    \A^(n-1)B  ...  ... B /    \ A^N /
         *
         *     X = BB * U + AA * x0 + gg
         * */
        if (i == 0)
        {
          BB.block(0, 0, n, m) = Bd_;
          AA.block(0, 0, n, n) = Ad_;
          gg.block(0, 0, n, 1) = gd_;
        }
        else
        {
          // TODO: set BB AA gg
          BB.block(n * i, 0, n, m * N_) = Ad_ * BB.block(n * (i - 1), 0, n, m * N_);
          BB.block(n * i, m * i, n, m) = Bd_;
          AA.block(n * i, 0, n, n) = Ad_ * AA.block(n * (i - 1), 0, n, n);
          gg.block(n * i, 0, n, 1) = Ad_ * gg.block(n * (i - 1), 0, n, 1) + gd_;
        }
        // TODO: set qx
        Eigen::Vector2d xy = s_(s0); // reference (x_r, y_r)
        qx.coeffRef(i * n + 0, 0) = -xy.x();
        qx.coeffRef(i * n + 1, 0) = -xy.y();
        qx.coeffRef(i * n + 2, 0) = -rho_ * phi;
        // std::cout << "phi[" << i << "]: " << phi << std::endl;
        if (i == N_ - 1)
        {
          qx.coeffRef(i * n + 0, 0) *= rhoN_;
          qx.coeffRef(i * n + 1, 0) *= rhoN_;
          qx.coeffRef(i * n + 2, 0) *= rhoN_;
        }
        s0 += desired_v_ * dt_;
        s0 = s0 < s_.arcL() ? s0 : s_.arcL();
      }
      Eigen::SparseMatrix<double> BB_sparse = BB.sparseView();
      Eigen::SparseMatrix<double> AA_sparse = AA.sparseView();
      Eigen::SparseMatrix<double> gg_sparse = gg.sparseView();
      Eigen::SparseMatrix<double> x0_sparse = x0.sparseView();

      Eigen::SparseMatrix<double> Cx = Cx_ * BB_sparse;
      Eigen::SparseMatrix<double> lx = lx_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;
      Eigen::SparseMatrix<double> ux = ux_ - Cx_ * AA_sparse * x0_sparse - Cx_ * gg_sparse;
      Eigen::SparseMatrix<double> A_T = A_.transpose();
      A_T.middleCols(0, Cx.rows()) = Cx.transpose();
      A_T.middleCols(Cx.rows(), Cu_.rows()) = Cu_.transpose();
      A_ = A_T.transpose();
      for (int i = 0; i < lx.rows(); ++i)
      {
        l_.coeffRef(i, 0) = lx.coeff(i, 0);
        u_.coeffRef(i, 0) = ux.coeff(i, 0);
      }
      for (int i = 0; i < lu_.rows(); ++i)
      {
        l_.coeffRef(i + lx.rows(), 0) = lu_.coeff(i, 0);
        u_.coeffRef(i + lx.rows(), 0) = uu_.coeff(i, 0);
      }
      Eigen::SparseMatrix<double> BBT_sparse = BB_sparse.transpose();
      P_ = BBT_sparse * Qx_ * BB_sparse;
      q_ = BBT_sparse * Qx_.transpose() * (AA_sparse * x0_sparse + gg_sparse) + BBT_sparse * qx;
      // osqp
      Eigen::VectorXd q_d = q_.toDense();
      Eigen::VectorXd l_d = l_.toDense();
      Eigen::VectorXd u_d = u_.toDense();
      qpSolver_.setMats(P_, q_d, A_, l_d, u_d);
      qpSolver_.solve();
      int ret = qpSolver_.getStatus();
      if (ret != 1)
      {
        ROS_ERROR("fail to solve QP!");
        return ret;
      }
      Eigen::VectorXd sol = qpSolver_.getPrimalSol();
      Eigen::MatrixXd solMat = Eigen::Map<const Eigen::MatrixXd>(sol.data(), m, N_);
      Eigen::VectorXd solState = BB * sol + AA * x0 + gg;
      Eigen::MatrixXd predictMat = Eigen::Map<const Eigen::MatrixXd>(solState.data(), n, N_);

      for (int i = 0; i < N_; ++i)
      {
        predictInput_[i] = solMat.col(i);
        predictState_[i] = predictMat.col(i);
      }
      init_ = true;
      return ret;
    }

    void getPredictXU(double t, VectorX &state, VectorU &input)
    {
      if (t <= dt_)
      {
        state = predictState_.front();
        input = predictInput_.front();
        return;
      }
      int horizon = std::floor(t / dt_);
      double dt = t - horizon * dt_;
      state = predictState_[horizon - 1];
      input = predictInput_[horizon - 1];
      double phi = state(2);
      double v = state(3);
      double a = input(0);
      double delta = input(1);
      state(0) += dt * v * cos(phi);
      state(1) += dt * v * sin(phi);
      state(2) += dt * v / ll_ * tan(delta);
      state(3) += dt * a;
    }

    void forward(const VectorX &xk,
                 const VectorU &uk,
                 VectorX &xk_1)
    {
      // const auto& x = xk(0);
      // const auto& y = xk(1);
      const auto &phi = xk(2);
      const auto &v = xk(3);
      const auto &a = uk(0);
      const auto &delta = uk(1);
      xk_1 = xk;
      xk_1(0) += v * std::cos(phi) * dt_;
      xk_1(1) += v * std::sin(phi) * dt_;
      xk_1(2) += v / ll_ * std::tan(delta) * dt_;
      xk_1(3) += a * dt_;
    }

    // obj.backward(states[i], inputs.col(i), grad_xk_1, grad_xk, grad_uk);
    void backward(const VectorX &xk,
                  const VectorU &uk,
                  const VectorX &grad_xk_1,
                  VectorX &grad_xk,
                  VectorU &grad_uk)
    {
      // const auto& x = xk(0);
      // const auto& y = xk(1);
      const auto &phi = xk(2);
      const auto &v = xk(3);
      // const auto& a = uk(0);
      const auto &delta = uk(1);
      const auto &grad_x_1 = grad_xk_1(0);
      const auto &grad_y_1 = grad_xk_1(1);
      const auto &grad_phi_1 = grad_xk_1(2);
      const auto &grad_v_1 = grad_xk_1(3);
      // auto& grad_x = grad_xk(0);
      // auto& grad_y = grad_xk(1);
      auto &grad_phi = grad_xk(2);
      auto &grad_v = grad_xk(3);
      auto &grad_a = grad_uk(0);
      auto &grad_delta = grad_uk(1);
      grad_xk = grad_xk_1;
      grad_uk.setZero();
      grad_v += grad_x_1 * std::cos(phi) * dt_;
      grad_phi += grad_x_1 * v * (-std::sin(phi)) * dt_;
      grad_v += grad_y_1 * std::sin(phi) * dt_;
      grad_phi += grad_y_1 * v * std::cos(phi) * dt_;
      grad_v += grad_phi_1 * std::tan(delta) / ll_ * dt_;
      grad_delta += grad_phi_1 * v / ll_ / std::cos(delta) / std::cos(delta) * dt_;
      grad_a += grad_v_1 * dt_;
    }

    double box_constrant(const double &x,
                         const double &l,
                         const double &u,
                         double &grad)
    {
      double rho = 1e4;
      double lpen = l - x;
      double upen = x - u;
      if (lpen > 0)
      {
        double lpen2 = lpen * lpen;
        grad = -rho * 3 * lpen2;
        return rho * lpen2 * lpen;
      }
      else if (upen > 0)
      {
        double upen2 = upen * upen;
        grad = rho * 3 * upen2;
        return rho * upen2 * upen;
      }
      else
      {
        grad = 0;
        return 0;
      }
    }

    double stage_cost_gradient(const int &k,
                               const VectorX &x,
                               VectorX &grad_x)
    {
      const Eigen::Vector3d &x_r = reference_states_[k];
      Eigen::Vector3d dx = x.head(3) - x_r; // 向量相-
      grad_x.head(3) = 2 * dx;              // 代价函数中梯度信息
      grad_x(3) = 0;                        //
      double cost = dx.squaredNorm();
      // TODO: penalty constraints
      double grad_v = 0;
      cost += box_constrant(x(3), -0.1, v_max_, grad_v);
      grad_x(3) += grad_v;
      return cost;
    }

    static inline double objectiveFunc(void *ptrObj,
                                       const double *x,
                                       double *grad,
                                       const int n)
    {
      // std::cout << "\033[32m ************************************** \033[0m" << std::endl;
      MpcCar &obj = *(MpcCar *)ptrObj;
      Eigen::Map<const Eigen::MatrixXd> inputs(x, m, obj.N_); // 转换为eigen matrix存
      Eigen::Map<Eigen::MatrixXd> grad_inputs(grad, m, obj.N_);

      // forward propogate
      std::vector<VectorX> states(obj.N_ + 1);
      states[0] = obj.x0_observe_;
      VectorX xk_1 = obj.x0_observe_;
      for (int i = 0; i < obj.N_; ++i)
      {
        obj.forward(states[i], inputs.col(i), xk_1); // 前向积分
        states[i + 1] = xk_1;
      }
      // cost and gradient of states
      double total_cost = 0;
      VectorX grad_xk, grad_xk_1;
      VectorU grad_uk;
      grad_xk.setZero();
      for (int i = obj.N_ - 1; i >= 0; i--)
      {
        total_cost += obj.stage_cost_gradient(i, states[i + 1], grad_xk_1);
        grad_xk_1 = grad_xk_1 + grad_xk;
        obj.backward(states[i], inputs.col(i), grad_xk_1, grad_xk, grad_uk);
        grad_inputs.col(i) = grad_uk;
      }
      // cost and gradient of inputs
      for (int i = 0; i < obj.N_; ++i)
      {
        double a = inputs.col(i)(0);
        double delta = inputs.col(i)(1);
        double grad_a, grad_delta;
        total_cost += obj.box_constrant(a, -obj.a_max_, obj.a_max_, grad_a);
        grad_inputs.col(i)(0) += grad_a;
        total_cost += obj.box_constrant(delta, -obj.delta_max_, obj.delta_max_, grad_delta);
        grad_inputs.col(i)(1) += grad_delta;
      }
      for (int i = 0; i < obj.N_ - 1; ++i)
      {
        double delta_k = inputs.col(i)(1);
        double delta_k_1 = inputs.col(i + 1)(1);
        double ddelta = delta_k_1 - delta_k;
        double grad_ddelta;
        total_cost += obj.box_constrant(ddelta,
                                        -obj.ddelta_max_ * obj.dt_,
                                        obj.ddelta_max_ * obj.dt_,
                                        grad_ddelta);
        grad_inputs.col(i)(1) -= grad_ddelta;
        grad_inputs.col(i + 1)(1) += grad_ddelta;
      }
      return total_cost;
    }

    int solveNMPC(const VectorX &x0_observe)
    {
      historyInput_.pop_front();
      historyInput_.push_back(predictInput_.front());
      // x0_observe_ = x0_observe;
      x0_observe_ = compensateDelay(x0_observe); // 延时观测值
      double s0 = s_.findS(x0_observe_.head(2));

      reference_states_.resize(N_);

      Eigen::Vector2d xy_r;
      double phi, last_phi = x0_observe_(2);
      // std::ofstream FoutC("/home/ming/ros1_workspace/mpc-car-tutorial-master/refxy.txt",std::ios::app);
      //   writefile(FoutC,s_(s0)(0),s_(s0)(1));
      // std::cout<<" "<<s_(s0)(0)<<" "<<s_(s0)(1)<<std::endl;
      for (int i = 0; i < N_; ++i)
      {
        s0 += desired_v_ * dt_;
        s0 = s0 < s_.arcL() ? s0 : s_.arcL();
        // calculate desired x,y.phi
        xy_r = s_(s0);
        Eigen::Vector2d dxy = s_(s0, 1);
        phi = std::atan2(dxy.y(), dxy.x());
        if (phi - last_phi > M_PI)
        {
          phi -= 2 * M_PI;
        }
        else if (phi - last_phi < -M_PI)
        {
          phi += 2 * M_PI;
        }
        last_phi = phi;
        reference_states_[i] = Eigen::Vector3d(xy_r.x(), xy_r.y(), phi);
      }
      double *x = new double[m * N_];
      Eigen::Map<Eigen::MatrixXd> inputs(x, m, N_);
      inputs.setZero();
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.past = 3;
      lbfgs_params.g_epsilon = 0.0;
      lbfgs_params.min_step = 1e-32;
      lbfgs_params.delta = 1e-4;
      lbfgs_params.line_search_type = 0;
      double minObjective;
      auto ret = lbfgs::lbfgs_optimize(m * N_, x, &minObjective, &objectiveFunc, nullptr, nullptr, this, &lbfgs_params);
      std::cout << "\033[32m"
                << "ret: " << ret << "\033[0m" << std::endl;
      VectorX xk = x0_observe_, xk_1;
      for (int i = 0; i < N_; ++i)
      {
        predictInput_[i] = inputs.col(i);
        forward(xk, inputs.col(i), xk_1);
        predictState_[i] = xk_1;
        xk = xk_1;
      }
      return ret;
    }
    /**/
    int solvePHRALM(const VectorX &x0_observe)
    {
      historyInput_.pop_front();
      historyInput_.emplace_back(predictInput_.front());
      x0_observe_ = compensateDelay(x0_observe);
      // 按弧长来取
      double s0 = s_.findS(x0_observe_.head(2)); // Vector 取前面值的方式
      reference_states_.resize(N_);

      alm_beta_ = 1e3;
      alm_mu_ = Evx::Zero(8 * N_); // 2*4=8dim constraints
      alm_rho_ = 1;                // todo adjust
      alm_gamma_ = 1.0;

      Eigen::Vector2d xy_r;
      double phi, last_phi = x0_observe_(2); // 车辆朝向
      for (int i = 0; i < N_; ++i)
      {
        s0 += desired_v_ * dt_;

        s0 = s0 < s_.arcL() ? s0 : s_.arcL(); // arcL_ = sL_.back();
        xy_r = s_(s0);
        Eigen::Vector2d dxy = s_(s0, 1);
        phi = std::atan2(dxy.y(), dxy.x()); // 参考角度
        if (phi - last_phi > M_PI)
        { // 角度跳变
          phi -= 2 * M_PI;
        }
        else if (phi - last_phi < -M_PI)
        { // note else if
          phi += 2 * M_PI;
        }
        last_phi = phi;
        reference_states_[i] = E3x(xy_r.x(), xy_r.y(), phi); // 直接填入值
      }
      double *x = new double[m * N_];
      Eigen::Map<Emx> inputs(x, m, N_); // 转化为matrixXd(m,N_)
      inputs.setZero();                 //
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      // lbfgs_params.past = 3;
      // lbfgs_params.g_epsilon = 0.0;
      lbfgs_params.past = 0;
      lbfgs_params.g_epsilon = 0.1;
      lbfgs_params.min_step = 1e-32;
      lbfgs_params.delta = 1e-4;
      lbfgs_params.line_search_type = 0;
      double minObjective;
      double e_cons = 1e5, e_prec = 1e5;           // 精度控制
      double req_e_cons = 1e-4, req_e_prec = 1e-3; // 精度指标
      int ret = 0, times = 0;
      while (e_cons > req_e_cons || e_prec > req_e_prec)
      {
        times++;
        ret = lbfgs::lbfgs_optimize(m * N_, x, &minObjective, &PHRALM_objectiveFunction, nullptr, nullptr, this, &lbfgs_params); // lbfgs_paramsvi
        double *grad11 = (double *)vecalloc(m * N_ * sizeof(double));
        double temp_cost = PHRALM_objectiveFunction(this, x, grad11, m * N_);
        ROS_INFO_STREAM("TEMP_COST CAL=" << temp_cost);
        Evx gg = Evx::Zero(8);

        VectorX xk = x0_observe_;
        VectorX xk_1;
        e_cons = -1e5;
        e_prec = -1e5;
        int NN = 8;

        for (int i = 0; i < N_; i++)
        {
          predictInput_[i] = inputs.col(i); // u
          forward(xk, inputs.col(i), xk_1);
          predictState_[i] = xk_1; // 根据运动学模型推导得到
          xk = xk_1;

          gg[0] = -a_max_ - inputs.col(i)(0); //
          gg[1] = inputs.col(i)(0) - a_max_;
          gg[2] = -delta_max_ - inputs.col(i)(1);
          gg[3] = inputs.col(i)(1) - delta_max_;
          // TODO for 循环
          for (int j = 0; j < 4; j++)
          {
            alm_mu_(i * NN + j) = std::max(0.0, alm_mu_(i * NN + j) + alm_rho_ * gg(j));
            e_cons = std::max(e_cons, std::abs(std::max(-alm_mu_(i * NN + j) / alm_rho_, gg(j)))); // 判断是否满足条件
          }

          if (i < N_ - 1) // 非终端量
          {
            double ddelta = inputs.col(i + 1)(1) - inputs.col(i)(1);
            gg[4] = -0.1 - xk_1[3]; // TODOv_min TODO
            gg[5] = xk_1[3] - v_max_;
            gg[6] = -ddelta_max_ * dt_ - ddelta;
            gg[7] = ddelta - ddelta_max_ * dt_; // note dt_
            // for(int j=4;j<8;j++){
            //   alm_mu_(i*NN+j) = std::max(0.0,alm_mu_(i*NN+j)+alm_rho_*gg(j));
            // }
            for (int j = 4; j < 8; j++)
            {
              alm_mu_(i * NN + j) = std::max(0.0, alm_mu_(i * NN + j) + alm_rho_ * gg(j));
              e_cons = std::max(e_cons, std::abs(std::max(-alm_mu_(i * NN + j) / alm_rho_, gg(j)))); // 判断是否满足条件
            }
          }
        }

        for (int i = 0; i < m * N_; ++i)
        {
          e_prec = std::max(e_prec, std::abs(grad11[i]));
        }
        alm_rho_ = std::min((1 + alm_gamma_) * alm_rho_, alm_beta_);
        lbfgs_params.g_epsilon = std::max(lbfgs_params.g_epsilon * 0.1, 1e-4);
      }

      VectorX xk = x0_observe_;
      VectorX xk_1; // k+1
      Eigen::Vector3d max_violation = -Eigen::Vector3d::Ones() * 1e3;
      Eigen::Vector3d violation = -Eigen::Vector3d::Ones() * 1e3;
      for (int i = 0; i < N_; i++)
      {
        predictInput_[i] = inputs.col(i); //
        forward(xk, inputs.col(i), xk_1);
        predictState_[i] = xk_1;
        xk = xk_1;
        constrain_violation(violation, predictState_[i], predictInput_[i]);
      }
      violation_.emplace_back(violation);
      violation.setZero(); // reset
      for (long unsigned int i = 0; i < violation_.size(); i++)
      {
        auto temp = violation_[i];
        violation += temp;
        for (int j = 0; j < 3; j++)
        {
          max_violation(j) = std::max(max_violation(j), temp(j));
        }
      }
      ROS_INFO_STREAM("OUT LOOP times" << times);
      ROS_INFO_STREAM("e_prec= " << e_prec << "\n"
                                 << "e_cons" << e_cons);
      violation /= violation_.size(); // note no / = 不能有空格
      ROS_INFO_STREAM("average constraints out =\n"
                      << violation);
      ROS_INFO_STREAM("max constraints out =\n"
                      << max_violation);
      return ret;
    }
    inline void *vecalloc(size_t size)
    {
      void *memblock = malloc(size);
      if (memblock)
      {
        memset(memblock, 0, size);
      }
      return memblock;
    }
    static inline double PHRALM_objectiveFunction(void *ptrObj,
                                                  const double *x,
                                                  double *grad,
                                                  const int n)
    {
      MpcCar &obj = *(MpcCar *)ptrObj;
      Eigen::Map<const Emx> inputs(x, m, obj.N_); // 数据类型转换 优化变量 m*N_矩阵
      Eigen::Map<Emx> grad_inputs(grad, m, obj.N_);

      // 前向模拟
      std::vector<VectorX> states(obj.N_ + 1);
      states[0] = obj.x0_observe_;
      VectorX xk_1 = obj.x0_observe_;
      for (int i = 0; i < obj.N_; i++)
      {
        obj.forward(states[i], inputs.col(i), xk_1); // i+1的状态
        states[i + 1] = xk_1;
      }
      // cost and grad of states
      double total_cost = 0;
      VectorX grad_xk, grad_xk_1;
      VectorU grad_uk;
      grad_xk.setZero();
      for (int i = obj.N_ - 1; i >= 0; i--) // 从后往前
      {
        total_cost += obj.PHRALM_stage_cost_gradient(i, states[i + 1], grad_xk_1);
        // std::cout<<i<<" total_cost = "<<total_cost<<std::endl;
        grad_xk_1 = grad_xk_1 + grad_xk;
        obj.backward(states[i], inputs.col(i), grad_xk_1, grad_xk, grad_uk);
        grad_inputs.col(i) = grad_uk;
      }
      int j = 1;
      ROS_INFO_STREAM(j++ << " COST TOTAL = " << total_cost);
      // cost and grad of inputs;
      for (int i = 0; i < obj.N_; i++)
      {
        double a = inputs.col(i)(0);     // a
        double delta = inputs.col(i)(1); // 当前的
        double grad_a, grad_delta;
        total_cost += obj.PHRALM_box_constraints(a, -obj.a_max_, obj.a_max_, grad_a, i, 0);
        grad_inputs.col(i)(0) += grad_a;
        total_cost += obj.PHRALM_box_constraints(delta, -obj.delta_max_, obj.delta_max_, grad_delta, i, 1);
        grad_inputs.col(i)(1) += grad_delta;
      }
      ROS_INFO_STREAM(j++ << " COST TOTAL = " << total_cost);

      for (int i = 0; i < obj.N_ - 1; i++)
      {
        double delta_k = inputs.col(i)(1);
        double delta_k_1 = inputs.col(i + 1)(1); //
        double ddelta = delta_k_1 - delta_k;
        double grad_ddelta; // 角加速度
        // note *obj.dt_
        total_cost += obj.PHRALM_box_constraints(ddelta, -obj.ddelta_max_ * obj.dt_, obj.ddelta_max_ * obj.dt_, grad_ddelta, i, 3);
        grad_inputs.col(i)(1) -= grad_ddelta;
        grad_inputs.col(i + 1)(1) += grad_ddelta;
      }
      ROS_INFO_STREAM(j++ << " COST TOTAL = " << total_cost);

      return total_cost;
    }
    inline double NormalizeAngle(const double angle)
    {
      double a = std::fmod(angle + M_PI, 2 * M_PI);
      if (a < 0)
      {
        a = a + M_PI;
        return a;
      }
      else
      {
        a = a - M_PI;
        return a;
      }
    }
    // PHRALM_box_constraints(x(3),-0.1,v_max_,grad_v,k,2);
    double PHRALM_box_constraints(const double &x,
                                  const double &l, // low
                                  const double &u, // upper
                                  double &grad,
                                  const int k,
                                  const int i) // 不同的状态
    {
      double lpen = l - x + alm_mu_(8 * k + 2 * i) / alm_rho_; // G+mu/rho
      double upen = x - u + alm_mu_(8 * k + 2 * i + 1) / alm_rho_;
      if (lpen > 0)
      {
        double lpen2 = lpen * lpen;
        grad = -alm_rho_ * lpen;     // 求导
        return alm_rho_ / 2 * lpen2; // 代价函数中未求导项
      }
      else if (upen > 0)
      {
        double upen2 = upen * upen;
        grad = alm_rho_ * upen;
        return alm_rho_ / 2 * upen2;
      }
      else
      {
        grad = 0;
        return 0;
      }
    }

    void constrain_violation(E3x &violation, VectorX xk, VectorU uk)
    {
      double temp = std::max(-a_max_ - uk(0), uk(0) - a_max_);
      if (temp > violation(0))
        violation(0) = temp;
      temp = std::max(-delta_max_ - uk(1), uk(1) - delta_max_);
      if (temp > violation(1))
        violation(1) = temp;
      temp = std::max(-0.1 - xk(3), xk(3) - v_max_); // TODO VMIN
      if (temp > violation(2))
        violation(2) = temp;
    }
    // 原目标函数求导
    double PHRALM_stage_cost_gradient(const int &k,
                                      const VectorX &x,
                                      VectorX &grad_x)
    {
      const E3x &x_r = reference_states_[k];
      E3x dx = x.head(3) - x_r;
      grad_x.head(3) = 2 * dx;        // 二次求偏导
      grad_x(3) = 0;                  // v
      double cost = dx.squaredNorm(); // no squareNorm
      #ifdef debug_mode
      if (k == 0)
      {
        std::cout << "x.head(3)" << x.head(3) << std::endl;
        std::cout << "x_r" << x_r << std::endl;
      }
      #endif
      // std::cout<<"dx cost = "<<cost<<std::endl;
      double grad_v = 0;
      cost += PHRALM_box_constraints(x(3), -0.1, v_max_, grad_v, k, 2);
      #ifdef debug_mode
      if (k == 0)
        std::cout << "stage cost = " << cost << std::endl;
      #endif 

      grad_x(3) += grad_v;
      return cost;
    }
double ReferenceCostGradient(
    const int&k,
    const VectorX& x,
    VectorX& grad_x){

    const Eigen::Vector3d& x_r = reference_states_[k];
    Eigen::Vector3d dx = x.head(3) - x_r;
    grad_x.head(3) = 2 * dx;
    grad_x(3) = 0;
    double cost = dx.squaredNorm();

    double v_max_bound = x(3,0) - v_max_ + mu_(4*N_+k,0)/phr_rho_;
    if(std::max(v_max_bound, 0.0) > 0.0){
      cost += 0.5*phr_rho_*v_max_bound*v_max_bound;
      grad_x(3) += phr_rho_*v_max_bound;
    }
    double v_min_bound = -x(3,0) - 0.1 + mu_(5*N_+k,0)/phr_rho_;
    if(std::max(v_min_bound, 0.0) > 0.0){
      cost += 0.5*phr_rho_*v_min_bound*v_min_bound;
      grad_x(3) -= phr_rho_*v_min_bound;
    }

    return cost;
  }

  static inline double PHRObjeciveFunction(
    void* ptr,
    const double* x,
    double* grad,
    const int n){
    
    MpcCar& obj = *(MpcCar*)ptr;
    Eigen::Map<const Eigen::MatrixXd> inputs(x, m, obj.N_);
    Eigen::Map<Eigen::MatrixXd> grad_inputs(grad, m, obj.N_);

    // 1. forward propogate
    std::vector<VectorX> states(obj.N_ + 1);
    states[0] = obj.x0_observe_;
    VectorX xk_1 = obj.x0_observe_;
    for (int i = 0; i < obj.N_; ++i) {
      obj.forward(states[i], inputs.col(i), xk_1);
      states[i + 1] = xk_1;
    }

    // 2. cost and gradient of states
    double total_cost = 0;
    VectorX grad_xk, grad_xk_1;
    VectorU grad_uk;
    grad_xk.setZero();
    // 从远到近进行访问
    for (int i = obj.N_ - 1; i >= 0; i--) {
      total_cost += obj.ReferenceCostGradient(i, states[i + 1], grad_xk_1);
      // 根据链式求导 目标函数对x(k)求偏导有k时刻和k+1时刻两部分组成
      // dJ/dx(k) = dJ/dx(k) + dJ/dx(k+1) * dx(k+1)/dx(k)
      // 一定要注意这里的时间顺序
      // grad_xk = dJ/dx(k+1) * dx(k+1)/dx(k) 
      // grad_xk_1 = dJ/dx(k)
      grad_xk_1 = grad_xk_1 + grad_xk;
      
      obj.backward(states[i], inputs.col(i), grad_xk_1, grad_xk, grad_uk);
      // 对状态量的偏导最终也会变成对输入量的偏导 因为我们优化的是输入量
      grad_inputs.col(i) = grad_uk;
    }

    // 3. cost and gradient of inputs
    for(int i=0; i<obj.N_; i++){
      // acc max
      double acc_max_bound = inputs.col(i)(0) - obj.a_max_ + obj.mu_(i,0)/obj.phr_rho_;
      if(std::max(acc_max_bound, 0.0) > 0.0){
        total_cost += 0.5 * obj.phr_rho_ * acc_max_bound * acc_max_bound;
        grad_inputs.col(i)(0) += obj.phr_rho_ * acc_max_bound;
      }
      // acc min
      int idx = obj.N_;
      double acc_min_bound = -inputs.col(i)(0) - obj.a_max_ + obj.mu_(i+idx,0)/obj.phr_rho_;
      if(std::max(acc_min_bound, 0.0) > 0.0){
        total_cost += 0.5 * obj.phr_rho_ * acc_min_bound * acc_min_bound;
        grad_inputs.col(i)(0) -= obj.phr_rho_ * acc_min_bound;
      }
      // delta max
      idx = 2*obj.N_;
      double delta_max_bound = inputs.col(i)(1) - obj.delta_max_ + obj.mu_(i+idx,0)/obj.phr_rho_;
      if(std::max(delta_max_bound, 0.0) > 0.0){
        total_cost += 0.5 * obj.phr_rho_ * delta_max_bound * delta_max_bound;
        grad_inputs.col(i)(1) += obj.phr_rho_ * delta_max_bound;
      }
      // delta min
      idx = 3*obj.N_;
      double delta_min_bound = -inputs.col(i)(1) - obj.delta_max_ + obj.mu_(i+idx,0)/obj.phr_rho_;
      if(std::max(delta_min_bound, 0.0) > 0.0){
        total_cost += 0.5 * obj.phr_rho_ * delta_min_bound * delta_min_bound;
        grad_inputs.col(i)(1) -= obj.phr_rho_ * delta_min_bound;
      }

      if(i < obj.N_ - 1){
        // ddelta max
        idx = 6*obj.N_;
        double ddelta_max_bound 
          = inputs.col(i+1)(1) - inputs.col(i)(1) - obj.ddelta_max_*obj.dt_ 
          + obj.mu_(i+idx,0)/obj.phr_rho_;
        if(std::max(ddelta_max_bound, 0.0) > 0.0){
          total_cost += 0.5 * obj.phr_rho_ * ddelta_max_bound * ddelta_max_bound;
          grad_inputs.col(i)(1) -= obj.phr_rho_ * ddelta_max_bound;
          grad_inputs.col(i+1)(1) += obj.phr_rho_ * ddelta_max_bound;
        }
        // ddelta min
        idx = 7*obj.N_ - 1;
        double ddelta_min_bound
          = inputs.col(i)(1) - inputs.col(i+1)(1) - obj.ddelta_max_*obj.dt_ 
          + obj.mu_(i+idx,0)/obj.phr_rho_;
        if(std::max(ddelta_min_bound, 0.0) > 0.0){
          total_cost += 0.5 * obj.phr_rho_ * ddelta_min_bound * ddelta_min_bound;
          grad_inputs.col(i)(1) += obj.phr_rho_ * ddelta_min_bound;
          grad_inputs.col(i+1)(1) -= obj.phr_rho_ * ddelta_min_bound;
        }
      }


      // cost and gradient of control smooth
      if(i==0){
        double diff_acc = inputs.col(i)(0) - obj.last_acc_;
        total_cost += diff_acc * diff_acc;
        grad_inputs.col(i)(0) += 2.0 * diff_acc;

        double diff_delta = inputs.col(i)(1) - obj.last_delta_;
        total_cost += diff_delta * diff_delta;
        grad_inputs.col(i)(1) += 2.0 * diff_delta;
      } else {
        double diff_acc = inputs.col(i)(0) - inputs.col(i-1)(0);
        total_cost += diff_acc * diff_acc;
        grad_inputs.col(i)(0) += 2.0 * diff_acc;
        grad_inputs.col(i-1)(0) -= 2.0 * diff_acc;

        double diff_delta = inputs.col(i)(1) - inputs.col(i-1)(1);
        total_cost += diff_delta * diff_delta;
        grad_inputs.col(i)(1) += 2.0 * diff_delta;
        grad_inputs.col(i-1)(1) -= 2.0 * diff_delta;
      }
    }



    // std::cout<<"grad"<<std::endl<<grad_inputs<<std::endl;

    return total_cost;
  }

  int SolvePHR(const VectorX& x0_observe){
    historyInput_.pop_front();
    historyInput_.push_back(predictInput_.front());
    // x0_observe_ = x0_observe;
    x0_observe_ = compensateDelay(x0_observe);
    double s0 = s_.findS(x0_observe_.head(2));
    reference_states_.resize(N_);

    Eigen::Vector2d xy_r;
    double phi, last_phi = x0_observe_(2);
    for (int i = 0; i < N_; ++i) {
      s0 += desired_v_ * dt_;
      s0 = s0 < s_.arcL() ? s0 : s_.arcL();
      // calculate desired x,y.phi
      xy_r = s_(s0);
      Eigen::Vector2d dxy = s_(s0, 1);
      phi = std::atan2(dxy.y(), dxy.x());
      if (phi - last_phi > M_PI) {
        phi -= 2 * M_PI;
      } else if (phi - last_phi < -M_PI) {
        phi += 2 * M_PI;
      }
      last_phi = phi;
      reference_states_[i] = Eigen::Vector3d(xy_r.x(), xy_r.y(), phi);
    }

    phr_rho_=1.0, phr_gamma_=1.0, phr_beta_=1000.0, phr_xi_=0.1;
    mu_ = Eigen::VectorXd::Zero(8*N_-2, 1);
    // mu_ = Eigen::VectorXd::Zero(4*N_, 1);
    bool found = false;
    double* x = new double[m * N_];
    Eigen::Map<Eigen::MatrixXd> inputs(x, m, N_);
    inputs.setZero();
    int loop_count = 0;
    int ret = -1;
    double kkt_1 = 100.0;
    while(!found){

      // solve inner loop via L-BFGS
      lbfgs::lbfgs_parameter_t lbfgs_params;
      lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
      lbfgs_params.mem_size = 16;
      lbfgs_params.past = 3;
      double g_e = phr_xi_ * std::min(1.0, kkt_1);
      if(g_e < 1e-5){
        g_e = 1e-5;
      }
      lbfgs_params.g_epsilon = g_e; // gradient-based convergence test
      lbfgs_params.min_step = 1e-32;
      lbfgs_params.delta = 1e-6; // delta-based convergence test 1e-4
      lbfgs_params.line_search_type = 0;
      lbfgs_params.max_iterations = 1000;
      double minObjective, gradient_norm;
      int iteration_count;
      // auto ret = lbfgs::lbfgs_optimize(m * N_,
      //  x, 
      //  &minObjective,
      //   &objectiveFunc, 
      //   nullptr, 
      //   nullptr, 
      //   this,
      //    &lbfgs_params);

      ret = lbfgs::lbfgs_optimize(
        m * N_, x, 
        &minObjective, 
        &PHRObjeciveFunction, 
        nullptr, 
        nullptr, 
        this, 
        &lbfgs_params);
      
      VectorX xk = x0_observe_, xk_1;
      for (int i = 0; i < N_; ++i) {
        predictInput_[i] = inputs.col(i);
        forward(xk, inputs.col(i), xk_1);
        predictState_[i] = xk_1;
        xk = xk_1;
      }
      
      // update mu
      for(int i=0; i<N_; i++){
        // acc max
        double acc_max_bound = mu_(i, 0) + phr_rho_*(inputs.col(i)(0)-a_max_);
        mu_(i,0) = std::max(acc_max_bound, 0.0);
        // acc min
        int idx = N_;
        double acc_min_bound = mu_(i+idx, 0) + phr_rho_*(-inputs.col(i)(0)-a_max_);
        mu_(i+idx, 0) = std::max(acc_min_bound, 0.0);
        // delta max
        idx = 2*N_;
        double delta_max_bound = mu_(i+idx, 0) + phr_rho_*(inputs.col(i)(1)-delta_max_);
        mu_(i+idx, 0) = std::max(delta_max_bound, 0.0);
        // delta min
        idx = 3*N_;
        double delta_min_bound = mu_(i+idx, 0) + phr_rho_*(-inputs.col(i)(1)-delta_max_);
        mu_(i+idx, 0) = std::max(delta_min_bound, 0.0);
        // vel max
        idx = 4*N_;
        double vel_max_bound = mu_(i+idx, 0) + phr_rho_*(predictState_[i](3,0)-v_max_);
        mu_(i+idx, 0) = std::max(vel_max_bound, 0.0);
        // vel min
        idx = 5*N_;
        double vel_min_bound = mu_(i+idx, 0) + phr_rho_*(-predictState_[i](3,0)-0.1);
        mu_(i+idx, 0) = std::max(vel_min_bound, 0.0);

        if(i < N_-1){
          // ddelta max
          idx = 6*N_;
          double ddelta_max_bound 
            = mu_(i+idx, 0) + phr_rho_*(inputs.col(i+1)(1)-inputs.col(i)(1)-ddelta_max_*dt_);
          mu_(i+idx, 0) = std::max(ddelta_max_bound, 0.0);
          
          // ddelta min
          idx = 7*N_ - 1;
          double ddelta_min_bound
            = mu_(i+idx, 0) + phr_rho_*(inputs.col(i)(1)-inputs.col(i+1)(1)-ddelta_max_*dt_);
          mu_(i+idx, 0) = std::max(ddelta_min_bound, 0.0);
        }
      }

      // update rho
      phr_rho_ = std::min((1+phr_gamma_)*phr_rho_, phr_beta_);

      // update xi
      phr_xi_ = phr_xi_ * 0.1;
      if(phr_xi_ < 1e-5){
        phr_xi_ = 1e-5;
      }

      // stop criterion
      kkt_1 = 0.0;
      for(int i=0; i<N_; i++){
        // acc max
        double acc_max = abs(std::max((inputs.col(i)(0)-a_max_), -mu_(i, 0)/phr_rho_));
        if(acc_max > kkt_1){
          kkt_1 = acc_max;
        }
        // acc min
        int idx = N_;
        double acc_min = abs(std::max((-inputs.col(i)(0)-a_max_), -mu_(i+idx, 0)/phr_rho_));
        if(acc_min > kkt_1){
          kkt_1 = acc_min;
        }
        // delta max
        idx = 2*N_;
        double delta_max = abs(std::max((inputs.col(i)(1)-delta_max_), -mu_(i+idx, 0)/phr_rho_));
        if(delta_max > kkt_1){
          kkt_1 = delta_max;
        }
        // delta min
        idx = 3*N_;
        double delta_min = abs(std::max((-inputs.col(i)(1)-delta_max_), -mu_(i+idx, 0)/phr_rho_));
        if(delta_min > kkt_1){
          kkt_1 = delta_min;
        }
        // vel max
        idx = 4*N_;
        double vel_max = abs(std::max((predictState_[i](3,0)-v_max_), -mu_(i+idx, 0)/phr_rho_));
        if(vel_max > kkt_1){
          kkt_1 = vel_max;
        }
        // vel min
        idx = 5*N_;
        double vel_min = abs(std::max((-predictState_[i](3,0)-0.1), -mu_(i+idx, 0)/phr_rho_));
        if(vel_min > kkt_1){
          kkt_1 = vel_min;
        }

        if(i<N_-1){
          idx = 6*N_;
          double ddelta_max = abs(
            std::max((inputs.col(i+1)(1)-inputs.col(i)(1)-ddelta_max_*dt_), -mu_(i+idx, 0)/phr_rho_));
          if(ddelta_max > kkt_1){
            kkt_1 = ddelta_max;
          }

          idx = 7*N_-1;
          double ddelta_min = abs(
            std::max((inputs.col(i)(1)-inputs.col(i+1)(1)-ddelta_max_*dt_), -mu_(i+idx, 0)/phr_rho_));
          if(ddelta_min > kkt_1){
            kkt_1 = ddelta_min;
          }
        }

        // if(i==0){
        //   std::cout<<"acc_max: "<<acc_max<<" acc_min: "<<acc_min<<" delta_max: "<<delta_max<<" delta_min: "<<delta_min<<std::endl;
        //   std::cout<<"inputs: "<<(inputs.col(i)(0)-a_max_)<<" mu: "<<-mu_(i, 0)/phr_rho_<<std::endl;
        // }
      }

      if(kkt_1 < 1e-5 && gradient_norm < 1e-4){
        found = true;
      }

      if(loop_count>20){
        found = true;
      }

      std::cout<<"*************"<<std::endl;
      std::cout<<"out loop: "<<loop_count<<std::endl
        <<"inner loop: "<<iteration_count<<std::endl
        <<"L-BFGS: "<<lbfgs::lbfgs_strerror(ret)<<std::endl
        <<"kkt_1: "<<kkt_1<<std::endl
        <<"gnorm: "<<gradient_norm<<std::endl
        <<"cost: "<<minObjective<<std::endl
        <<"max_mu: "<<mu_.cwiseAbs().maxCoeff()<<std::endl
        <<"acc_0: "<<inputs.col(0)(0) <<std::endl
        <<"delta_0: "<<inputs.col(0)(1) <<std::endl;

      loop_count++;
    }
    // std::cout<<"result: "<<std::endl<<inputs<<std::endl;


    VectorX xk = x0_observe_, xk_1;
    for (int i = 0; i < N_; ++i) {
      predictInput_[i] = inputs.col(i);
      forward(xk, inputs.col(i), xk_1);
      predictState_[i] = xk_1;
      xk = xk_1;
    }

    last_acc_ = predictInput_[0].x();
    last_delta_ = predictInput_[0].y();

    // save result
    // std::ofstream foutC(log_path+"/log.txt", std::ios::app);
    // foutC.setf(std::ios::scientific, std::ios::floatfield);
    // foutC.precision(15);

    // foutC << predictInput_[0].x() <<" "
    //   << predictInput_[0].y() <<" "
    //  << std::endl;

    // foutC.close();

    return ret;
  }    
    // visualization
    void visualization()
    {
      nav_msgs::Path msg;
      msg.header.frame_id = "map";
      msg.header.stamp = ros::Time::now();
      geometry_msgs::PoseStamped p;
      for (double s = 0; s < s_.arcL(); s += 0.01)
      {
        p.pose.position.x = s_(s).x();
        p.pose.position.y = s_(s).y();
        p.pose.position.z = s;
        msg.poses.push_back(p);
      }
      ref_pub_.publish(msg);
      msg.poses.clear();
      for (int i = 0; i < N_; ++i)
      {
        p.pose.position.x = predictState_[i](0);
        p.pose.position.y = predictState_[i](1);
        p.pose.position.z = 0.0;
        msg.poses.push_back(p);
      }
      traj_pub_.publish(msg);
      msg.poses.clear();
      VectorX x0_delay = x0_observe_;
      double dt = 0.001;
      for (double t = delay_; t > 0; t -= dt)
      {
        int i = std::ceil(t / dt_);
        VectorU input = historyInput_[history_length_ - i];
        step(x0_delay, input, dt);
        p.pose.position.x = x0_delay(0);
        p.pose.position.y = x0_delay(1);
        p.pose.position.z = 0.0;
        msg.poses.push_back(p);
      }
      traj_delay_pub_.publish(msg);
    }
  };

} // namespace mpc_car