/**************************************************************************
 * PolynomialBase.hpp
 *
 * @Author： bornchow
 * @Date: 2023.02.12
 *
 * @Description:
 * polynomial trajectory 构造二次规划形式
 *
 *
 * 参考程序:
 *  ****************************************************/
#pragma once
#include "traj_opti/TrajType.hpp"

// #include <iostream>
// #include "ros/ros.h"
// #include <Eigen/Core>
// #include <Eigen/Eigen>
// #include <traj_opti/CorridorGen2D.hpp>

class PolynomialBase : public TrajType
{
private:
    ros::NodeHandle nh_;

    int n; // order of traj
    int m; // segmentation number of traj
    int k; // k阶导数
    bool is_debug_;

    int single_param_num_;

    Eigen::Vector3d p_vec_;
    Eigen::Vector3d v_vec_;
    Eigen::Vector3d a_vec_;
    Eigen::Vector3d j_vec_;

    Eigen::MatrixXd Qi_param_; // (n+1) * (n+1)
    Eigen::MatrixXd Q;         // m(n+1) * m(n+1)
    Eigen::MatrixXd A_equality_constraint;
    Eigen::MatrixXd b_equality_constraint;
    Eigen::MatrixXd A_inequality_constraint;
    Eigen::MatrixXd b_inequality_constraint;

    Eigen::MatrixXd P; //[m X (n+1)] X 3 (xyz)

    // cal n!
    int factorialNum(int n)
    {
        int fact = 1;
        for (int i = n; i > 0; i--)
        {
            fact *= i;
        }
        return fact;
    }

    /*
      return [1, t^1, t^2, ....t^n]
    */
    Eigen::MatrixXd getPVec(double t)
    {
        Eigen::MatrixXd pVecSignle = Eigen::MatrixXd::Zero(1, single_param_num_); // 1 X (n+1))
        for (int i = 0; i < single_param_num_; i++)
        {
            pVecSignle(0, i) = pow(t, i);
        }
        return pVecSignle;
    }

    /*
        return [0, 1, 2t, ...., nt^(n-1)]
    */
    Eigen::MatrixXd getVVec(double t)
    {
        Eigen::MatrixXd vVecSignle = Eigen::MatrixXd::Zero(1, single_param_num_); // 1 X (n+1))
        for (int i = 1; i < single_param_num_; i++)
        {
            vVecSignle(0, i) = i * pow(t, i - 1);
        }
        return vVecSignle;
    }

    /*
        return [0, 0, 2, ...., n(n-1)t^(n-2)]
    */
    Eigen::MatrixXd getaVec(double t)
    {
        Eigen::MatrixXd aVecSignle = Eigen::MatrixXd::Zero(1, single_param_num_); // 1 X (n+1))
        for (int i = 2; i < single_param_num_; i++)
        {
            aVecSignle(0, i) = i * (i - 1) * pow(t, i - 2);
        }
        return aVecSignle;
    }

public:
    PolynomialBase(ros::NodeHandle nh) : nh_(nh)
    {
        nh_.param<int>("traj_opti/order", n, 5);
        nh_.param<int>("traj_opti/min_order", k, 4);
        nh_.param<bool>("traj_opti/is_debug", is_debug_, false);

        ROS_INFO("\033[1;32m ----> This is polynomial trajectory !! .\033[0m");
        ROS_WARN_STREAM("[polynomial] param | order     : " << n);
        ROS_WARN_STREAM("[polynomial] param | min_order : " << k);
        ROS_WARN_STREAM("[polynomial] param | is_debug  : " << is_debug_);

        single_param_num_ = n + 1; //

        //
        // Qi  = [ 0                      0                               ]  |
        //       [ 0     r!c!/((r-k)!(c-k)!) * 1/(r+c-2k+1) * t^(r+c-2k+1)]  | t(i) - t(i-1)
        Qi_param_ = Eigen::MatrixXd::Zero(n + 1, n + 1);
        for (int r = k; r < Qi_param_.rows(); r++)
        {
            for (int c = k; c < Qi_param_.cols(); c++)
            {
                long double value =
                    factorialNum(r) * factorialNum(c) / (factorialNum(r - k) * factorialNum(c - k)) *
                    1.0 / (r + c - 2 * k + 1);
                Qi_param_(r, c) = value;
            }
        }

        if (is_debug_)
            ROS_INFO_STREAM("-----------Qi param----------------\n"
                            << Qi_param_);
    }

    PolynomialBase(int order, int mini_order, bool debug) : n(order),
                                                            k(mini_order),
                                                            is_debug_(debug)
    {

        single_param_num_ = n + 1; //

        //
        // Qi  = [ 0                      0                               ]  |
        //       [ 0     r!c!/((r-k)!(c-k)!) * 1/(r+c-2k+1) * t^(r+c-2k+1)]  | t(i) - t(i-1)
        Qi_param_ = Eigen::MatrixXd::Zero(n + 1, n + 1);
        for (int r = k; r < Qi_param_.rows(); r++)
        {
            for (int c = k; c < Qi_param_.cols(); c++)
            {
                long double value =
                    factorialNum(r) * factorialNum(c) / (factorialNum(r - k) * factorialNum(c - k)) *
                    1.0 / (r + c - 2 * k + 1);
                Qi_param_(r, c) = value;
            }
        }
    }
    ~PolynomialBase();

    // template <class CORRIDOR>
    void updateOptiParam(std::vector<Rectangle> corridors, Eigen::MatrixXd pos, Eigen::MatrixXd vel, Eigen::MatrixXd acc);
    Eigen::MatrixXd getQ() { return Q; }
    Eigen::MatrixXd getAEC() { return A_equality_constraint; }
    Eigen::MatrixXd getBEC() { return b_equality_constraint; }
    Eigen::MatrixXd getAIEC() { return A_inequality_constraint; }
    Eigen::MatrixXd getBIEC() { return b_inequality_constraint; }
    std::vector<Eigen::Vector3d> getTrajPos(Eigen::MatrixXd ploy_coeff, std::vector<double> time);
};

PolynomialBase::~PolynomialBase()
{
}

std::vector<Eigen::Vector3d> PolynomialBase::getTrajPos(Eigen::MatrixXd ploy_coeff, std::vector<double> time)
{
    // ploy_coeff m*(n+1) X 3 参数
    Eigen::MatrixXd ploy_single_seg; // (n+1) X 3
    Eigen::MatrixXd p_time_vector;   // 1 X (n+1)
    Eigen::Vector3d pos;             // 3X1
    std::vector<Eigen::Vector3d> traj_poses;

    // std::cout << " --------- " << std::endl;
    for (int seg_num = 0; seg_num < time.size(); seg_num++)
    {
        // std::cout << " seg_num:  " << seg_num << " time: " << time[seg_num] << std::endl;
        // 获取当前分段的参数
        ploy_single_seg = ploy_coeff.block(single_param_num_ * seg_num, 0, single_param_num_, 3); //(n+1) X 3
        for (double t = 0; t < time[seg_num]; t += 0.1)
        {
            // std::cout << " t: " << t <<std::endl;
            p_time_vector = getPVec(t); // 1 X (n+1)
            pos = (p_time_vector * ploy_single_seg).transpose();
            traj_poses.push_back(pos);
        }
    }

    return traj_poses;
}

// template <class CORRIDOR>
void PolynomialBase::updateOptiParam(std::vector<Rectangle> corridors, Eigen::MatrixXd pos, Eigen::MatrixXd vel, Eigen::MatrixXd acc)
{

    //**********************************************************
    // create Q matrix
    // 即是求优化目标函数的海塞矩阵; 最终的Q matrix是分段Qi矩阵的组合 m为段数
    //
    //                | Q0               |
    //                |   Q1             |
    //            Q = |     ...          |
    //                |        ...       |
    //                |           Q(m-1) |

    //*********************************************************
    if (is_debug_)
        ROS_INFO_STREAM("[polyBase]: -------- create Q---------");
    m = corridors.size();
    Q = Eigen::MatrixXd::Zero(m * single_param_num_, m * single_param_num_);
    Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(single_param_num_, single_param_num_); // Qi [(n+1) * (n+1)]
    for (int i = 0; i < m; i++)
    {
        for (int r = k; r < Qi.rows(); r++)
        {
            for (int c = k; c < Qi.cols(); c++)
            {
                long double value = Qi_param_(r, c) * pow(corridors[i].t, c + r - 2 * k + 1);
                Qi(r, c) = value;
            }
        }

        Q.block(i * single_param_num_, i * single_param_num_, single_param_num_, single_param_num_) = Qi;
    }

    if (is_debug_)
    {
        ROS_INFO_STREAM("[polyBase]: -------- Q---------\n"
                        << Q);
        ROS_INFO_STREAM("[polyBase]: -- Q size: " << Q.rows() << " x " << Q.cols());
    }

    // ********************************************************
    // create A_equality_constraint, 共计(m+1)*3个等式约束
    // 等式约束,
    // 先存放初始位置和结束位置上的 p , v , a 等式关系; 共计 2 * 3 = 6个
    // 然后存放中间位置的 p , v , a 关系相等约束关系 共计 (m-1) * 3 个
    // A_equality_constraint * P = b_equality_constraint
    // A_([(m+1)*3] X [(n+1)*m])  P_([(n+1)*m] X 1) = b_([(m+1)*3] X 1)
    // ********************************************************
    A_equality_constraint = Eigen::MatrixXd::Zero((m + 1) * 3, m * single_param_num_); // 等式约束矩阵  (m+1)*3  * (n+1)*m

    for (size_t i = 0; i < A_equality_constraint.rows(); i++)
    {

        A_equality_constraint.row(0).block(0, 0, 1, single_param_num_) = getPVec(0);
        A_equality_constraint.row(1).block(0, 0, 1, single_param_num_) = getVVec(0);
        A_equality_constraint.row(2).block(0, 0, 1, single_param_num_) = getaVec(0);

        A_equality_constraint.row(3).block(0, (m - 1) * single_param_num_, 1, single_param_num_) = getPVec(corridors.back().t);
        A_equality_constraint.row(4).block(0, (m - 1) * single_param_num_, 1, single_param_num_) = getVVec(corridors.back().t);
        A_equality_constraint.row(5).block(0, (m - 1) * single_param_num_, 1, single_param_num_) = getaVec(corridors.back().t);

        if (i >= 6)
        {
            if (i % 3 == 0)
            {                                                                                                                                       // p
                A_equality_constraint.row(i).block(0, ((i - 6) / 3) * single_param_num_, 1, single_param_num_) = getPVec(corridors[(i - 6) / 3].t); // m段末尾
                A_equality_constraint.row(i).block(0, ((i - 6) / 3 + 1) * single_param_num_, 1, single_param_num_) = -1 * getPVec(0);               // m+1段首
            }
            else if (i % 3 == 1)
            {                                                                                                                                       // v
                A_equality_constraint.row(i).block(0, ((i - 6) / 3) * single_param_num_, 1, single_param_num_) = getVVec(corridors[(i - 6) / 3].t); // m段末尾
                A_equality_constraint.row(i).block(0, ((i - 6) / 3 + 1) * single_param_num_, 1, single_param_num_) = -1 * getVVec(0);               // m+1段首
            }
            else
            {                                                                                                                                       // a
                A_equality_constraint.row(i).block(0, ((i - 6) / 3) * single_param_num_, 1, single_param_num_) = getaVec(corridors[(i - 6) / 3].t); // m段末尾
                A_equality_constraint.row(i).block(0, ((i - 6) / 3 + 1) * single_param_num_, 1, single_param_num_) = -1 * getaVec(0);               // m+1段首
            }
        }
    }

    if (is_debug_)
    {
        ROS_INFO_STREAM("[polyBase]: -------- A_equality_constraint ---------\n"
                        << A_equality_constraint);
        ROS_INFO_STREAM("[polyBase]: -- A_equality_constraint size: " << A_equality_constraint.rows() << " x " << A_equality_constraint.cols());
    }

    //********************************************
    // create b_equality_constraint 等式约束边界矩阵
    //  A * P = b   b为(m+1)*3 x 3维 向量 这里的 X3是指xyz三个维度
    // 前6维为起始和终止位置的 p, v, a
    //********************************************

    b_equality_constraint = Eigen::MatrixXd::Zero((m + 1) * 3, 3);
    b_equality_constraint.row(0) = pos.row(0);
    b_equality_constraint.row(1) = vel.row(0);
    b_equality_constraint.row(2) = acc.row(0);
    b_equality_constraint.row(3) = pos.row(1);
    b_equality_constraint.row(4) = vel.row(1);
    b_equality_constraint.row(5) = acc.row(1);

    if (is_debug_)
    {
        ROS_INFO_STREAM("[polyBase]: -------- b_equality_constraint ---------\n"
                        << b_equality_constraint);
        ROS_INFO_STREAM("[polyBase]: -- b_equality_constraint size: " << b_equality_constraint.rows() << " x " << b_equality_constraint.cols());
    }

    //************************************************
    // 创建　A_inequality_constraint　不等式约束, 目前约束中间点的位置在corridor重叠区域内， 共计 (m-1) 个
    // b_i_lo　< A*P < b_i_up
    // A_inequality_constraint (m-1) X (n+1)*m
    //************************************************
    A_inequality_constraint = Eigen::MatrixXd::Zero(m - 1, (n + 1) * m);

    // 约束中间点的位置p
    for (size_t i = 0; i < A_inequality_constraint.rows(); i++)
    {
        A_inequality_constraint.row(i).block(0, i * single_param_num_, 1, single_param_num_) = getPVec(corridors[i].t); // m段末尾
    }

    if (is_debug_)
    {
        ROS_INFO_STREAM("[polyBase]: -------- A_inequality_constraint ---------\n"
                        << A_inequality_constraint);
        ROS_INFO_STREAM("[polyBase]: -- A_inequality_constraint size: " << A_inequality_constraint.rows() << " x " << A_inequality_constraint.cols());
    }

    //************************************************
    // 创建 b_inequality_constraint 不等式约束边界矩阵 (m-1) * 6
    // (x_lo , x_up, y_lo, y_up, z_lo, z_up)
    //
    //************************************************
    b_inequality_constraint = Eigen::MatrixXd::Zero(m - 1, 6);
    for (int i = 0; i < m - 1; i++)
    {
        double x_lo, x_up, y_lo, y_up, z_lo, z_up;
        x_lo = corridors[i].cross_rect->vertex(3, 0);
        x_up = corridors[i].cross_rect->vertex(2, 0);

        y_lo = corridors[i].cross_rect->vertex(3, 1);
        y_up = corridors[i].cross_rect->vertex(0, 1);

        // 二维z是没有用的
        z_lo = 0;
        z_up = 0.5;

        b_inequality_constraint.row(i) << x_lo, x_up, y_lo, y_up, z_lo, z_up;
    }

    if (is_debug_)
    {
        ROS_INFO_STREAM("[polyBase]: -------- b_inequality_constraint ---------\n"
                        << b_inequality_constraint);
        ROS_INFO_STREAM("[polyBase]: -- b_inequality_constraint size: " << b_inequality_constraint.rows() << " x " << b_inequality_constraint.cols());
    }
}
