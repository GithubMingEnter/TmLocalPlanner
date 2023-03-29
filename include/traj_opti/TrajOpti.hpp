/**************************************************************************
 * TrajOpti.hpp
 * 
 * @Author： bornchow
 * @Date: 2023.02.16
 * 
 * @Description:
 * 轨迹优化器
 * 
 * 二次规划求解器：
 *  目前支持 QPOASES
 * 
 * 轨迹表征方式：
 *  目前支持 多项式轨迹
 *  
 * 参考程序:
 *  ****************************************************/
#include <iostream>
#include <ros/ros.h>
#include <traj_opti/CorridorGen2D.hpp>
#include <qpOASES.hpp>
#include "traj_opti/PolynomialBase.hpp"
#include "traj_opti/BezierBase.hpp"
#include "traj_opti/TrajType.hpp"

USING_NAMESPACE_QPOASES

class TrajOpti
{
private:
    ros::NodeHandle nh_;

    bool is_debug_;
    int traj_type_;

    shared_ptr<TrajType> trajectory_;
public:
    TrajOpti(ros::NodeHandle nh);

    template <class CORRIDOR>
    std::vector<Eigen::Vector3d> updateOptiParam(std::vector<CORRIDOR> corridors, Eigen::MatrixXd pos, Eigen::MatrixXd vel, Eigen::MatrixXd acc);

    ~TrajOpti();
};

TrajOpti::TrajOpti(ros::NodeHandle nh) :
nh_(nh)
{
    ROS_INFO("\033[1;32m ----> this is trajectory optimizer !! .\033[0m");
    nh.param<bool>("traj_opti/is_debug", is_debug_, false);
    nh.param<int>("traj_opti/traj_type", traj_type_, 0);
    ROS_WARN_STREAM("[traj opti] param | is_debug  : " << is_debug_);
    ROS_WARN_STREAM("[traj opti] param | traj_type : " << traj_type_);

    switch (traj_type_)
    {
        case 0:{
            trajectory_ = make_shared<PolynomialBase>(nh);
            break;
        }
        case 1:{
            trajectory_ = make_shared<BezierBase>(nh);
            break;
        }

        default:{
            ROS_ERROR_STREAM("NOW only support trajectory type: \n \
            0 : Polynomial \n \
            1 : BezierBase \n \
            BUT got traj_type: "<< traj_type_);
        }
        
    }
    

}


TrajOpti::~TrajOpti()
{

}


template <class CORRIDOR>
std::vector<Eigen::Vector3d> TrajOpti::updateOptiParam(std::vector<CORRIDOR> corridors, Eigen::MatrixXd pos, Eigen::MatrixXd vel, Eigen::MatrixXd acc){

    ros::Time opti_start_time = ros::Time::now();

    trajectory_->updateOptiParam(corridors, pos, vel, acc);

    // ROS_ERROR_STREAM("test");

    Eigen::MatrixXd Q = trajectory_->getQ();
    Eigen::MatrixXd A_equality_constraint = trajectory_->getAEC();
    Eigen::MatrixXd b_equality_constraint = trajectory_->getBEC();
    Eigen::MatrixXd A_inequality_constraint = trajectory_->getAIEC();
    Eigen::MatrixXd b_inequality_constraint = trajectory_->getBIEC();

    
    //************************************************
    // 创建QProblem类的实例 
    // QProblem( int_t nV, int_t nC );  
    // 变量数nV   约束数nC
    //************************************************

    // 变量数nV = A_equality_constraint.cols()
    // 约束数nC = A_equality_constraint.rows() + A_inequality_constraint.rows()

    int nV = A_equality_constraint.cols();
    int nC = A_equality_constraint.rows()+A_inequality_constraint.rows();
    Eigen::MatrixXd result_P = Eigen::MatrixXd::Zero(nV, 3);

    QProblem qp_solver(nV, nC, qpOASES::HST_SEMIDEF);

    if(!is_debug_){
        qp_solver.setPrintLevel(qpOASES::PL_LOW); // PL_NONE: no output at all,
    }

    int kNumOfMatrixElements = Q.cols()*Q.rows();
    double h_matrix[kNumOfMatrixElements];
    int index = 0;
    for (int r = 0; r < Q.rows(); r++)
    {
        for (int c = 0; c < Q.cols(); c++)
        {
            h_matrix[index] = Q(r,c);
            index++;
        } 
    }

    // 等式约束与不等式约束组合在一起
    // 等式约束 AP = b -->  b <= AP <= b
    // 不等式约束  lo <= AP <= up
    // lo <= AP <= up 
    double Affine_constraint_matrix[nV*nC];  // NOLINT 大小为参数个数乘以约束个数
    double constraint_lower_bound[nC];                // NOLINT 大小为约束条件个数
    double constraint_upper_bound[nC];                // NOLINT 大小为约束条件个数

    index = 0;
    for (int r = 0; r < A_equality_constraint.rows(); r++)
    {
        for (int c = 0; c < A_equality_constraint.cols(); c++)
        {
            Affine_constraint_matrix[index] = A_equality_constraint(r,c);
            index++;
        } 
    }


    for (int r = 0; r < A_inequality_constraint.rows(); r++)
    {
        for (int c = 0; c < A_inequality_constraint.cols(); c++)
        {
            Affine_constraint_matrix[index] = A_inequality_constraint(r,c);
            index++;
        }
        
    }

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(nV, 1);
    
    double* g_matrix = G.data();
    double* lb = nullptr;
    double* ub = nullptr;


    for(int xyz=0; xyz<2; xyz++){
        if(is_debug_) ROS_INFO_STREAM("[traj opti] slove :" << xyz << " [0 - x , 1 - y, 2 - z]");
        index = 0;
        for (int r = 0; r < b_equality_constraint.rows(); ++r) {
            constraint_lower_bound[r] = b_equality_constraint(r, xyz); // 上下限相同
            constraint_upper_bound[r] = b_equality_constraint(r, xyz);
            index++;
        }

        for (int r = 0; r < b_inequality_constraint.rows(); ++r)
        {
            constraint_lower_bound[r+index] = b_inequality_constraint(r, xyz*2);
            constraint_upper_bound[r+index] = b_inequality_constraint(r, xyz*2+1);
        }


        int_t nWSR = 1000;

        returnValue ret;

        if (xyz == 0) //初始化
        {
            ret = qp_solver.init(h_matrix, g_matrix, Affine_constraint_matrix, lb, ub, constraint_lower_bound, constraint_upper_bound, nWSR);
        }else // 热启动
        {
            ret = qp_solver.hotstart(g_matrix, lb, ub, constraint_lower_bound, constraint_upper_bound, nWSR);
        }

        
        if (ret != qpOASES::SUCCESSFUL_RETURN)
        {
            ROS_WARN_STREAM("[traj opti] qpOASES : slover failed !!!!!");            
        }else
        {
            double result[nV];  // NOLINT
            memset(result, 0, sizeof result);      //全为0
            qp_solver.getPrimalSolution(result);  //获取结果

            // cout << "result " << endl;
            for (size_t i = 0; i < nV; i++)
            {
                result_P(i, xyz) = result[i];
                // cout << result[i] << " ";
            }
            // cout << endl;
        } 

    }

    if(is_debug_) ROS_INFO_STREAM("------result: P---------\n" << result_P);

    std::vector<double> times;


    for(int i=0; i<corridors.size(); i++){
        times.push_back(corridors[i].t);

    }


    std::vector<Eigen::Vector3d> res_poses = trajectory_->getTrajPos(result_P, times);

    ROS_INFO_STREAM("\033[1;32m[traj opti] | traj size      : " << res_poses.size() << " \033[0m");
    ROS_INFO_STREAM("\033[1;32m[traj opti] | traj opti time : " << (ros::Time::now() - opti_start_time).toSec() * 1000 << " (ms) \033[0m");

    return res_poses;

}
