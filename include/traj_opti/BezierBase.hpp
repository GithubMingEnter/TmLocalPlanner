/**************************************************************************
 * BezierBase.hpp
 * 
 * @Author： bornchow
 * @Date: 2023.02.26
 * 
 * @Description:
 * Bezier curve trajectory 构造二次规划形式
 * 
 *  
 * 参考程序:
 *  ****************************************************/
# pragma once
# include <traj_opti/PolynomialBase.hpp>
# include "traj_opti/TrajType.hpp"

class BezierBase : public TrajType
{
private:
    ros::NodeHandle nh_;
    int n;
    int k;
    int m;
    bool is_debug_;

    shared_ptr<PolynomialBase> poly_;

public:
    BezierBase(ros::NodeHandle nh);

    template <class CORRIDOR>
    void updateOptiParam(std::vector<CORRIDOR> corridors, Eigen::MatrixXd pos, Eigen::MatrixXd vel, Eigen::MatrixXd acc);

    ~BezierBase();
};

BezierBase::BezierBase(ros::NodeHandle nh):
nh_(nh){
    
    ROS_INFO("\033[1;32m ----> This is Bezier trajectory !! .\033[0m");
    nh_.param<int>("traj_opti/order", n, 5);
    nh_.param<int>("traj_opti/min_order", k, 4);
    nh_.param<bool>("traj_opti/is_debug", is_debug_, false);


    ROS_WARN_STREAM("[Bezier] param | order     : " << n);
    ROS_WARN_STREAM("[Bezier] param | min_order : " << k);
    ROS_WARN_STREAM("[Bezier] param | is_debug  : " << is_debug_);

    poly_ = make_shared<PolynomialBase>(n, k, false);

}

BezierBase::~BezierBase()
{
}

template <class CORRIDOR>
void BezierBase::updateOptiParam(std::vector<CORRIDOR> corridors, Eigen::MatrixXd pos, Eigen::MatrixXd vel, Eigen::MatrixXd acc){

   
    if(is_debug_) ROS_INFO_STREAM("[BezierBase]: -------- create Q---------");
    poly_->updateOptiParam(corridors, pos, vel, acc);

}
