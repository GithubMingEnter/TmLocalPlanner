/**************************************************************************
 * TrajType.hpp
 * 
 * @Author： bornchow
 * @Date: 2023.02.26
 * 
 * @Description:
 * 
 * 这是一个空父类
 *  
 * 参考程序:
 *  ****************************************************/
#ifndef __TRAJ_TYPE_HPP__
#define __TRAJ_TYPE_HPP__

#include <iostream>
#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <traj_opti/CorridorGen2D.hpp>

class TrajType
{
private:
    /* data */
public:
    TrajType(/* args */);
    ~TrajType();

    virtual void updateOptiParam(std::vector<Rectangle> corridors, Eigen::MatrixXd pos, Eigen::MatrixXd vel, Eigen::MatrixXd acc){
        std::cout << " this is function <updateOptiParam> in base " << std::endl;
        return;
    }
    virtual Eigen::MatrixXd getQ(){ return Eigen::MatrixXd::Zero(1, 1); }
    virtual Eigen::MatrixXd getAEC(){ return Eigen::MatrixXd::Zero(1, 1); }
    virtual Eigen::MatrixXd getBEC(){ return Eigen::MatrixXd::Zero(1, 1); }
    virtual Eigen::MatrixXd getAIEC(){ return Eigen::MatrixXd::Zero(1, 1); }
    virtual Eigen::MatrixXd getBIEC(){ return Eigen::MatrixXd::Zero(1, 1); }
    virtual std::vector<Eigen::Vector3d> getTrajPos(Eigen::MatrixXd ploy_coeff, std::vector<double> time){ return {}; }
};

TrajType::TrajType(/* args */)
{
}

TrajType::~TrajType()
{
}



#endif // __TRAJ_TYPE_HPP__