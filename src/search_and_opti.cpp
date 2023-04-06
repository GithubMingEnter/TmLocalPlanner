/**************************************************************************
 * search_and_opti.cpp
 * 
 * @Author： bornchow
 * @Date: 2022.06.18
 * 
 * @Description:
 * This is path search and trajectory optimization demo
 * YOU can:
 * 1. choose a path search algorhtm, both based on graph search or sampling method
 * 2. choose a trajectory type, Now support Polynomial trajectory, Bezier trajectory(come soon!!!)
 * 3. choose a hard-constraint trajectory optimization, mini-snap or mini-jeck
 *  
 *  ****************************************************/

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include <iostream>
#include <vector> 
#include <thread>
#include "path_search/Astar.hpp"
#include "vis/visualization.hpp"
#include "map/GridMap.hpp"
#include "traj_opti/CorridorGen2D.hpp"
#include "traj_opti/TrajOpti.hpp"

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

class SearchAndOpti
{
private:
    ros::NodeHandle n;
    ros::NodeHandle pri_nh;

    ros::Subscriber goal_sub_;

    // ros param
    int sreach_algorithm_;
    // for vis
    std::vector<Eigen::Vector3d> res_points_;
    float resolution_;
    visualization_msgs::Marker points_; 

    // shared_ptr<GraphGroupSearch> graph_sreach_;
    shared_ptr<Astar> a_star_sreach_;
    std::shared_ptr<vis::Visualization> vis_ptr_;
    std::shared_ptr<env::GridMap> env_ptr_;
private:
    void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg);
public:
    SearchAndOpti(ros::NodeHandle nh, ros::NodeHandle nh_);
    ~SearchAndOpti();
    void run();
    void visGraphSreachProcessThread();
};

SearchAndOpti::SearchAndOpti(ros::NodeHandle nh, ros::NodeHandle nh_):
n(nh),
pri_nh(nh_)
{
    // ros参数
    nh_.param<int>("sreach_algorithm", sreach_algorithm_, 0);
    goal_sub_ = n.subscribe("/clicked_point", 10, &SearchAndOpti::goalCallBack, this);
    res_points_.clear();
    //
    vis_ptr_ = make_shared<vis::Visualization>(nh_);
    env_ptr_ = make_shared<env::GridMap>(nh_);
    resolution_ = env_ptr_->getResolution();
    
    a_star_sreach_ = make_shared<Astar>(nh_, env_ptr_, vis_ptr_);
    vis_ptr_->registe<nav_msgs::Path>("a_star_final_path");

}

SearchAndOpti::~SearchAndOpti()
{
}



void SearchAndOpti::goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){
    res_points_.emplace_back(msg->point.x, msg->point.y, msg->point.z);
}

void SearchAndOpti::run(){

    // 获取起始点
    bool is_start_set, is_goal_set;
    is_start_set = is_goal_set = false;
    ros::Rate loop_rate(20);

    Eigen::Vector3d start_vel;
    Eigen::Vector3d start_acc;

    start_vel.setZero();
    start_acc.setZero();

    while (ros::ok()){

        // reset param
        res_points_.clear();
        is_start_set = is_goal_set = false;

        // get the start and end point
        while (res_points_.size()<3)
        {
            if(res_points_.size() == 0){
                ROS_INFO_ONCE("set {start point} in rviz");
            }

            if(res_points_.size() == 1){

                if(!is_start_set){

                    std::cout << "....." <<res_points_.back().transpose() << std::endl;
                    vis_ptr_->visualize2dPoint("start_point", res_points_.back(), 2*resolution_, vis::Color::green);
                    is_start_set = true;

                    ROS_INFO("set {goal point} in rviz");
                }
            }
            if(res_points_.size() == 2){
                
                if(!is_goal_set){
        
                    std::cout << "....." <<res_points_.back().transpose() << std::endl;
                    vis_ptr_->visualize2dPoint("goal_point", res_points_.back(), 2*resolution_, vis::Color::green);
                    is_goal_set = true;

                    ROS_INFO("clicked point to start!!!");
                }
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
        
        // A-star search and opti
        ros::Time a_star_search_time = ros::Time::now();
        // 1.fornt-end path search 
        bool a_star_res = a_star_sreach_->search(res_points_[0], res_points_[1]);
        if(a_star_res){
            std::vector<Eigen::Vector3d> final_path = a_star_sreach_->getPathInWorld();
            
            ros::Time time1 = ros::Time::now();
            vis_ptr_->visPath("a_star_final_path", final_path);
            ROS_WARN_STREAM("A star search time: | time1 --> " << (ros::Time::now() - time1).toSec() * 1000 << " (ms)");


            shared_ptr<CorridorGen2D> corridor_gen = make_shared<CorridorGen2D>(pri_nh, env_ptr_, vis_ptr_, start_vel);
            
            std::vector<Rectangle> corridor = corridor_gen->corridorGeneration(final_path);
            ROS_INFO_STREAM("corridor size: " << corridor.size());
            time1 = ros::Time::now();
            vis_ptr_->visCorridor("corridor", corridor, vis::Color::pink);
            ROS_WARN_STREAM("A star search time: | time2 --> " << (ros::Time::now() - time1).toSec() * 1000 << " (ms)");


            // traj opti
            shared_ptr<TrajOpti> traj_opti = make_shared<TrajOpti>(pri_nh);
            // pva of start and end point 
            Eigen::MatrixXd pos = Eigen::MatrixXd::Zero(2,3);
            Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2,3);
            Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2,3);
            pos.row(0) = final_path.front();
            pos.row(1) = final_path.back();
            vel.row(0) = start_vel;
            vel.row(1) << 0, 0, 0;
            acc.row(0) = start_acc;
            acc.row(1) << 0, 0, 0;

            // vis_ptr_->registe<nav_msgs::Path>("a_star_final_traj");
            std::vector<Eigen::Vector3d> final_traj = traj_opti->updateOptiParam(corridor, pos, vel, acc);
            // vis_ptr_->visPath("a_star_final_traj", final_traj);

            time1 = ros::Time::now();
            vis_ptr_->visualize2dPoints("a_star_final_traj", final_traj, env_ptr_->getResolution(), vis::Color::green, 1, "map", true);
            ROS_WARN_STREAM("A star search time: | time3 --> " << (ros::Time::now() - time1).toSec() * 1000 << " (ms)");

            ROS_WARN_STREAM("A star search time: | total --> " << (ros::Time::now() - a_star_search_time).toSec() * 1000 << " (ms)");
        }

    }
   
}


///////////////////////////////////////////

int main(int argc, char** argv){

    ros::init(argc, argv, "search_and_opti");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate(10);
    
    SearchAndOpti search_and_opti(nh, nh_);
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); 

    search_and_opti.run();

}