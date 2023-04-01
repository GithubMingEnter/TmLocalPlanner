/**************************************************************************
 * path_search.cpp
 * 
 * @Author： bornchow
 * @Date: 2023.03.04
 * 
 * @Description:
 *  本程序为路径搜索算法的启动文件，提供以下功能：
 * 1. 选取搜索算法类型
 * 2. 支持算法可视化(rviz)
 * 3. 与rviz交互,包括获取地图信息与起始点信息
 *  
 *  ****************************************************/

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include <iostream>
#include <vector> 
#include <thread>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "vis/visualization.hpp"
#include "map/GridMap.hpp"

#include "path_search/JPS.hpp"
#include "path_search/Astar.hpp"
#include "path_search/HybridAstarSearch.hpp"
using namespace std;

class PathSearch
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber goal_sub_;

    bool use_a_star_, use_jps_,use_hybrid_;

    std::vector<Eigen::Vector3d> res_points_;
    std::shared_ptr<vis::Visualization> vis_ptr_;
    std::shared_ptr<env::GridMap> env_ptr_;

    float resolution_;
private:
    void goalCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){
        res_points_.emplace_back(msg->point.x, msg->point.y, msg->point.z);
    }


public:
    PathSearch(ros::NodeHandle nh, ros::NodeHandle pri_nh):
    nh_(pri_nh){

        nh_.param<bool>("use_a_star", use_a_star_, false);
        nh_.param<bool>("use_jps", use_jps_, false);
        nh_.param<bool>("use_hybrid", use_hybrid_, false);
        ROS_WARN_STREAM("[path search] | param use_a_star : " << use_a_star_);
        ROS_WARN_STREAM("[path search] | param use_JPS    : " << use_jps_);

        goal_sub_ = nh.subscribe("/clicked_point", 10, &PathSearch::goalCallBack, this);

        vis_ptr_ = make_shared<vis::Visualization>(nh_);
        env_ptr_ = make_shared<env::GridMap>(nh_);
        resolution_ = env_ptr_->getResolution();
    }
    ~PathSearch(){};

    void run(){
        // 获取起始点
        bool is_start_set, is_goal_set;
        is_start_set = is_goal_set = false;
        ros::Rate loop_rate(20);

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
            } // set start and goal point

            /******************************************************************************/
            // *  A star search method
            /******************************************************************************/                                                                           
            if(use_a_star_){
                shared_ptr<Astar> a_star_sreach = make_shared<Astar>(nh_, env_ptr_, vis_ptr_);
                vis_ptr_->registe<nav_msgs::Path>("a_star_final_path");
                bool a_star_res = a_star_sreach->search(res_points_[0], res_points_[1]);
                
                if(a_star_res){
                    std::vector<Eigen::Vector3d> final_path = a_star_sreach->getPathInWorld();
                    vis_ptr_->visPath("a_star_final_path", final_path);
                }
            }

            /******************************************************************************/
            // *  JPS method
            /******************************************************************************/ 
            if(use_jps_){
                shared_ptr<JPS> jps_search = make_shared<JPS>(nh_, env_ptr_, vis_ptr_);
                vis_ptr_->registe<nav_msgs::Path>("jps_final_path");
                bool jps_res = jps_search->search(res_points_[0], res_points_[1]);
                if(jps_res){
                    std::vector<Eigen::Vector3d> final_path = jps_search->getPathInWorld();
                    vis_ptr_->visPath("jps_final_path", final_path);
                }

            }
            // if(use_hybrid_){
            //     shared_ptr<HybridAstarSearch> hybrid_search=make_shared<
            // }




        } 


    } // void run()
};


///////////////////////////////////////////

int main(int argc, char** argv){

    ros::init(argc, argv, "path_search");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    ros::Rate rate(10);
    
    PathSearch path_search(nh, nh_);
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); 

    path_search.run();

}
