/*

author: dominic 

date: 2023.4.5
融合空间走廊与topp

*/


#ifndef A60F0D12_9292_455C_BC07_FF49C0960FB1
#define A60F0D12_9292_455C_BC07_FF49C0960FB1

#include "ros1_disp/scene_dis.hpp"
#include "ros1_disp/spline_opt_dis.hpp"
#include "ros1_disp/topp_dis.hpp"
#include "scene/scene_astar.hpp"
#include "topp/spline_opt.hpp"
#include "topp/conic_alm_topp.hpp"
#include "tmTypes.h"
#include "common.h"
#include <tf2/buffer_core.h>

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
#include <mpc_car/mpc_car.hpp>
#include "pid_controller.hpp"

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

bool is_print=false;
vecD gx,gy;
int sample_global=1;
std::vector<gridNode*> grid_global_path;
std::vector<Eigen::Vector3d> pt_vec;
const std::string xy_path="/home/ming/ros1_workspace/testws/src/hybrid_astar_planner/test_the_plugin/scripts/Aspoint/xy.txt";
bool loadGlobalPath(const std::string &roadmap_path,
                    const double target_speed){
    std::ifstream infile;
    infile.open(roadmap_path);
    if(!infile.is_open()){
        std::cout<<("can't open global path\n");
    }
    std::vector<std::pair<double,double>> xy_points;
    std::string s,x,y;
    int count=0;
    while(getline(infile,s)){
      
        std::stringstream word(s);
        word>>x;word>>y;
        //每隔20取一个点
        if(count%sample_global==0)
        {
          double pt_x=std::atof(x.c_str());
          double pt_y=std::stod(y);
          gx.emplace_back(pt_x);
          gy.emplace_back(pt_y);
         
//           std::cout<<count++<<std::endl;
          Eigen::Vector3d pt;
          pt<<pt_x,pt_y,0.0;
          pt_vec.emplace_back(pt);
//          gridNode grid_temp(pt);
//            gridNode* node_current = &grid_temp;
// std::cout<<count<<std::endl;
//           grid_global_path.emplace_back(node_current);
          // delete node_current;
        }
        //TODO 考虑终点

    }
    infile.close();
    return true;

}
class TmLocalPlanner{
private:
    int sreach_algorithm_; //0 A* 1 hybrid A*

    //map
    float resolution_;
    //planning 
    double step_s_;//离散点步长
    double c0_,c1_;
    double planning_frequency_;
    // control
    double  control_frequency_;
    std::shared_ptr<mpc_car::MpcCar> mpcPtr_;
    bool init = false;
    double delay_ = 0.0;
    bool nmpc_ = false;
    double speed_P, speed_I, speed_D, target_speed;

    bool PhrAlm_ = true; //debug
    double all_time_;
    int times_=0;
    double dt_ ;
    //车辆参数
    double wheelbase_length_;
    double v_min_,v_max_;
    double a_max_;
    VehicleState current_state_;   
    Eigen::Vector4d  state_;  
      
    bool b_vehicle_state;//获得odom
    bool b_traj;//obtain trajectory

  // ROS
    ros::NodeHandle pri_nh_;
    ros::NodeHandle nh_;
  // Subscribers and Publishers
    ros::Subscriber odom_sub_,goal_sub_;
    ros::Subscriber obstacles_sub_;  
    
    ros::Publisher current_pose_rviz_pub_,vis_car_pub;
    ros::Publisher cmd_vel_pub_;
    // display
    visualization_msgs::Marker car_m;

    visualization_msgs::Marker points_; 
    int car_id;
    // Timer
    ros::Timer vis_timer_;
    ros::Timer control_timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
   
    geometry_msgs::PoseStamped goal_pose; 
    std::vector<Eigen::Vector3d> startGoal;

    // shared_ptr<GraphGroupSearch> graph_sreach_;
    shared_ptr<Astar> a_star_search_;
    std::shared_ptr<vis::Visualization> vis_ptr_;
    std::shared_ptr<env::GridMap> env_ptr_;
    std::shared_ptr<control::PIDController> speedPidControllerPtr_;

    Emx tqv;
    Emx tqxy;
    int t=0;
    bool b_global_path;
private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    ROS_INFO_STREAM_ONCE("ENTER odomCallback");
    
    geometry_msgs::TransformStamped transform_stamped;
    
    // tf::TransformListener transform_listener = new tf::TransformListener();
    try{
        //TODO
       transform_stamped = tf_buffer_.lookupTransform("map","odom",ros::Time(0));
      //  ('map','base_link',ros::Time(0),ros::Duration(0.1));
       
    }
    catch(tf2::TransformException& e){
        ROS_WARN("%s",e.what());

        return ;
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x=transform_stamped.transform.translation.x;
    pose.pose.position.y=transform_stamped.transform.translation.y;
    pose.pose.position.z=transform_stamped.transform.translation.z;
    pose.pose.orientation=transform_stamped.transform.rotation;
    
    // createCurrentPoseMarker(current_state_,pose);
    current_pose_rviz_pub_.publish(pose);
    car_m.pose.position=pose.pose.position;
    car_m.pose.orientation=pose.pose.orientation;
    car_m.id=car_id++;

    vis_car_pub.publish(car_m);

    geometry_msgs::PoseStamped pose_trans,pose_tran_after;
    pose_trans.header.frame_id=odom_msg->header.frame_id;
    pose_trans.header.stamp=odom_msg->header.stamp;
    pose_trans.pose=odom_msg->pose.pose;
    tf2::doTransform(pose_trans,pose_tran_after,transform_stamped);

    tf::Quaternion q(pose_tran_after.pose.orientation.x,pose_tran_after.pose.orientation.y,
                    pose_tran_after.pose.orientation.z,pose_tran_after.pose.orientation.w );
    tf::Matrix3x3 m(q);
    double roll,pitch;
    m.getRPY(roll,pitch,current_state_.yaw);//获取整车的姿态角
    
    double x = odom_msg->pose.pose.position.x;
    double y = odom_msg->pose.pose.position.y;
    Eigen::Quaterniond qua(odom_msg->pose.pose.orientation.w,
                         odom_msg->pose.pose.orientation.x,
                         odom_msg->pose.pose.orientation.y,
                         odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d euler = qua.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector2d v(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y);
    state_ << x, y, euler.z(), v.norm();

    current_state_.yaw=euler.z();
    current_state_.speed = odom_msg->twist.twist.linear.x;
    current_state_.x=x;
    current_state_.y=y;
    current_state_.steer = atan(wheelbase_length_*odom_msg->twist.twist.angular.z/current_state_.speed);
    ROS_INFO_STREAM("STATE: "<<state_.transpose());
    
    ROS_INFO_STREAM_ONCE("LEAVE odomCallback");
    b_vehicle_state=true;//获取到当前车辆的状态
  };
/********goalCallback********/
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    ROS_INFO_STREAM("enter goalCallback");
    goal_pose.header=msg->header;
    goal_pose.pose=msg->pose;
    tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,
                      msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    startGoal.emplace_back(msg->pose.position.x,msg->pose.position.y,yaw);
    ROS_INFO_STREAM("leave goalCallback");

  }
  
  void mainTimerCallback(const ros::TimerEvent& timer_event){
    ROS_INFO_ONCE("enter mainTimeCallback");    

      
  }

// 两点之间的距离
double PointDistanceSquare(const VehicleState & point, const double x,
                           const double y) {
  double dx = point.x - x;
  double dy = point.y - y;
  return dx * dx + dy * dy;
}
int QueryNearestPointByPosition(const double x,const double y){
  double d_min=PointDistanceSquare(current_state_,x,y);
  int index_min=0;

}
  void controlTimerCallback(const ros::TimerEvent& timer_event);
public:
  void run();

  TmLocalPlanner(ros::NodeHandle nh, ros::NodeHandle   pri_nh):
    tf_listener_(tf_buffer_),
    nh_(nh),
    pri_nh_(pri_nh)
  {
    pri_nh_.param<int>("sreach_algorithm", sreach_algorithm_, 0);
    std::string odom_topic_;
    std::string cmd_vel_topic_;
    std::string current_pose_rviz_topic_;
    int obs_num_=6;
    pri_nh_.param<std::string>("odom_topic",odom_topic_,"/odom");
    pri_nh_.param<std::string>("cmd_vel_topic",cmd_vel_topic_,"/cmd_vel");
    pri_nh_.param<std::string>("current_pose_rviz_topic",current_pose_rviz_topic_,"current_pose_rviz");

    pri_nh_.param<double>("wheelbase_length",wheelbase_length_,0.15);
    pri_nh_.param<double>("v_max",v_max_,12.0);
    pri_nh_.param<double>("v_min",v_min_,0.0);
    pri_nh_.param<double>("a_max",a_max_,0.5);
    pri_nh_.param<double>("planning_frequency",planning_frequency_,5);
    pri_nh_.param<double>("control_frequency",control_frequency_,50);
    pri_nh_.param<int>("obs_num",obs_num_,10);
    pri_nh_.param<double>("step_s",step_s_,0.45);
    pri_nh_.param<double>("c0",c0_,5);
    pri_nh_.param<double>("c1",c1_,-18);
    pri_nh_.param<double>("speed_P", speed_P, 0.4);                //读取PID参数
    pri_nh_.param<double>("speed_I", speed_I, 0.1);
    pri_nh_.param<double>("speed_D", speed_D, 0.0);
    pri_nh_.getParam("dt", dt_);
    pri_nh_.getParam("delay", delay_);
    pri_nh_.getParam("nmpc", nmpc_);
    pri_nh_.getParam("PhrAlm", PhrAlm_);

    ROS_WARN_STREAM("v_max_ = "<<v_max_);
    ROS_WARN_STREAM("obs_num_ = "<<obs_num_);
    ROS_WARN_STREAM("delay_ = "<<delay_);
    vis_ptr_ = make_shared<vis::Visualization>(nh_);
    env_ptr_ = make_shared<env::GridMap>(nh_);
    speedPidControllerPtr_ = std::shared_ptr<control::PIDController>(new control::PIDController(speed_P, speed_I, speed_D));
    
    resolution_ = env_ptr_->getResolution();
    
    a_star_search_ = make_shared<Astar>(nh_, env_ptr_, vis_ptr_);
    vis_ptr_->registe<nav_msgs::Path>("a_star_final_path");


    current_pose_rviz_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(current_pose_rviz_topic_, 1, true);
    goal_sub_=nh_.subscribe("/move_base_simple/goal",10,&TmLocalPlanner::goalCallback,this);
    odom_sub_ = nh_.subscribe(odom_topic_,1,&TmLocalPlanner::odomCallback,this);
    vis_timer_=nh_.createTimer(ros::Duration(1.0/planning_frequency_),&TmLocalPlanner::mainTimerCallback,this);
    control_timer_=nh_.createTimer(ros::Duration(1.0/control_frequency_),&TmLocalPlanner::controlTimerCallback,this);
    
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    vis_car_pub = nh_.advertise<visualization_msgs::Marker>("vis_car", 10);
    
    car_m.header.frame_id = "map";
    car_m.header.stamp = ros::Time::now();
    car_m.ns = "car_m";
    car_m.action = visualization_msgs::Marker::ADD;
    car_m.type = visualization_msgs::Marker::CUBE;
    car_m.pose.orientation.w = 1.0;
    car_m.scale.x = 1.0;
    car_m.scale.y=0.5;
    car_m.scale.z=0.3;
    car_m.color.r = car_m.color.g = car_m.color.b = car_m.color.a = 0.8;
    car_id=0;

    dt_=1/control_frequency_; //离散时间

    
  }
};

#endif /* A60F0D12_9292_455C_BC07_FF49C0960FB1 */
