/*

author: dominic

date: 2023.4.5
融合空间走廊与topp

*/

#ifndef BA380FFA_A832_4EE6_A6BB_8C0278097C3B
#define BA380FFA_A832_4EE6_A6BB_8C0278097C3B

#include "ros1_disp/scene_dis.hpp"
#include "ros1_disp/spline_opt_dis.hpp"
#include "ros1_disp/topp_dis.hpp"
#include "scene/scene_astar.hpp"
#include "topp/spline_opt.hpp"
#include "topp/conic_alm_topp.hpp"
#include "common.h"
#include <tf2/buffer_core.h>
#include "tmTypes.h"
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
#include "stanley_control.hpp"

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

bool is_print = false;
vecD gx, gy;
int sample_global = 1;
std::vector<gridNode *> grid_global_path;
std::vector<Eigen::Vector3d> pt_vec;
const std::string xy_path = "/home/ming/ros1_workspace/testws/src/hybrid_astar_planner/test_the_plugin/scripts/Aspoint/xy.txt";
bool loadGlobalPath(const std::string &roadmap_path,
                    const double target_speed)
{
  std::ifstream infile;
  infile.open(roadmap_path);
  if (!infile.is_open())
  {
    std::cout << ("can't open global path\n");
  }
  std::vector<std::pair<double, double>> xy_points;
  std::string s, x, y;
  int count = 0;
  while (getline(infile, s))
  {

    std::stringstream word(s);
    word >> x;
    word >> y;
    // 每隔20取一个点
    if (count % sample_global == 0)
    {
      double pt_x = std::atof(x.c_str());
      double pt_y = std::stod(y);
      gx.emplace_back(pt_x);
      gy.emplace_back(pt_y);

      //           std::cout<<count++<<std::endl;
      Eigen::Vector3d pt;
      pt << pt_x, pt_y, 0.0;
      pt_vec.emplace_back(pt);
      //          gridNode grid_temp(pt);
      //            gridNode* node_current = &grid_temp;
      // std::cout<<count<<std::endl;
      //           grid_global_path.emplace_back(node_current);
      // delete node_current;
    }
    // TODO 考虑终点
  }
  infile.close();
  return true;
}
class TmLocalPlanner
{
private:
  int sreach_algorithm_; // 0 A* 1 hybrid A*

  // map
  float resolution_;
  // planning
  double step_s_; // 离散点步长
  double c0_, c1_; //势力场参数
  double goal_tolerance_;
  double c_;//// 前视常数
  double lam_; // 前视距离系数
  double planning_frequency_;
  std::shared_ptr<cubicSplineOpt> cso;
  std::shared_ptr<conicALMTOPP2> topp2;
  Evx a, b, c, d; // topp点
  // control
  double control_frequency_;
  std::shared_ptr<mpc_car::MpcCar> mpcPtr_;
  std::shared_ptr<control::PIDController> speedPidControllerPtr_;
  std::shared_ptr<control::PIDController> headPidControllerPtr_;
  std::unique_ptr<control::StanleyController> stanleyPtr_;
  bool init = false;
  double delay_ = 0.0;
  bool nmpc_ = false;
  double speed_P, speed_I, speed_D, target_speed;
  double head_P, head_I, head_D;

  bool PhrAlm_ = true; // debug
  double all_time_;
  int times_ = 0;
  double dt_;
  // 车辆参数
  double wheelbase_length_;
  double v_min_, v_max_;
  double a_max_;
  VehicleState current_state_;
  Eigen::Vector4d state_;

  bool b_vehicle_state = false; // 获得odom
  bool b_traj_ = false;          // obtain trajectory
  bool b_global_=false;
  bool reach_goal_=false;
  bool is_track_=false;

  // ROS
  ros::NodeHandle pri_nh_;
  ros::NodeHandle nh_;
   nav_msgs::Path g_path_;
   nav_msgs::Path ref_global_path_;
  // Subscribers and Publishers
  ros::Subscriber odom_sub_, goal_sub_,global_path_sub_;
  ros::Subscriber obstacles_sub_;

  ros::Publisher current_pose_rviz_pub_, vis_car_pub;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher global_path_pub_;
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
  std::string odom_link_;

  geometry_msgs::PoseStamped goal_pose;
  std::vector<Eigen::Vector3d> startGoal;
  std::vector<Eigen::Vector3d> Ev3_path_;

  // shared_ptr<GraphGroupSearch> graph_sreach_;
  shared_ptr<Astar> a_star_search_;
  std::shared_ptr<vis::Visualization> vis_ptr_;
  std::shared_ptr<env::GridMap> env_ptr_;

  Emx tqv;
  Emx tqxy;
  int t = 0;
  bool b_global_path;
  bool b_temp;

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
  {
    ROS_INFO_STREAM_ONCE("ENTER odomCallback");

    geometry_msgs::TransformStamped transform_stamped;

    // tf::TransformListener transform_listener = new tf::TransformListener();
    try
    {
      // TODO
      transform_stamped = tf_buffer_.lookupTransform("map", odom_link_, ros::Time(0));
      //  ('map','base_link',ros::Time(0),ros::Duration(0.1));
    }
    catch (tf2::TransformException &e)
    {
      ROS_WARN("%s", e.what());

      return;
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = transform_stamped.transform.translation.x;
    pose.pose.position.y = transform_stamped.transform.translation.y;
    pose.pose.position.z = transform_stamped.transform.translation.z;
    pose.pose.orientation = transform_stamped.transform.rotation;

    // createCurrentPoseMarker(current_state_,pose);
    current_pose_rviz_pub_.publish(pose);
    car_m.pose.position = pose.pose.position;
    car_m.pose.orientation = pose.pose.orientation;
    car_m.id = car_id++;

    vis_car_pub.publish(car_m);

    geometry_msgs::PoseStamped pose_trans, pose_tran_after;
    pose_trans.header.frame_id = odom_msg->header.frame_id;
    pose_trans.header.stamp = odom_msg->header.stamp;
    pose_trans.pose = odom_msg->pose.pose;
    tf2::doTransform(pose_trans, pose_tran_after, transform_stamped);

    tf::Quaternion q(pose_tran_after.pose.orientation.x, pose_tran_after.pose.orientation.y,
                     pose_tran_after.pose.orientation.z, pose_tran_after.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_state_.yaw); // 获取整车的姿态角

    double x = odom_msg->pose.pose.position.x;
    double y = odom_msg->pose.pose.position.y;
    Eigen::Quaterniond qua(odom_msg->pose.pose.orientation.w,
                           odom_msg->pose.pose.orientation.x,
                           odom_msg->pose.pose.orientation.y,
                           odom_msg->pose.pose.orientation.z);
    Eigen::Vector3d euler = qua.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector2d v(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y);
    state_ << x, y, euler.z(), v.norm();

    current_state_.yaw = euler.z();
    current_state_.speed = odom_msg->twist.twist.linear.x;
    current_state_.x = x;
    current_state_.y = y;
    current_state_.steer = atan(wheelbase_length_ * odom_msg->twist.twist.angular.z / current_state_.speed);
    // ROS_INFO_STREAM("STATE: "<<state_.transpose());
    
    

    ROS_INFO_STREAM_ONCE("LEAVE odomCallback");
    b_vehicle_state = true; // 获取到当前车辆的状态
  };


  void pub_zero(){
    geometry_msgs::Twist twist_z;
    twist_z.linear.x=0;
    twist_z.angular.z=0;
    cmd_vel_pub_.publish(twist_z);
  }
  /********goalCallback********/
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    ROS_INFO_STREAM("enter goalCallback");
    reach_goal_=false; //重置
    b_traj_=false;//待优化
    goal_pose.header = msg->header;
    goal_pose.pose = msg->pose;
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                     msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    startGoal.emplace_back(msg->pose.position.x, msg->pose.position.y, yaw);
    ROS_INFO_STREAM("leave goalCallback");
  }

  void pathCallback(const nav_msgs::Path::ConstPtr &path_msg){
    ROS_INFO_STREAM("enter pathCallback");
    Ev3_path_.clear();//
    g_path_=*path_msg; //复制
   for(std::vector<geometry_msgs::PoseStamped>::const_iterator it=g_path_.poses.begin();it!=g_path_.poses.end();++it){
    // TODO YAW角不对
    tf::Quaternion q(it->pose.orientation.x, it->pose.orientation.y,
                     it->pose.orientation.z, it->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    Ev3_path_.emplace_back(it->pose.position.x,it->pose.position.y,yaw);
    
   }
   b_global_=true;
   b_traj_=false;//未优化
    ROS_INFO_STREAM("leave pathCallback");

  }
  
  void mainTimerCallback(const ros::TimerEvent &timer_event)
  {
    ROS_INFO_ONCE("enter mainTimeCallback");
  }

  // 两点之间的距离
  double PointDistanceSquare(const VehicleState &point, const double x,
                             const double y)
  {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
  }
  int QueryNearestPointByPosition(const VehicleState &cur_state)
  {
    double d_min = PointDistanceSquare(cur_state, topp2->q(0, 0), topp2->q(0, 1));
    int index_min = 0;
    for (size_t i = 1; i < topp2->q.rows(); ++i)
    {

      double d_temp = PointDistanceSquare(cur_state, topp2->q(i, 0), topp2->q(i, 1));
      if (d_temp < d_min)
      {
        d_min = d_temp;
        index_min = i;
      }
    }
    return index_min;
  }
  int QueryNearestPointByPosition(const VehicleState &cur_state, vector<double>& dists)
  {
    double d_min = PointDistanceSquare(cur_state, topp2->q(0, 0), topp2->q(0, 1));
    dists.emplace_back(d_min);
    int index_min = 0;
    for (size_t i = 1; i < topp2->q.rows(); ++i)
    {

      double d_temp = PointDistanceSquare(cur_state, topp2->q(i, 0), topp2->q(i, 1));
      dists.emplace_back(d_temp);
      if (d_temp < d_min)
      {
        d_min = d_temp;
        index_min = i;
      }
    }
    return index_min;
  }
  double speed_pid_control(double targetSpeed)
  {
    double ego_speed = current_state_.speed;
    
    double v_err = targetSpeed - ego_speed; // 速度误差
    ROS_INFO_STREAM(  "v_err: " << v_err << " targetSpeed_ is " << targetSpeed);
    double speed_cmd =
        speedPidControllerPtr_->Control(v_err, 1 / control_frequency_);
    return speed_cmd;
  };
  /**/
  double head_pid_control(double target_head)
  {
    const double ego_head = current_state_.yaw;
    double head_err = NormalizeAngle(target_head - ego_head); // 角度误差
    infoD("target_head: ",target_head);
    infoD("head_err is ",head_err);
    double angluar_cmd =
        headPidControllerPtr_->Control(head_err, 1 / control_frequency_);
    return angluar_cmd;
  };

  void controlTimerCallback(const ros::TimerEvent &timer_event);

public:
  void run();

  TmLocalPlanner(ros::NodeHandle nh, ros::NodeHandle pri_nh) : tf_listener_(tf_buffer_),
                                                               nh_(nh),
                                                               pri_nh_(pri_nh)
  {
    pri_nh_.param<int>("sreach_algorithm", sreach_algorithm_, 0);
    std::string odom_topic_,global_path_topic_;
    std::string cmd_vel_topic_;
    std::string current_pose_rviz_topic_;
    int obs_num_ = 6;
    pri_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
    pri_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    pri_nh_.param<std::string>("current_pose_rviz_topic", current_pose_rviz_topic_, "current_pose_rviz");
    pri_nh_.param<std::string>("global_path_topic", global_path_topic_, "/move_base/HybridAStarPlanner/plan");
    pri_nh_.param<std::string>("odom_link", odom_link_, "/odom");
  
    pri_nh_.param<double>("wheelbase_length", wheelbase_length_, 0.15);
    pri_nh_.param<double>("v_max", v_max_, 5.0);
    pri_nh_.param<double>("v_min", v_min_, 0.0);
    pri_nh_.param<double>("a_max", a_max_, 1.0);
    pri_nh_.param<double>("planning_frequency", planning_frequency_, 5);
    pri_nh_.param<double>("control_frequency", control_frequency_, 50);
    pri_nh_.param<int>("obs_num", obs_num_, 10);
    pri_nh_.param<double>("step_s", step_s_, 0.45);
    pri_nh_.param<double>("c0", c0_, 5);
    pri_nh_.param<double>("c1", c1_, -18);
    pri_nh_.param<double>("speed_P", speed_P, 1.0); // 读取PID参数
    pri_nh_.param<double>("speed_I", speed_I, 0.1);
    pri_nh_.param<double>("speed_D", speed_D, 0.0);

    pri_nh_.param<double>("head_P", head_P, 1.0); // 读取PID参数
    pri_nh_.param<double>("head_I", head_I, 0.1);
    pri_nh_.param<double>("head_D", head_D, 0.0);
    pri_nh_.param<bool>("is_track",is_track_,true);
    pri_nh_.param<double>("goal_tolerance",goal_tolerance_,0.3);
    pri_nh_.param<double>("c", c_, 0.1);
    pri_nh_.param<double>("lam", lam_, 0.1);

    pri_nh_.getParam("dt", dt_);
    pri_nh_.getParam("delay", delay_);
    pri_nh_.getParam("nmpc", nmpc_);
    pri_nh_.getParam("PhrAlm", PhrAlm_);

    ROS_WARN_STREAM("v_max_ = " << v_max_);
    ROS_WARN_STREAM("obs_num_ = " << obs_num_);
    ROS_WARN_STREAM("delay_ = " << delay_);
    // ROS Multi thread
    ros::AsyncSpinner spinner(4);
    spinner.start();
    vis_ptr_ = make_shared<vis::Visualization>(nh_);
    env_ptr_ = make_shared<env::GridMap>(nh_);
    speedPidControllerPtr_ = std::shared_ptr<control::PIDController>(new control::PIDController(speed_P, speed_I, speed_D));
    headPidControllerPtr_ = std::shared_ptr<control::PIDController>(new control::PIDController(head_P, head_I, head_D));
    stanleyPtr_ = std::unique_ptr<control::StanleyController>(new control::StanleyController(20)); // TODO限制角度
    stanleyPtr_->LoadControlConf();                                                                // TODO 参数调整
    resolution_ = env_ptr_->getResolution();

    a_star_search_ = make_shared<Astar>(nh_, env_ptr_, vis_ptr_);
    vis_ptr_->registe<nav_msgs::Path>("a_star_final_path");

    current_pose_rviz_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(current_pose_rviz_topic_, 1, true);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &TmLocalPlanner::goalCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 1, &TmLocalPlanner::odomCallback, this);
    global_path_sub_ = nh_.subscribe(global_path_topic_, 1, &TmLocalPlanner::pathCallback, this);
    vis_timer_ = nh_.createTimer(ros::Duration(1.0 / planning_frequency_), &TmLocalPlanner::mainTimerCallback, this);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_), &TmLocalPlanner::controlTimerCallback, this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    vis_car_pub = nh_.advertise<visualization_msgs::Marker>("vis_car", 10);
    global_path_pub_=nh_.advertise<nav_msgs::Path>("/topp_path/traj",1000);

    car_m.header.frame_id = "map";
    car_m.header.stamp = ros::Time::now();
    car_m.ns = "car_m";
    car_m.action = visualization_msgs::Marker::ADD;
    car_m.type = visualization_msgs::Marker::CUBE;
    car_m.pose.orientation.w = 1.0;
    car_m.scale.x = 1.0;
    car_m.scale.y = 0.5;
    car_m.scale.z = 0.3;
    car_m.color.r = car_m.color.g = car_m.color.b = car_m.color.a = 0.8;
    car_id = 0;

    dt_ = 1 / control_frequency_; // 离散时间
  }

};

#endif /* BA380FFA_A832_4EE6_A6BB_8C0278097C3B */
