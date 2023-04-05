#ifndef MAIN_HPP
#define MAIN_HPP

#include "ros1_disp/scene_dis.hpp"
#include "ros1_disp/spline_opt_dis.hpp"
#include "ros1_disp/topp_dis.hpp"
#include "scene/scene_astar.hpp"
#include "topp/spline_opt.hpp"
#include "topp/conic_alm_topp.hpp"
#include "tmTypes.h"
#include "common.h"
#include <tf2/buffer_core.h>
vecD gx,gy;
int sample_global=20;
std::vector<gridNode*> grid_global_path;
std::vector<Eigen::Vector2d> pt_vec;
bool is_print=false;

class TmLocalPlanner{
public:

  TmLocalPlanner():tf_listener_(tf_buffer_){

        ros::NodeHandle private_nh("~");
        std::string odom_topic_;
        std::string cmd_vel_topic_;
        std::string current_pose_rviz_topic_;
        int obs_num_=6;
        private_nh.param<std::string>("odom_topic",odom_topic_,"/odom");
        private_nh.param<std::string>("cmd_vel_topic",cmd_vel_topic_,"/cmd_vel");
        private_nh.param<std::string>("current_pose_rviz_topic",current_pose_rviz_topic_,"current_pose_rviz");

        private_nh.param<double>("wheelbase_length",wheelbase_length_,1.0);
        private_nh.param<double>("v_max",v_max_,12.0);
        private_nh.param<double>("v_min",v_min_,0.0);
        private_nh.param<double>("a_max",a_max_,0.5);
        private_nh.param<double>("planning_frequency",planning_frequency_,5);
        private_nh.param<int>("obs_num",obs_num_,10);

        ROS_WARN_STREAM("v_max_ = "<<v_max_);
        ROS_WARN_STREAM("obs_num_ = "<<obs_num_);
        
        // ros::Subscriber  odom_sub = nh_.subscribe(odom_topic_, 1,&TmlocalPlanner::odomCallback,this);
        // odom_sub_ = nh_.subscribe(odom_topic_,1,&TmLocalPlanner::odomCallback,this);
        current_pose_rviz_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(current_pose_rviz_topic_, 1, true);
        goal_sub_=nh_.subscribe("/move_base_simple/goal",1,&TmLocalPlanner::goalCallback,this);
        timer_=nh_.createTimer(ros::Duration(1.0/planning_frequency_),&TmLocalPlanner::mainTimerCallback,this);
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
    // initialize a scene
    //   // initialize a scene x0,y0,yaw0 lx,ly,ld,obs_d_min _obs_d_max _p_min _p_max
  //   //区域100*50 与障碍物间隔距离最大最小 , 障碍物的大小
    sceneAstar sa(0.,0., 0., 45., 20., 0.5, 1.6, 3, 3, 12);
    std::cout << "[[[ scene generation OK!]]]" << std::endl;
    sa.random_obstacles(obs_num_);
    std::cout << "[[[ obstacles generation OK!]]]" << std::endl;
    // std:: cout <<"{{{"<<sa.is_free(0.0, 0.0) <<std::endl;;
    rviz1DisSimp disp_rviz(nh_, "topp_array", "map", "topp_test");
    sceneAstarDisp disp_sa(disp_rviz, sa);
    disp_sa.add_obstacles();// draw out the scene
    std::cout << "[[[ disp_sa add obstacles OK!]]]" << std::endl;
    // disp_rviz.send();
    
      ros::Rate rate(1);
    while(ros::ok()){
      if(startGoal.size()>=2){
        ROS_INFO_STREAM("START PLANNER");
        startGoal.clear();
        sa.set_start(2,2);
        sa.set_goal(40,15);
        // sa.set_start(startGoal[0](0),startGoal[0](1));
        // sa.set_goal(startGoal[1](0),startGoal[1](1));
        int search_state = sa.random_astar_path();
        std::cout << "[[[ Astart search state: " << search_state << " ]]]" << std::endl;

        // path optimization
      //   cubicSplineOpt(sceneBase& _scene, Mat& _path, double _step, double _c0 = 200, double _c1 = -10)
      // : scene0(_scene), path0(_path), step(_step), c0(_c0), c1(_c1) 
        cubicSplineOpt cso(sa, sa.astar_path, 0.9, 200., -10.);
        std::cout << "[[[ initial cubic spline OK!]]]" << std::endl;
        cso.path_opt_lbfgs();
        std::cout << "[[[ L-BFGS path optimization OK!]]]" << std::endl;

        // trajectory time optimization
        cso.topp_prepare(4);
        // conicALMTOPP2(Vec& _s, Mat& _q, Mat& _qv, Mat& _qa, double _a_max,
        //         double _v_max, double _v_start, double _v_end)
        conicALMTOPP2 topp2(cso.s1, cso.q1, cso.qv1, cso.qa1, a_max_, v_max_, 0.0, 0.0);
        Vec a,b,c,d;
        double result;
        int topp_ret = topp2.solve(result, a, b, c, d);
        if(is_print)
        {
          std::cout << "\n===============\n[Solving status:] " << topp_ret << "\n";
          std::cout << "[Minimum value:] " << result << "\n";
          std::cout << "[a:] " << a.transpose() << std::endl;
          std::cout << "[b:] " << b.transpose() << std::endl;
          std::cout << "[c:] " << c.transpose() << std::endl;
          std::cout << "[d:] " << d.transpose() << std::endl;
        }
        tqv=topp2.qv;
        tqxy=topp2.q;
        // b_global_path=true;//全局轨迹规划完成
        

        std::cout << "[[[ disp_sa initialization OK!]]]" << std::endl;
        // disp_sa.add_scene_grid();
        disp_sa.add_pos_goal();
        std::cout << "[[[ disp_sa add goal OK!]]]" << std::endl;
        
        // disp_sa.add_astar_path();
        std::cout << "[[[ disp_sa add Astar path OK!]]]" << std::endl;

        splineOptDis disp_cso(disp_rviz, cso);
        disp_cso.add_lbfgs_path_points();
        // disp_cso.add_lbfgs_path();
        std::cout << "[[[ disp_sa add L-BFGS path OK!]]]" << std::endl;

        conicAlmToppDis disp_topp(disp_rviz, topp2);
        disp_topp.add_topp_trajectory_points(v_min_, v_max_);
        std::cout << "[[[ disp_sa add TOPP trajectory OK!]]]" << std::endl;

      }
      disp_rviz.send();
      ros::spinOnce();
      rate.sleep();

      
    }   
  };

   ~TmLocalPlanner(){};

private:
    //planning 
    double planning_frequency_;
    int t=0;
    // is Variables
    bool b_global_path;
    bool b_vehicle_state;
    bool b_obstacles;
    bool is_goal;
    //车辆参数
    double wheelbase_length_;
    double v_min_,v_max_;
    double a_max_;
    VehicleState current_state;//
    
  // ROS
  // Subscribers and Publishers
    ros::Subscriber odom_sub_,goal_sub_;
    ros::Subscriber obstacles_sub_;  
    ros::NodeHandle nh_;
    ros::Publisher current_pose_rviz_pub_,vis_car_pub;
    ros::Publisher cmd_vel_pub_;
    visualization_msgs::Marker car_m;
    int car_id;
    std::vector<Eigen::Vector3d> startGoal;
    // Timer
    ros::Timer timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
   
    geometry_msgs::PoseStamped goal_pose; 

    Mat tqv;
    Mat tqxy;
    // Main Function 
  void mainTimerCallback(const ros::TimerEvent& timer_event){
    ROS_INFO_ONCE("enter mainTimeCallback");    
        geometry_msgs::Twist cmd_vel_msg;
      if(b_global_path)
      {
        if(tqv.size()>t)
        {
            cmd_vel_msg.linear.x = tqv(t,0);
          cmd_vel_msg.linear.y = tqv(t,1);
          cmd_vel_msg.linear.z = 0;

          cmd_vel_msg.angular.x = 0;
          cmd_vel_msg.angular.y = 0;
          cmd_vel_msg.angular.z = 0;//atan2(tqv(t,1),tqv(t,0));//y/x
        }
        else{
          cmd_vel_msg.linear.x = 0;
          cmd_vel_msg.linear.y = 0;
          cmd_vel_msg.linear.z = 0;

          cmd_vel_msg.angular.x = 0;
          cmd_vel_msg.angular.y = 0;
          cmd_vel_msg.angular.z = 0;//y/x
        }
        t++;
        cmd_vel_pub_.publish(cmd_vel_msg);
      }
      
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
  {
    ROS_INFO_STREAM_ONCE("ENTER odomCallback");
    current_state.speed = odom_msg->twist.twist.linear.x;
    current_state.steer = atan(wheelbase_length_*odom_msg->twist.twist.angular.z/current_state.speed);
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
    
    // createCurrentPoseMarker(current_state,pose);
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
    m.getRPY(roll,pitch,current_state.yaw);//获取整车的姿态角
    
    
    ROS_INFO_STREAM_ONCE("LEAVE odomCallback");
    b_vehicle_state=true;//获取到当前车辆的状态
  };
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    goal_pose.header=msg->header;
    goal_pose.pose=msg->pose;
    tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,
                      msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    startGoal.emplace_back(msg->pose.position.x,msg->pose.position.y,yaw);

    is_goal=true;
  }

};


#endif
