#include "main.hpp"
#include <Eigen/Geometry>
#include <mpc_car/mpc_car.hpp>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <fstream>

void TmLocalPlanner::run()
{
  // 获取起始点
  bool is_start_set, is_goal_set;
  is_start_set = is_goal_set = false;
  ros::Rate loop_rate(20);

  Eigen::Vector3d start_vel;
  Eigen::Vector3d start_acc;

  start_vel.setZero();
  start_acc.setZero();

  Emx astar_path;
  rviz1DisSimp disp_rviz(nh_, "topp_array", "map", "topp_test");

  if (loadGlobalPath(xy_path, 1.0))
  {
    ROS_INFO("GET REF traj");
  }
  
  // startGoal.emplace_back(-0.95,0.218,0);
  // startGoal.emplace_back(1.755,-1.533,0);
  // startGoal.emplace_back(0,0,0);
  
  while (ros::ok())
  {
    startGoal.clear();
    is_start_set = is_goal_set = false;
    // get s e
    // while (startGoal.size() < 3)
    // {
    //   if (startGoal.size() == 0)
    //   {
    //     ROS_INFO_ONCE("set {start point} in rviz");
    //   }

    //   if (startGoal.size() == 1)
    //   {

    //     if (!is_start_set)
    //     {

    //       std::cout << "....." << startGoal.back().transpose() << std::endl;
    //       vis_ptr_->visualize2dPoint("start_point", startGoal.back(), 2 * resolution_, vis::Color::green);
    //       is_start_set = true;

    //       ROS_INFO("set {goal point} in rviz");
    //     }
    //   }
    //   if (startGoal.size() == 2)
    //   {

    //     if (!is_goal_set)
    //     {

    //       std::cout << "....." << startGoal.back().transpose() << std::endl;
    //       vis_ptr_->visualize2dPoint("goal_point", startGoal.back(), 2 * resolution_, vis::Color::green);
    //       is_goal_set = true;

    //       ROS_INFO("clicked point to start!!!");
    //     }
    //   }
    //   ros::spinOnce();
    //   loop_rate.sleep();
    // }
    // A-star search and opti
    ros::Time a_star_search_time = ros::Time::now();
    // 1.fornt-end path search
    
    topp_path_.header.stamp = ros::Time::now();
    topp_path_.header.frame_id = "map";

    if (b_global_ && !b_traj_)
    {
      disp_rviz.ma.markers.clear(); // 清理绘制图形
      topp_path_.poses.clear(); //清空

      vis_ptr_->visPath("a_star_final_path", Ev3_path_);
      ROS_WARN_STREAM("A star search time: | time1 --> " << (ros::Time::now() - a_star_search_time).toSec() * 1000 << " (ms)");

      // 2. corridor generte
      ros::Time time1 = ros::Time::now();
      shared_ptr<CorridorGen2D> corridor_gen = make_shared<CorridorGen2D>(pri_nh_, env_ptr_, vis_ptr_, start_vel);
      std::vector<Rectangle> corridor = corridor_gen->corridorGeneration(Ev3_path_);

      ROS_INFO_STREAM("corridor size: " << corridor.size());
      vis_ptr_->visCorridor("corridor", corridor, vis::Color::blue);
      ROS_WARN_STREAM("corridor_gen time: | time1 --> " << (ros::Time::now() - time1).toSec() * 1000 << " (ms)");

      time1 = ros::Time::now();
      astar_path.resize(Ev3_path_.size(), 2);
      Vec dist_grad(2);
      
      for (int i = 0; i < Ev3_path_.size(); i++)
      {
      
        astar_path.row(i) = Ev3_path_[i].head(2);
        // corridor_gen->dist_field(astar_path(i,0),astar_path(i,1),&dist_grad);
      }

      // path optimization
      //   cubicSplineOpt(sceneBase& _scene, Mat& _path, double _step, double _c0 = 200, double _c1 = -10)
      // : scene0(_scene), path0(_path), step(_step), c0(_c0), c1(_c1)
       cso = make_shared<cubicSplineOpt>(corridor_gen, astar_path, step_s_, c0_, c1_);
      // 3. optimal
      cso->path_opt_lbfgs();
      // 4. trajectory time optimization
      cso->topp_prepare(4);
      // conicALMTOPP2(Vec& _s, Mat& _q, Mat& _qv, Mat& _qa, double _a_max,
      // double _v_max, double _v_start, double _v_end);
      topp2=make_shared<conicALMTOPP2>(cso->s1, cso->q1, cso->qv1, cso->qa1, a_max_, v_max_, 0.0, 0.0);
      
      double result;
      int topp_ret = topp2->solve(result, a, b, c, d);
      
      std::cout<<"head = \n";
      // TODO 最后一项 Nan -1
      for(int i=0;i<cso->heading1.rows()-1;i++){
        geometry_msgs::PoseStamped ps;
        ps.pose.position.x=topp2->q(i,0);
        ps.pose.position.y=topp2->q(i,1);
        ps.pose.position.z =cso->heading1(i);
        std::cout<<cso->heading1(i)<<std::endl;
        geometry_msgs::Quaternion pose_quat=tf::createQuaternionMsgFromYaw(ps.pose.position.z);  //不适用这种 tf::createQuaternionFromYaw(ps.pose.position.z);
        ps.pose.orientation=pose_quat;
        topp_path_.poses.emplace_back(ps);
        

      }
      
      splineOptDis disp_cso(disp_rviz, *cso);
      disp_cso.add_lbfgs_path_points();
      // disp_cso.add_lbfgs_path();

      conicAlmToppDis disp_topp(disp_rviz, *topp2);
      disp_topp.add_topp_trajectory_points(v_min_, v_max_);
      // vis_ptr_->visualize2dPoints("a_star_final_traj", final_traj, env_ptr_->getResolution(), vis::Color::green, 1, "map", true);
      ROS_WARN_STREAM("Optimal traj time: | time3 --> " << (ros::Time::now() - time1).toSec() * 1000 << " (ms)");

      ROS_WARN_STREAM("total generate trajectory time: | total --> " << (ros::Time::now() - a_star_search_time).toSec() * 1000 << " (ms)");
      b_traj_ = true;

      b_global_=false;//reset 
    }
    disp_rviz.send();
    if(!topp_path_.poses.empty()){
      topp_path_pub_.publish(topp_path_);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}

void TmLocalPlanner::controlTimerCallback(const ros::TimerEvent &timer_event)
{
  ROS_INFO_ONCE("enter controlTimerCallback");
  geometry_msgs::Twist cmd_vel_msg;
  if (b_vehicle_state && b_traj_)
  {
    // ROS_INFO_STREAM("TIMES = "<<times_);
    int ret = 0;
    double solve_time, current_time;
    ++times_;

    // Eigen::Matrix<double, 3, 1> xba;
    // xba.norm()
    ros::Time ts = ros::Time::now();
    if(reach_goal_){
      pub_zero();
      
    }
    else{
      
      if(norm_double(current_state_.x-goal_pose.pose.position.x,current_state_.y-goal_pose.pose.position.y)<goal_tolerance_)
      {
        ROS_INFO_STREAM("Goal reached");
        reach_goal_=true;
        pub_zero();
      }
      else{
        
        double ld=lam_*current_state_.speed+c_;//lam*current_state_.speed+c;//前视距离范围

        // TODO ERROR
        vector<double> dists;
        int match_index=QueryNearestPointByPosition(current_state_,dists);
        //  double min_ind = min_element(dists.begin(),dists.end())-dists.begin(); //返回vector最小元素的下标
        
        double match_x=topp2->q(match_index,0);
        double match_y=topp2->q(match_index,1);
        double match_yaw=cso->heading1(match_index);
        double delta_l=PointDistanceSquare(current_state_, match_x, match_y); //norm_double(current_state_.x-match_x,current_state_.y-match_y);//前向距离实际点
        vector<double> match_array;
        double alpha;
        int look_index=0;
        for(int di=match_index;di<dists.size();di++){
          if(dists[di]>ld) // && dists[di]<ld
          {// TODO 终点距离0.3
            match_array.emplace_back(dists[di]);
            alpha=cso->heading1(di)-current_state_.yaw;
            if(abs(alpha)<M_PI/4 ){
              look_index=di;//update
              
              break;
            }
          }
            
        }
        if(look_index==0){
          ROS_INFO("CHECK ");
        }
        
        delta_l=dists[look_index]; 
        double R=delta_l/(2*sin(alpha));//(match_yaw-current_state_.yaw));
        double delta=std::atan(wheelbase_length_/R);//std::atan2(wheelbase_length_,);//     atan2(2*wheelbase_length_*sin(alpha),ld);
        delta=limit_deg(delta,20);
        delta=std::abs(delta)<0.1 ? 0 :delta;
        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = b(look_index)<1e-3? 0.03:b(look_index);//0.3;//speed_pid_control(0.5);//(b(match_index));
        infoD("match_index",look_index);
        infoD(" targetSpeed_ is ",b(look_index));
        infoD("v error",current_state_.speed);
        cmd_vel_msg.linear.y = 0;
        cmd_vel_msg.linear.z = 0;
        cmd_vel_msg.angular.x = 0;
        cmd_vel_msg.angular.y = 0;
        cmd_vel_msg.angular.z = delta;
        if(is_track_)
          cmd_vel_pub_.publish(cmd_vel_msg);

      }
      
    }
    
    double angular_cmd;
    // ROS_INFO_STREAM(" EGO HEAD "<<current_state_.yaw) ;
    

    ros::Time te = ros::Time::now();
    solve_time = (te - ts).toSec();

    all_time_ += solve_time;

    // std::cout << "mean solve_time: " << all_time_ / times_ << std::endl;
    // ROS_INFO_STREAM("time now" << (ros::Time::now()).toSec());


    // TODO


    // ackerman发布控制车辆指令
    // ackermann_msgs::AckermannDrive cmd_msg;
    // cmd_msg.speed = state_(3)+(u(0)*(dt_+solve_time));
    // cmd_msg.steering_angle = u(1);
    // cmd_msg.steering_angle_velocity = 2;
    // ack_pub_.publish(cmd_msg);
    // std::cout << "u: " <<state_(3)<<" "<<cmd_msg.speed << std::endl;

    // cmd_vel


  }
  b_vehicle_state=false;//需要新的车辆状态
}

// void TmlocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topp");
  ros::NodeHandle nh, pri_nh("~");
  std::shared_ptr<TmLocalPlanner> tm_local_planner=std::make_shared<TmLocalPlanner>(nh, pri_nh);
  // TmLocalPlanner tm_local_planner(nh, pri_nh);
  tm_local_planner->run();

  return 0;
}
