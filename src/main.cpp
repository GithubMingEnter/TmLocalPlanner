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
    while (startGoal.size() < 3)
    {
      if (startGoal.size() == 0)
      {
        ROS_INFO_ONCE("set {start point} in rviz");
      }

      if (startGoal.size() == 1)
      {

        if (!is_start_set)
        {

          std::cout << "....." << startGoal.back().transpose() << std::endl;
          vis_ptr_->visualize2dPoint("start_point", startGoal.back(), 2 * resolution_, vis::Color::green);
          is_start_set = true;

          ROS_INFO("set {goal point} in rviz");
        }
      }
      if (startGoal.size() == 2)
      {

        if (!is_goal_set)
        {

          std::cout << "....." << startGoal.back().transpose() << std::endl;
          vis_ptr_->visualize2dPoint("goal_point", startGoal.back(), 2 * resolution_, vis::Color::green);
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

    bool a_star_res = a_star_search_->search(startGoal[0], startGoal[1]);
    if (a_star_res && !b_traj)
    {
      disp_rviz.ma.markers.clear(); // 清理绘制图形
      std::vector<Ev3> final_path = a_star_search_->getPathInWorld();

      vis_ptr_->visPath("a_star_final_path", pt_vec);
      ROS_WARN_STREAM("A star search time: | time1 --> " << (ros::Time::now() - a_star_search_time).toSec() * 1000 << " (ms)");

      // 2. corridor generte
      ros::Time time1 = ros::Time::now();
      shared_ptr<CorridorGen2D> corridor_gen = make_shared<CorridorGen2D>(pri_nh_, env_ptr_, vis_ptr_, start_vel);
      std::vector<Rectangle> corridor = corridor_gen->corridorGeneration(pt_vec);

      ROS_INFO_STREAM("corridor size: " << corridor.size());
      vis_ptr_->visCorridor("corridor", corridor, vis::Color::blue);
      ROS_WARN_STREAM("corridor_gen time: | time1 --> " << (ros::Time::now() - time1).toSec() * 1000 << " (ms)");

      time1 = ros::Time::now();
      astar_path.resize(pt_vec.size(), 2);
      Vec dist_grad(2);
      for (int i = 0; i < pt_vec.size(); i++)
      {
        astar_path.row(i) = pt_vec[i].head(2);
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

      splineOptDis disp_cso(disp_rviz, *cso);
      disp_cso.add_lbfgs_path_points();
      disp_cso.add_lbfgs_path();

      conicAlmToppDis disp_topp(disp_rviz, *topp2);
      disp_topp.add_topp_trajectory_points(v_min_, v_max_);
      // vis_ptr_->visualize2dPoints("a_star_final_traj", final_traj, env_ptr_->getResolution(), vis::Color::green, 1, "map", true);
      ROS_WARN_STREAM("Optimal traj time: | time3 --> " << (ros::Time::now() - time1).toSec() * 1000 << " (ms)");

      ROS_WARN_STREAM("total generate trajectory time: | total --> " << (ros::Time::now() - a_star_search_time).toSec() * 1000 << " (ms)");
      b_traj = true;
    }
    disp_rviz.send();
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}

void TmLocalPlanner::controlTimerCallback(const ros::TimerEvent &timer_event)
{
  ROS_INFO_ONCE("enter controlTimerCallback");
  geometry_msgs::Twist cmd_vel_msg;
  if (b_vehicle_state && b_traj)
  {
    // ROS_INFO_STREAM("TIMES = "<<times_);
    int ret = 0;
    double solve_time, current_time;
    ++times_;

    // Eigen::Matrix<double, 3, 1> xba;
    // xba.norm()
    ros::Time ts = ros::Time::now();
    int match_index=QueryNearestPointByPosition(current_state_);
    double vel_cmd=speed_pid_control(0.3);//(b(match_index));
    double angular_cmd=head_pid_control(cso->heading1(match_index));
    // stanleyPtr_->ref_state(topp2->q(match_index,0),topp2->q(match_index,0),cso->heading1(match_index));
    // stanleyPtr_->ComputeControlCmd(current_state_,angular_cmd);
    // angular_cmd= current_state_.speed*angular_cmd/wheelbase_length_;

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
    cmd_vel_msg.linear.x = vel_cmd;
    cmd_vel_msg.linear.y = 0;
    cmd_vel_msg.linear.z = 0;

    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    cmd_vel_msg.angular.z = angular_cmd;
    cmd_vel_pub_.publish(cmd_vel_msg);

  }
}

// void TmlocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "topp");
  ros::NodeHandle nh, pri_nh("~");

  TmLocalPlanner tm_local_planner(nh, pri_nh);
  tm_local_planner.run();

  return 0;
}
