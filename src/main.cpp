#include "main.hpp"








// void TmlocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)

int main(int argc, char** argv) {
  ros::init(argc, argv, "topp");
  ros::NodeHandle nh,pri_nh("~");

  TmLocalPlanner tm_local_planner(nh,pri_nh);
  tm_local_planner.run();

}


// int main(int argc, char** argv) {
//   ros::init(argc, argv, "topp_node");
//   ros::NodeHandle nh;

//   // initialize a scene x0,y0,yaw0 lx,ly,ld,obs_d_min _obs_d_max _p_min _p_max
//   //区域100*50 与障碍物间隔距离最大最小 , 障碍物的大小
//   sceneAstar sa(1., 1., 0., 50., 20., 0.5, 3.6, 12.9, 1, 3);
//   std::cout << "[[[ scene generation OK!]]]" << std::endl;
//   sa.random_obstacles(3);
//   std::cout << "[[[ obstacles generation OK!]]]" << std::endl;
//   // std:: cout <<"{{{"<<sa.is_free(0.0, 0.0) <<std::endl;;
//   int search_state = sa.random_astar_path();
//   std::cout << "[[[ Astart search state: " << search_state << " ]]]" << std::endl;
//   Mat astar_path_load;
//   //指定路径
//   std::string path="/home/lwm/ros1_workspace/simulation_ws/src/topp/config/xy.txt";
//   loadGlobalPath(path,2);
//   astar_path_load.resize(gx.size(),2);
//   for(size_t i=0;i<gx.size();i++){
//     astar_path_load.row(i)=pt_vec[i];
//   }
  
  
//   // path optimization
//   cubicSplineOpt cso(sa, sa.astar_path, 0.9, 5., -18.);
//   std::cout << "[[[ initial cubic spline OK!]]]" << std::endl;
//   cso.path_opt_lbfgs();
//   std::cout << "[[[ L-BFGS path optimization OK!]]]" << std::endl;

//   // trajectory time optimization
//   cso.topp_prepare(2);
//   conicALMTOPP2 topp2(cso.s1, cso.q1, cso.qv1, cso.qa1, 1.0, 10.0, 0.0, 0.0);
//   Vec a,b,c,d;
//   double result;
//   int topp_ret = topp2.solve(result, a, b, c, d);
//   std::cout << "\n===============\n[Solving status:] " << topp_ret << "\n";
//   std::cout << "[Minimum value:] " << result << "\n";
//   std::cout << "[a:] " << a.transpose() << std::endl;
//   std::cout << "[b:] " << b.transpose() << std::endl;
//   std::cout << "[c:] " << c.transpose() << std::endl;
//   std::cout << "[d:] " << d.transpose() << std::endl;


  
//   // draw out the scene
//   rviz1DisSimp disp_rviz(nh, "sl_hw5_array", "map", "sl_hw5_test");
//   sceneAstarDisp disp_sa(disp_rviz, sa);
//   std::cout << "[[[ disp_sa initialization OK!]]]" << std::endl;
//   disp_sa.add_scene_grid();
//   disp_sa.add_pos_goal();
//   std::cout << "[[[ disp_sa add grid OK!]]]" << std::endl;
//   disp_sa.add_obstacles();
//   std::cout << "[[[ disp_sa add obstacles OK!]]]" << std::endl;
//   disp_sa.add_astar_path();
//   std::cout << "[[[ disp_sa add Astar path OK!]]]" << std::endl;

//   // splineOptDis disp_cso(disp_rviz, cso);
//   // disp_cso.add_lbfgs_path_points();
//   // disp_cso.add_lbfgs_path();
//   // std::cout << "[[[ disp_sa add L-BFGS path OK!]]]" << std::endl;
//   std::cout<<"sa.astar_path.size()= "<<sa.astar_path.size()<<std::endl;
//   conicAlmToppDis disp_topp(disp_rviz, topp2);
//   disp_topp.add_topp_trajectory_points(0.8, 6.8);
//   std::cout << "[[[ disp_sa add TOPP trajectory OK!]]]" << std::endl;
//   disp_rviz.send();
//   disp_rviz.send();
// }
