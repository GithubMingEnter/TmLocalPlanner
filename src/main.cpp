#include "main.hpp"


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
          Eigen::Vector2d pt;
          pt<<pt_x,pt_y;
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






// void TmlocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)

int main(int argc, char** argv) {
  ros::init(argc, argv, "topp");
  TmLocalPlanner tm_local_planner;

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
