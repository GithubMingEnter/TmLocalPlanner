#include"ros1_rviz/rviz_dis.hpp"





int main(int argc , char** argv){
    ros::init(argc,argv,"topp");
    ros::NodeHandle nh;
    rviz1DisSimp disp_rviz(nh, "test_rviz", "map", "test_rviz");
      ros::Rate rate(1);
    Eigen::MatrixXd m;
    m.resize(4,2);

    m(0,0)=0;m(0,1)=0;
    m(1,0)=3;m(1,1)=0;
    m(2,0)=3;m(2,1)=3;
    m(3,0)=2;m(3,1)=5;
    // m(4,0)=0;m(4,1)=3;
    disp_rviz.add_convex_poly_flat(1,m,1);
    while(ros::ok()){
      disp_rviz.send();
      ros::spinOnce();
      rate.sleep();
    }     
    return 0;
}





























