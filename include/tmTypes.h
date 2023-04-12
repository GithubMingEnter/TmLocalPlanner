#ifndef EEF29937_7449_4F00_BD9C_D1CEF8AFE588
#define EEF29937_7449_4F00_BD9C_D1CEF8AFE588
#ifndef E7078478_D63F_4638_B381_91D4CAB5A713
#define E7078478_D63F_4638_B381_91D4CAB5A713
#ifndef _TM_TYPES_H
#define _TM_TYPES_H

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>
#include<stdexcept>
#include<Eigen/Eigen>
#include<sstream>

#include <fstream>
#include <cstdio>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <time.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>


#define NONE -1e9

using Vec_f=std::vector<float>;
using Poi_f=std::array<float, 2>;


using Poi_d=std::array<double, 2>;


using Vec_Poi=std::vector<Poi_d>;
using uint=unsigned int;

using vxd=Eigen::VectorXd;
using vxf=Eigen::VectorXf;
using mxf=Eigen::MatrixXf;
using mxd=Eigen::MatrixXd;
using vecD =  std::vector<double>;



#endif


#endif /* E7078478_D63F_4638_B381_91D4CAB5A713 */


#endif /* EEF29937_7449_4F00_BD9C_D1CEF8AFE588 */
