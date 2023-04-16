#ifndef COMMON_H
#define COMMON_H

#include <fstream>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Core>

using Emx=Eigen::MatrixXd;

using Evx=Eigen::VectorXd;
using Ev2=Eigen::Vector2d;
using Ev3=Eigen::Vector3d;
typedef struct  
{
    double x;
    double y;
    double yaw;
    double speed;
    double steer;
} VehicleState;


// 将角度(弧度制)归化到[-M_PI, M_PI]之间
double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double deg_to_PI(const double atan2) 
{
    return atan2 * M_PI / 180.0;
}
double PI_to_deg(const double atan2) 
{
    return atan2 *  180.0 /  M_PI;
}
double norm_double(double dx,double dy)
{
    return sqrt(dx*dx+dy*dy);
}
double limit_deg(double raw_steering_control,double limit_angle){
  if (raw_steering_control >= deg_to_PI(limit_angle))
  {
      raw_steering_control = deg_to_PI(limit_angle);
  }
  else if (raw_steering_control <= -deg_to_PI(limit_angle))
  {
      raw_steering_control = -deg_to_PI(limit_angle);
  }
  return raw_steering_control;
}
#endif