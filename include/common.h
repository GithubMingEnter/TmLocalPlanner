#ifndef COMMON_H
#define COMMON_H

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




#endif