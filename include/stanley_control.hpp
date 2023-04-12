#ifndef C4C9A1DC_82BC_419B_9191_51D95D7DB618
#define C4C9A1DC_82BC_419B_9191_51D95D7DB618
#include "common.h"
#include "main.hpp"
#include "pid_controller.hpp"

namespace control {


PIDController e_theta_pid_controller(1.0, 0.0, 0.4); 
// PID控制器中的微分环节相当于阻尼，加在航向误差引起的前轮转角上

class StanleyController {
 public:
  StanleyController(double limit_angle):limit_angle_(limit_angle)
  {};
  ~StanleyController(){};

  void LoadControlConf()
  {
    k_y_=0.5;
  };
  void ComputeControlCmd(const VehicleState &vehicle_state,
                         double& angular_cmd)
  {
    // 主车的状态信息
      double current_vehicle_x = vehicle_state.x;  
      double current_vehicle_y = vehicle_state.y;
      double current_vehicle_heading = vehicle_state.yaw;
      double current_vehicle_velocity = vehicle_state.speed;
      double e_y = 0.0;
      double e_theta = 0.0;
      ComputeLateralErrors(current_vehicle_x, current_vehicle_y, current_vehicle_heading, e_y, e_theta);
      double e_delta=atan2(k_y_*e_y,current_vehicle_velocity+ 6.0);//atan2 返回的是弧度单位，在分子上添加一个常数项，在低速条件下改善控制器性能
      e_delta=NormalizeAngle(e_delta);
    //  PID控制器中的微分环节相当于阻尼，加在航向误差引起的前轮转角上，抑制高速工况下的过大的前轮转角变化率
      double e_theta_pd = e_theta_pid_controller.Control(e_theta,0.01);
      e_theta_pd=NormalizeAngle(e_theta_pd);
      double raw_steering_control = e_y + 0.5 * e_theta_pd;
      // 限制前轮转角的取值区间
      if (raw_steering_control > M_PI)
      {
          raw_steering_control = raw_steering_control - M_PI * 2;
      }
      if (raw_steering_control < -M_PI)
      {
          raw_steering_control = raw_steering_control + M_PI * 2;
      }
      // 限制前轮最大转角，这里定义前轮最大转角位于 [-20度～20度]
      if (raw_steering_control >= deg_to_PI(limit_angle_))
      {
          raw_steering_control = deg_to_PI(limit_angle_);
      }
      else if (raw_steering_control <= -deg_to_PI(limit_angle_))
      {
          raw_steering_control = -deg_to_PI(limit_angle_);
      }
      angular_cmd= raw_steering_control;

  };
  void ComputeLateralErrors(const double x, const double y, const double theta,
                            double &e_y, double &e_theta){
       e_y=norm_double((x-ref_x_),(y-ref_y_));
      // 将位置误差转换为前轮转角的时候：需要将路径上距离车辆最近的点从世界坐标系变换到车辆坐标系下，
      // 根据路径点在车辆坐标系下的横坐标的正负决定前轮转角的方向
      double closest_point_y_in_vehicle_coordinate=-(ref_x_-x)*sin(theta)+(ref_y_-y)*cos(theta);
      // 车辆坐标系：X轴沿着车辆纵向，向前为正，Y沿着车辆横向，向左为正（从车头往前看的视角），
      // 在车辆坐标系下，距离车辆最近的路径点位于车辆左侧，车辆应该左转以跟踪参考路径，
      if(closest_point_y_in_vehicle_coordinate>0){
        e_y=-e_y;
      }          
      e_theta = theta_ref_- theta;

  };
  void ref_state(const double x, const double y,const double ref_theta)
  {
    ref_x_=x;
    ref_y_=y;
    theta_ref_=ref_theta;
  };

 protected:

  
  double k_y_ = 0.0;
  double u_min_ = 0.0;
  double u_max_ = 100.0;
  const double limit_angle_;
  double theta_ref_;
  double ref_x_,ref_y_;
  double theta_0_;
};

}  // namespace control



#endif /* C4C9A1DC_82BC_419B_9191_51D95D7DB618 */
