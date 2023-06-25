
更新链接，https://github.com/GithubMingEnter/TmLocalPlanner/tree/real_env
欢迎star

# 环境配置

ros安装

osqp安装 [**osqp.org**](https://osqp.org/)
[https://blog.csdn.net/whuzhang16/article/details/111508384](https://blog.csdn.net/whuzhang16/article/details/111508384)

qpOASES安装
https://zhuanlan.zhihu.com/p/433639446

hybrid A*代码 https://github.com/dengpw/hybrid_astar_planner


# run
## 仿真环境
运行

`roslaunch topp2 simulation.launch `

轨迹规划与跟踪控制

`roslaunch topp2 topp.launch`

在play.world中修改相关参数

也可与teb算法对比，compare文件夹下


## 实验环境
位于real_env分支

底层节点
这里采用的是WHEELTEC的akm小车，自己写的一个启动文件

`roslaunch turn_on_wheeltec_robot topp_remote.launch  `



轨迹规划

`roslaunch topp2 real_topp.launch `


轨迹跟踪

`topp2/launch/real_env$ python real_diff_mpc.py `
也可以采用pure_pursuit跟踪，在topp.yaml中将is_track设为true
mpc跟踪要准确一点,要注意路径的配置

键盘遥控
```bash
roslaunch wheeltec_robot_rc keyboard_teleop.launch
```



停止命令
``` bash
$ rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0


```


topp_remote.launch
```xml
<launch>
<!-- 使用cartography建图注意 -->
  <!-- Arguments参数 -->
  <arg name="car_mode"  default="mini_diff" 
       doc="opt: mini_akm,senior_akm,top_akm_bs,top_akm_dl,
                 mini_mec,senior_mec_bs,senior_mec_dl,top_mec_bs,top_mec_dl,senior_mec_EightDrive,top_mec_EightDrive,
                 mini_omni,senior_omni,top_omni,
                 mini_tank,mini_diff,mini_4wd,senior_diff,four_wheel_diff_bs,four_wheel_diff_dl, brushless_senior_diff,
                 mini_tank_moveit,mini_4wd_moveit,mini_mec_moveit"/>
  <!--是否重复开启底层节点 在语音运行自主建图时开启 此处默认不开启-->
  <arg name="repeat"  default="false"/>
    <!--是否使用cartographer建图算法 此处默认不使用-->
  <arg name="is_cartographer"  default="false"/>
  <arg name="odom_frame_id"  default="odom_combined"/>
  <!-- turn on base_serial 开启底层单片机的控制节点  -->
  <include file="$(find turn_on_wheeltec_robot)/launch/include/base_serial.launch" unless="$(arg repeat)">
    <arg name="odom_frame_id"  value="$(arg odom_frame_id)"/>
  </include>
   
<!-- turn on lidar开启思岚雷达   -->
  <include file="$(find rplidar_ros)/launch/rplidar.launch" />


 <!-- turn on lidar开启思岚雷达  -->
 <!-- <include file="$(find rplidar_ros)/launch/rplidar.launch" /> -->

 <!-- 设置需要用于导航的地图  -->
 <arg name="map_file" default="$(find turn_on_wheeltec_robot)/map/c103_1.yaml"/>
 <node name="map_server_for_test" pkg="map_server" type="map_server" args="$(arg map_file)">
 </node>
  <!-- 发布用于建图、导航的TF关系与小车外形可视化 -->
  <include file="$(find turn_on_wheeltec_robot)/launch/robot_model_visualization.launch" unless="$(arg repeat)">
    <arg name="car_mode" value="$(arg car_mode)"/>
  </include>

  <!-- 扩展卡尔曼滤波 发布odom_combined到footprint的TF,即小车定位 使用cartographer算法时不使用该滤波算法-->
  <include file="$(find turn_on_wheeltec_robot)/launch/include/robot_pose_ekf.launch" unless="$(arg repeat)">
    <arg name="is_cartographer" value="$(arg is_cartographer)"/>
  </include>

 <!-- 开启用于导航的自适应蒙特卡洛定位amcl-->
 <include file="$(find turn_on_wheeltec_robot)/launch/include/amcl.launch" />



</launch>


```
# 数据处理
data文件夹下文件，
在下面库中进行绘图
https://github.com/GithubMingEnter/data-graph-matlab-/settings

# ref
https://github.com/tk166/Shenlan_Numerical-Optimization_in_Robotics

https://github.com/dengpw/hybrid_astar_planner
