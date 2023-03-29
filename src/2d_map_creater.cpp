#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <ctime>

using namespace std;

#define random(a, b) (rand()%(b-a+1)+a) // 返回[a, b]区间的随机数


int main(int argc, char * argv[]) {
    //随机种子
    srand(time(0));

    
    ros::init(argc, argv, "gridMap");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::Publisher pub = nh_private.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1000);

    int obstacle_num, obstacle_height, obstacle_width;
    int map_size_h, map_size_w;
    nh_private.param<int>("obstacle_num", obstacle_num, 20);
    nh_private.param<int>("obstacle_h", obstacle_height, 30);
    nh_private.param<int>("obstacle_w", obstacle_width, 30);
    nh_private.param<int>("map_size_h", map_size_h, 10);
    nh_private.param<int>("map_size_w", map_size_w, 10);

    nav_msgs::OccupancyGrid map;

    ROS_INFO("\033[1;32m ----> This is [2d] map create node... .\033[0m");


    // 向Rviz发送的数据中一定要包含frame_id
    map.header.frame_id="map";
    map.header.stamp = ros::Time::now(); 
    map.info.resolution = 0.05;          // float32
    // 栅格地图分辨率对应栅格地图中一小格的长和宽
    map.info.width      = (int)map_size_w/map.info.resolution;           // uint32
    map.info.height     = (int)map_size_h/map.info.resolution;           // uint32

    // 设置地图起始坐标　默认为0
    map.info.origin.position.x = -(map.info.width*map.info.resolution)/2;
    map.info.origin.position.y = -(map.info.height*map.info.resolution)/2;

    map.data.resize(map.info.width*map.info.height);

    // int p[map.info.width*map.info.height];   // [0,100] 
    // p[21] = 100;  //(1, 1) map坐标
    // p[20] = 100;  //(1, 0)

    // p[30] = 100;
    // p[31] = -1;
    // p[32] = -1;

    for(int n=0; n<obstacle_num;n++){
        int x = random(obstacle_width/2, map.info.width-obstacle_width/2);
        int y = random(obstacle_height/2, map.info.height-obstacle_height/2);
        for(int i = x - obstacle_width/2; i<x+obstacle_width/2; i++){
            if(i < 0 || i >= map.info.width) continue;
            for(int j = y - obstacle_height/2; j < y + obstacle_height/2; j++){
                if(j < 0 || j >= map.info.height) continue;
                map.data[j*map.info.width+i] = 100;
            }
        }
    }

    // ROS_INFO_STREAM("22222222222222");
    // p[int(floor((x-map.info.origin.position.x)/map.info.resolution) + floor((y-map.info.origin.position.y)/map.info.resolution))] = 100;

    // std::vector<signed char> a(p, p+map.info.width*map.info.height);
    // ROS_INFO_STREAM("3333333333");
    // map.data = a;


    ROS_WARN_STREAM("[map gen] param | map_size_h   : " << map_size_h);
    ROS_WARN_STREAM("[map gen] param | map_size_w   : " << map_size_w);
    ROS_WARN_STREAM("[map gen] param | obstacle_num : " << obstacle_num);
    ROS_WARN_STREAM("[map gen] param | obstacle_h   : " << obstacle_height);
    ROS_WARN_STREAM("[map gen] param | obstacle_w   : " << obstacle_width);
    ROS_WARN_STREAM("[map gen] | origin             : " << map.info.origin.position.x << " x " << map.info.origin.position.y);
    ROS_WARN_STREAM("[map gen] | resolution         : " << map.info.resolution);
    ROS_WARN_STREAM("[map gen] | width X height     : " << map.info.width << " x " << map.info.height);

    ros::Rate r(10);
    while (ros::ok())
    {
        map.header.stamp = ros::Time::now(); 
        pub.publish(map);
        r.sleep();
        // cout << "origin: " << map.info.origin.position.x << " " << map.info.origin.position.y << endl;
    }
    // ros::shutdown();
    return 0;
}
