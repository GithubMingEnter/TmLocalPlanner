#ifndef _GRIDMAP_HPP
#define _GRIDMAP_HPP
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Core>

namespace env
{

    class GridMap
    {
    private:
        ros::NodeHandle n;

        nav_msgs::OccupancyGrid map_data_;
        
        void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg);



        int mapToInd(Eigen::Vector3i p_map){
            return p_map(1) * map_data_.info.width + p_map(0);
        }

        
    public:
        GridMap(ros::NodeHandle &nh) : n(nh){
            // ros::Subscriber map_sub = n.subscribe("/gridMap", 10, &GridMap::mapCallBack, this);
            ros::Subscriber map_sub = n.subscribe("/my_map", 10, &GridMap::mapCallBack, this);

            // wait until map is received, when a map is received, mapData.header.seq will not be < 1  
            while (map_data_.header.seq<1 or map_data_.data.size()<1){
                ROS_INFO("waiting for map ... \033[1A");
                ros::spinOnce();  
                ros::Duration(0.1).sleep();
            }
            ROS_INFO("map info res!");
            ROS_WARN_STREAM("map info print: | map size: " << map_data_.info.width << " * " << map_data_.info.height <<" (" 
            << map_data_.info.width* map_data_.info.resolution << " * " << map_data_.info.height*map_data_.info.resolution  <<")");
            ROS_WARN_STREAM("map info print: | map resolution: " << map_data_.info.resolution);
            ROS_WARN_STREAM("map info print: | map origin: " << map_data_.info.origin.position.x << " "
                                                            << map_data_.info.origin.position.y << " "
                                                            << map_data_.info.origin.position.z );

        }

        double getResolution(){return map_data_.info.resolution; }

        // 
        bool isStateValid(const Eigen::Vector3d p_real){
            Eigen::Vector3i p_map;
            p_map = worldToMap(p_real);
            
            return isPointValid(p_map);
        }

        bool isPointValid(const Eigen::Vector3i p_map){

            if(!isInMap(p_map)) return false;
            
            // TO DO: 
            return (map_data_.data[mapToInd(p_map)] != 100);
        }

        bool isInMap(Eigen::Vector3i p_map){
            // return ((p_map(0) | p_map(1) | (map_data_.info.width - p_map(0) - 1) | (map_data_.info.height - p_map(1) - 1)) >= 0);
            return p_map(0) >= 0 && p_map(1) >=0 && p_map(0) <= map_data_.info.width && p_map(1) <= map_data_.info.height;
        }

        // **************************************
        // 将点的真实坐标转换为地图坐标
        // 这里的地图是珊格地图，其以地图左下角(map_data.info.origin.position)为坐标原点， 
        // 地图的宽(map_data.info.width)为x方向,
        // 高(map_data.info.height)为y方向， 
        // 地图的分辨率(map_data.info.resolution)为大小构建的方格地图
        //  ^ y
        //  |
        //  |
        //  -----------> x
        void worldToMap(const Eigen::Vector3d p_real, Eigen::Vector3i &p_map){
            p_map(0) = floor((p_real(0) - map_data_.info.origin.position.x)/map_data_.info.resolution);
            p_map(1) = floor((p_real(1) - map_data_.info.origin.position.y)/map_data_.info.resolution);
            p_map(2) = 0;
        }

        Eigen::Vector3i worldToMap(const Eigen::Vector3d p_real){
            Eigen::Vector3i p_map;
            worldToMap(p_real, p_map);
            return p_map;
        }

        // **************************************
        // 将地图坐标点转换为真实坐标
        void mapToWorld(const Eigen::Vector3i p_map, Eigen::Vector3d &p_real){
            p_real(0) = (p_map(0) + 0.5) * map_data_.info.resolution + map_data_.info.origin.position.x;
            p_real(1) = (p_map(1) + 0.5) * map_data_.info.resolution + map_data_.info.origin.position.y;
            p_real(2) = 0;
        }

        Eigen::Vector3d mapToWorld(const Eigen::Vector3i p_map){
            Eigen::Vector3d p_real;
            mapToWorld(p_map, p_real);
            return p_real;
        }

        ~GridMap(){};
    };


    void GridMap::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg){
        map_data_ = *msg;
    }

}

#endif