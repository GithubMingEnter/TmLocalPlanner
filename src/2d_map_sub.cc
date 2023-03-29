#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include <ctime>

using namespace std;


void mapCallback()
{

}
int main(int argc,char *argv[])
{
    ros::init(argc,argv,"gridMap");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;
    ros::Subscriber sub_map=nh_private.subscribe<nav_msgs::OccupancyGrid>("/grid_map",10000,mapCallback);//需要大量的时间加载地图




    return 0;
}




