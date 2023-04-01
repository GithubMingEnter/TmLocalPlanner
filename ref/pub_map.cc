#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
 
using namespace std;
 
nav_msgs::OccupancyGrid BuildMap(const string& imgPath,const string& metaPath);
 
//服务回调
bool ServiceCallBack(nav_msgs::GetMap::Request &req,nav_msgs::GetMap::Response &res)
{
    res.map=BuildMap(
    "/media/chen/chen/Robot/projects_ros_test/test/src/mrobot_navigation/maps/cloister_gmapping.pgm",
    "/media/chen/chen/Robot/projects_ros_test/test/src/mrobot_navigation/maps/cloister_gmapping1.yaml"
    );
    return true;
}
 
int main(int argc, char * argv[]) 
{
	ros::init(argc, argv, "map_gen_node");
 
	ros::NodeHandle nh;
 
	ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
 
	ros::ServiceServer serv=nh.advertiseService("/static_map",ServiceCallBack);
	
	nav_msgs::OccupancyGrid map=BuildMap(
	  "/media/chen/chen/Robot/projects_ros_test/test/map.pgm",
	  "/media/chen/chen/Robot/projects_ros_test/test/map1.yaml"
	);
 
	cout<<"正在发布地图"<<endl;
 
	while (ros::ok())
	{
		pub.publish(map);
	}
 
	ros::shutdown();
	return 0;
}
 
 
nav_msgs::OccupancyGrid BuildMap(const string& imgPath,const string& metaPath)
{
	nav_msgs::OccupancyGrid map;
	//消息头
	map.header.frame_id="map";
	map.header.stamp = ros::Time::now();
 
	cv::FileStorage f_yaml(metaPath,cv::FileStorage::READ);//读取yaml信息
	map.info.resolution=(float)f_yaml["resolution"]; //地图分辨率
 
	cv::FileNode arr = f_yaml["origin"];
	cv::FileNodeIterator it=arr.begin(), it_end = arr.end();
	map.info.origin.position.x=(float)( *it );it++;//设置地图原点
	map.info.origin.position.y=(float)( *it );it++;
	map.info.origin.position.z=(float)( *it );
 
	f_yaml.release();
 
	//读取地图图像文件
	cv::Mat image = cv::imread(imgPath, -1);
	vector<signed char> vec;
	//将图像文件转换为vector，并将数据范围缩放至0-100
        for (int i = image.rows-1; i >=0 ; i--) {
            uchar *imageRow = image.ptr(i);
            for (int j = 0;j<image.cols; j++)
                vec.push_back((char)((255-imageRow[j])*100/255));
        }
	map.info.width=image.cols;
	map.info.height=image.rows;
 
	cout<<"占据网格元信息："<<endl;
	cout<<map<<endl;
 
	map.data = vec;
 
	return map;
}