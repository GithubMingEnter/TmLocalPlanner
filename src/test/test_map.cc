#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>

int main(int argc,char** argv){
    ros::init(argc,argv,"test_map_node");
    ros::NodeHandle nh;

    ros::Publisher map_pub=nh.advertise<nav_msgs::OccupancyGrid>("test_map",1000);
    ros::Rate r(10);
    nav_msgs::OccupancyGrid occ_msg;
    
    
    while(ros::ok()){
        occ_msg.header.stamp=ros::Time::now();
        occ_msg.header.frame_id="map";
        occ_msg.info.height=2;
        occ_msg.info.width=4;
        occ_msg.info.origin.position.x=0;
        occ_msg.info.origin.position.y=0;
        occ_msg.info.resolution=1;
        occ_msg.data.resize(4*2);
        occ_msg.data[0]=100;
        occ_msg.data[1]=50;
        occ_msg.data[2]=0;
        occ_msg.data[3]=-1;
        map_pub.publish(occ_msg);
        r.sleep();
    }
    return 0;

}