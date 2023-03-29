/**************************************************************************
 * visualization.hpp
 * 
 * @Author： bornchow
 * @Date: 2022.12.09
 * 
 * @Description:
 * visualization for navi algorithm
 *  
 *  ****************************************************/
#ifndef _VISUALIZATION_HPP
#define _VISUALIZATION_HPP

#include <ros/ros.h>
#include <iostream>
#include <unordered_map>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <Eigen/Core>

using namespace std;

namespace vis{

typedef std::unordered_map<std::string, ros::Publisher> PublisherMap;

enum Color
{
    white,
    red,
    green,
    blue,
    yellow,
    chartreuse,
    black,
    gray,
    orange,
    purple,
    pink,
    steelblue
};

class Visualization
{
private:
    ros::NodeHandle nh_;
    PublisherMap publisher_map_;

    void setMarkerScale(visualization_msgs::Marker &marker, double x, double y, double z){
        marker.scale.x = x;
        marker.scale.y = y;
        marker.scale.z = z;
    }

    void setMarkerPose(visualization_msgs::Marker &marker, double x, double y, double z){
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
    }

    void setMarkerColor(visualization_msgs::Marker &marker,
                            Color color = blue,
                            double a = 1)
        {
            marker.color.a = a;
            switch (color)
            {
            case white:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 1;
                break;
            case red:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case green:
                marker.color.r = 0;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case blue:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case yellow:
                marker.color.r = 1;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case chartreuse:
                marker.color.r = 0.5;
                marker.color.g = 1;
                marker.color.b = 0;
                break;
            case black:
                marker.color.r = 0;
                marker.color.g = 0;
                marker.color.b = 0;
                break;
            case gray:
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
                break;
            case orange:
                marker.color.r = 1;
                marker.color.g = 0.5;
                marker.color.b = 0;
                break;
            case purple:
                marker.color.r = 0.5;
                marker.color.g = 0;
                marker.color.b = 1;
                break;
            case pink:
                marker.color.r = 1;
                marker.color.g = 0;
                marker.color.b = 0.6;
                break;
            case steelblue:
                marker.color.r = 0.4;
                marker.color.g = 0.7;
                marker.color.b = 1;
                break;
            }
        }

        void setMarkerColor(visualization_msgs::Marker &marker,
                            double a,
                            double r,
                            double g,
                            double b)
        {
            marker.color.a = a;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
        }

        template<class POINT>
        void setMarkerPoint(visualization_msgs::Marker &marker, POINT pt){
            geometry_msgs::Point p;
            p.x = pt(0);
            p.y = pt(1);
            p.z = pt(2);
            marker.points.push_back(p);
        }
        
        template<class POINT>
        void setMarkerPoints(visualization_msgs::Marker &marker, POINT pts){
            for(auto pt : pts){
                setMarkerPoint(marker, pt);
            }
        }
    
public:
    Visualization(ros::NodeHandle &nh): nh_(nh){};
    ~Visualization();

    template <class TOPIC, class POINT>  //vis a 2d point 
    void visualize2dPoint(const TOPIC &topic, 
                         const POINT &pt,
                         double size,
                         Color color,
                         double a=1,
                         std::string frame_id = "map"
                        ){

        auto got = publisher_map_.find(topic);
        if(got == publisher_map_.end()){
            ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
            publisher_map_[topic] = pub;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        setMarkerScale(marker, size, size, 0.1);
        setMarkerPoint(marker, pt);
        marker.ns = topic;
        marker.id = 0;
        marker.lifetime = ros::Duration();

        setMarkerColor(marker, color, a);
        marker.header.stamp = ros::Time::now();

        double time = ros::Time::now().toSec();
        while(publisher_map_[topic].getNumSubscribers() < 1 &&
                ros::Time::now().toSec() - time < 1.0){
            if(!ros::ok()) return;
            ros::Duration(0.01).sleep();
        }

        publisher_map_[topic].publish(marker);
    }

   
    template <class TOPIC, class POINT>  //vis 2d points 
    void visualize2dPoints(const TOPIC &topic, 
                         const POINT &pts, // vector<Eigen::Vector3d>
                         double size,
                         Color color,
                         double a=1,
                         std::string frame_id = "map",
                         bool is_delete_pre_data = false
                        ){
        static visualization_msgs::Marker marker;
        marker.ns = topic;

        auto got = publisher_map_.find(topic);
        if(got == publisher_map_.end()){
            ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10);
            publisher_map_[topic] = pub;
            marker.points.clear();
            marker.ns = topic;
        }

        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        setMarkerScale(marker, size, size, 0.1);
        setMarkerColor(marker, color, a);
        marker.id = 0;
        marker.lifetime = ros::Duration();

        if(is_delete_pre_data){
            marker.points.clear();
            setMarkerPoints(marker, pts);
            // marker.action = visualization_msgs::Marker::DELETE;
        }else{
            setMarkerPoints(marker, pts);
        }
        
        marker.header.stamp = ros::Time::now();

        double time = ros::Time::now().toSec();
        while(publisher_map_[topic].getNumSubscribers() < 1 ){
            if(!ros::ok()) return;
            ros::Duration(0.01).sleep();

            if(ros::Time::now().toSec() - time > 1.0){
                ROS_WARN_STREAM("NO ROS Subscriber to topic :" << topic);
                break;
            }
        }

        publisher_map_[topic].publish(marker);
    }

    template<class TOPIC, class POINTS>
    void visPath(const TOPIC &topic, const POINTS& pts, std::string frame_id = "map"){
        nav_msgs::Path path;
        for(size_t i=0; i<pts.size(); i++){
            geometry_msgs::PoseStamped thisPose;
            thisPose.header.frame_id = frame_id;
            thisPose.header.stamp = ros::Time::now();
            thisPose.pose.position.x = pts[i](0);
            thisPose.pose.position.y = pts[i](1);
            thisPose.pose.position.z = pts[i](2);
            thisPose.pose.orientation.x = 0;
            thisPose.pose.orientation.y = 0;
            thisPose.pose.orientation.z = 0;
            thisPose.pose.orientation.w = 1;
            path.poses.push_back(thisPose);
        }
        path.header.frame_id = frame_id;
        path.header.stamp = ros::Time::now();
        
        double time = ros::Time::now().toSec();
        while(publisher_map_[topic].getNumSubscribers() < 1 &&
                ros::Time::now().toSec() - time < 1.0){
            if(!ros::ok()) return;
            ros::Duration(0.01).sleep();
        }

        publisher_map_[topic].publish(path);
    }

    template<class TOPIC, class CORRIDOR_LIST>
    void visCorridor(const TOPIC &topic, const CORRIDOR_LIST &corridor, 
                    Color color, double a = 0.3, std::string frame_id = "map"){
   
        auto got = publisher_map_.find(topic);
        if(got == publisher_map_.end()){
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 1);
            publisher_map_[topic] = pub;
        }

        bool is_2d = corridor[0].vertex.rows() == 4 ? true : false;

        visualization_msgs::MarkerArray markers;

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        setMarkerColor(marker, color, a);
        marker.ns = "corridor";
        marker.header.stamp = ros::Time::now();

        for(size_t i=0; i<corridor.size(); i++){

            marker.id = i;
            
            if(is_2d){
                setMarkerScale(marker, 
                        abs(corridor[i].box[0].first - corridor[i].box[0].second), 
                        abs(corridor[i].box[1].first - corridor[i].box[1].second), 
                        0.05);
            }else{
                setMarkerScale(marker, 
                        abs(corridor[i].box[0].first - corridor[i].box[0].second), 
                        abs(corridor[i].box[1].first - corridor[i].box[1].second), 
                        abs(corridor[i].box[2].first - corridor[i].box[2].second));
            }

            setMarkerPose(marker, corridor[i].center(0), corridor[i].center(1), corridor[i].center(2));
            
            markers.markers.push_back(marker);
        }

        double time = ros::Time::now().toSec();
        while(publisher_map_[topic].getNumSubscribers() < 1 &&
                ros::Time::now().toSec() - time < 1.0){
            if(!ros::ok()) return;
            ros::Duration(0.01).sleep();
        }

        publisher_map_[topic].publish(markers);

    }

    template <class TOPIC_TYPE, class TOPIC>
    void registe(const TOPIC& topic) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<TOPIC_TYPE>(topic, 10);
            publisher_map_[topic] = pub;
        }
    }
};


Visualization::~Visualization()
{
}

}

#endif