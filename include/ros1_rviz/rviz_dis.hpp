#ifndef RVIZ_DIS_
#define RVIZ_DIS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <string>
#include "../common.h"


struct colorSimp{
    colorSimp(double r0=0.5,double g0=1.0,double b0=1.0,double a0=1.0):
    r(r0),g(g0),b(b0),a(a0)
    {

    }
    ~colorSimp(){}
    double r,g,b,a;
};
void createCurrentPoseMarker(const VehicleState& current_state, geometry_msgs::PoseStamped& pose)
{
    pose.header.frame_id = "map";
    pose.pose.position.x = current_state.x;
    pose.pose.position.y = current_state.y;
    //使用tf中的createQuaternionMsgFromYaw
    geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromYaw(current_state.yaw);
    pose.pose.orientation = pose_quat;
}
struct rviz1DisSimp {
    using vis_Mar=visualization_msgs::Marker;
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Rate r;
    std::string p_name;
    std::string f_name;
    std::string ns;
    visualization_msgs::MarkerArray ma;
    int id0;
    rviz1DisSimp( ros::NodeHandle& nh,const std::string pub_name,const std::string frame_name,const std::string name_space)
    :n(nh),r(30),p_name(pub_name),f_name(frame_name),ns(name_space)
    {
        id0=0;
        pub=n.advertise<visualization_msgs::MarkerArray>(pub_name,30);
        while(pub.getNumSubscribers()==0)//判断订阅者是否连接
        {
            r.sleep();
        }


    }
    vis_Mar addCorlor(vis_Mar& m,double r,double g,double b,double a){
        m.color.r=r;
        m.color.g=g;
        m.color.b=b;
        m.color.a=a;
        return m;
    }
    vis_Mar get_default_marker(int id){
        vis_Mar m;
        m.header.frame_id=f_name;
        m.ns=ns;
        m.action=vis_Mar::ADD;
        m.id=id;
        m.type=vis_Mar::LINE_STRIP;
        m.pose.orientation.w=1.0;
        m.scale.x=0.2;
        //TODO
        m.color.r=1.0;
        m.color.g=1.0;
        m.color.b=1.0;
        m.color.a=1.0;
        return m;
    }
    int send(){
        int send_num=2;
        if(ros::ok()){
            while(send_num--)
                pub.publish(ma);
            return 1;
        }
        else{
            ROS_INFO_STREAM("send fail");
            return 0;
        }
        

    }
    int clear(){
        ma.markers.clear();
        id0=0;
        return 0;
    }
    int add_arrow(int id,double x0,double y0,double z0,double x1,double y1,double z1,
                double size,colorSimp c=colorSimp(1.0,0,0,1.0)){
        vis_Mar m=get_default_marker(id);
        m.type=vis_Mar::ARROW;

        
        m.color.r=c.r;
        m.color.g=c.g;
        m.color.b=c.b;
        m.color.a=c.a;
        m.scale.x=size; //adjust
        m.scale.y=size;
        m.scale.z=size;
        geometry_msgs::Point p1;
        p1.x=x0;p1.y=y0;p1.z=z0;
        m.points.emplace_back(p1);
        p1.x=x1;p1.y=y1;p1.z=z1;
        m.points.emplace_back(p1);
        ma.markers.emplace_back(m);
        return 0;      
    }
    int add_colored_path_strip2d(int id, Emx& path, Evx& vel, double vel_min, double vel_max, double width,
                               double height, colorSimp c_min = colorSimp(0.8, 0.2, 0.2, 1.0), colorSimp c_max = colorSimp(1.0, 1.0, 0.2, 1.0))    
    {
        vis_Mar m=get_default_marker(id);
        m.type = vis_Mar::LINE_STRIP;
        m.color.r=1.0;
        m.color.g=1.0;
        m.color.b=1.0;
        m.color.a=1.0;
        m.points.clear();
        m.scale.x=width;
        geometry_msgs::Point pl;
        std_msgs::ColorRGBA cl;
        pl.z=height;
        for(int k=0;k<path.rows()-1;++k){
            ROS_INFO_STREAM(k<<" (x,y)="<<path(k,0)<<" "<<path(k,1));
            pl.x=path(k,0);
            pl.y=path(k,1);
            //same type data
            double cl_pos=std::min(1.0,std::max(0.0,(vel(k)-vel_min)/(vel_max-vel_min)));
            cl.r=c_min.r+(c_max.r-c_min.r)*cl_pos;
             cl.g=c_min.g+(c_max.g-c_min.g)*cl_pos;
              cl.b=c_min.b+(c_max.b-c_min.b)*cl_pos;
               cl.a =1.0;
            m.points.emplace_back(pl);
            m.colors.emplace_back(cl);
        }
        ma.markers.emplace_back(m); //推入makers
        return 0;

    }
    int add_scattered2d(int id,Emx& points, double diameter,double height,colorSimp c=colorSimp())
    {
        vis_Mar m=get_default_marker(id);
        m.type=vis_Mar::SPHERE_LIST;
        m.color.r=c.r;
        m.color.g=c.g;
        m.color.b=c.b;
        m.color.a=c.a;
        m.points.clear();
        m.scale.x=diameter; //adjust
        m.scale.y=diameter; 
        m.scale.z=diameter; 
        geometry_msgs::Point pl;
        pl.z=height;
        for(int k=0;k<points.rows();k++){
            pl.x=points(k,0);
            pl.y=points(k,1);
            m.points.emplace_back(pl);
        }  
        ma.markers.emplace_back(m);
        return 0;

    }
    int add_path_strip2d(int id,Emx& path,double width,double height,colorSimp c=colorSimp())
    {
        vis_Mar m=get_default_marker(id);
        m.type=vis_Mar::LINE_STRIP;
        m.color.r=c.r;
        m.color.g=c.g;
        m.color.b=c.b;
        m.color.a=c.a;
        m.points.clear();
        m.scale.x=width;
        geometry_msgs::Point pl;
        pl.z=height;
        for(int k=0;k<path.rows();k++){
            pl.x=path(k,0);
            pl.y=path(k,1);
            m.points.emplace_back(pl);
        }
        ma.markers.emplace_back(m);
        return 0;

    }
    int add_cylinder(int id, double x0,double y0,double r0,double height,colorSimp c=colorSimp())
    {
        vis_Mar m=get_default_marker(id);
        m.type=vis_Mar::CYLINDER;
        m.color.r=c.r;
        m.color.g=c.g;
        m.color.b=c.b;
        m.color.a=c.a;
        m.pose.position.x=x0;
        m.pose.position.y=y0;
        m.scale.x=r0*2;
        m.scale.y=2*r0;
        m.scale.z=height;
        ma.markers.emplace_back(m);
        // boost::bind(&rviz1DisSimp::add_arrow,&rviz1DisSimp,m,ma);
        return 0;
    }
    int add_convex_poly_flat(int id,Emx& vertex,double height,colorSimp c=colorSimp())
    {
        vis_Mar m=get_default_marker(id);
        m.type=vis_Mar::TRIANGLE_LIST;
        m.scale.x =1.0;
        m.scale.y =1.0;
        m.scale.z =1.0;
        geometry_msgs::Point pl;
        std_msgs::ColorRGBA c_edge;
        c_edge.r=c.r;
        c_edge.g=c.g;
        c_edge.b=c.b;
        c_edge.a=c.a;
        m.points.clear();
        m.colors.clear();
        const int num=vertex.rows();
        for(int i=1;i<num-1;i++){
            pl.z=0.0; //?
            pl.x=vertex(0,0);
            pl.y=vertex(0,1);
            m.points.emplace_back(pl);
            pl.x=vertex(i,0);
            pl.y=vertex(i,1);
             m.points.emplace_back(pl);
            pl.x=vertex(i+1,0);
            pl.y=vertex(i+1,1);
             m.points.emplace_back(pl);
             m.colors.emplace_back(c_edge);
            pl.z = height;
            pl.x=vertex(0,0);
            pl.y=vertex(0,1);
            m.points.emplace_back(pl);
            pl.x=vertex(i,0);
            pl.y=vertex(i,1);
             m.points.emplace_back(pl);
            pl.x=vertex(i+1,0);
            pl.y=vertex(i+1,1);
             m.points.emplace_back(pl);
             m.colors.emplace_back(c_edge);
        }
        std_msgs::ColorRGBA c_face;
        c_face.r=0.8;
        c_face.g=0.8;
        c_face.b=0.8;
        c_face.a=0.8;
        int j=num-1;
        for(int k=0;k<=num-1;++k){
            pl.z=height; //?
            pl.x=vertex(k,0);
            pl.y=vertex(k,1);
            m.points.emplace_back(pl);
            pl.z=0.0;
            m.points.emplace_back(pl);
            pl.x=vertex(j,0);
            pl.y=vertex(j,1);
            m.points.emplace_back(pl);
            m.colors.emplace_back(c_face);
            //第二个三角形
            m.points.emplace_back(pl);
            pl.z = height;
            m.points.emplace_back(pl);
            pl.x=vertex(k,0);
            pl.y=vertex(k,1);
             m.points.emplace_back(pl);
             m.colors.emplace_back(c_face);
             j=k;
        }
        ma.markers.emplace_back(m);
        return 0;
    }
    
    virtual ~rviz1DisSimp() {}





};



#endif