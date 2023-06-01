#include <tf/transform_listener.h>
#include <iostream>
#include <tf/tf.h>
#include <pluginlib/class_loader.h>
#include <pluginlib_tutorials/polygon_base.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_core/base_global_planner.h>
#include <tf2_ros/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <ros/node_handle.h>

class TestPlanner{
    public:
        /**
         * @brief   Default constructor for the TestPlanner
         * @param tf A reference to a TransformListener
         */
        TestPlanner(tf2_ros::Buffer &_tf):
        tf(_tf){
            n.param<std::string>("base_link_frame",base_link_frame_,"base_link");
            //订阅目标主题，绑定响应函数,这里使用suscribe订阅目标点，当目标点刷新就重新进行路径规划
            make_plane = n.subscribe("/move_base_simple/goal", 1, &TestPlanner::setgoal, this);
            //定义类插件的名称，以便之后接入系统
            
            // std::string name_plugin="global_planner/GlobalPlanner";
            // global_planner = name_plugin; 
            global_planner = std::string("hybrid_astar_planner/HybridAStarPlanner");  
            
            // ros::NodeHandle nh("~/global_costmap");
            pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"); //导入插件
            planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
            //这个global_costmap,是它自己的名字，这里需要用param服务器给到costmap的参数，才会初始化

            // 需要注意的是，这里的初始化函数Costmap2DROS在构造时候的问题，~代表的是私有域的数据
            costmap = new costmap_2d::Costmap2DROS("global_costmap", tf);         

            std::cout << "creat the global costmap" << std::endl;
            //指定costmap中的base_link为起始坐标
            robot_pose.header.frame_id = base_link_frame_;  
                                            
            transformStarPose();
            try
            {
                planner_=bgp_loader_.createInstance(global_planner);                    
                planner_->initialize(bgp_loader_.getName(global_planner),costmap);
            }
            catch (const pluginlib::PluginlibException &ex)
            {
                ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
                exit(1);
            }
        }
        
        ;
        /**
         * @brief   Destructor - Cleans up 
         **/
        ~TestPlanner()
        {
            planner_.reset();
            if(costmap) {
                delete costmap;
            }
        };
        /**
         * @brief   Call back function of a suscriber 目的是接收RViz传过来的goal pose
         * @param  _goal A reference to geometry_msgs::PoseStamped::ConstPtr
         */
        void setgoal(const geometry_msgs::PoseStamped::ConstPtr& _goal){
            // std::cout << "receved the goal pose" <<std::endl;
            goal_pose.pose = _goal->pose;
            ROS_INFO_STREAM("GOAL POSE= "<<goal_pose.pose);
            goal_pose.pose.orientation = _goal->pose.orientation;
            goal_pose.header = _goal->header;
            transformStarPose();
            costmap->start();
            planner_->makePlan(start_pose, goal_pose, *planner_plan_);
        }

    private:
        /**
         * @brief   Transform the Start pose from tf tree 将起始点从TF转化树中提取出来(测试程序将bese_link作为起始点提取)
         * 
         */
        std::string origin_frame="map";
        std::string base_link_frame_="base_link";
        bool transformStarPose(void){
            try
            {
                start_transform = tf.lookupTransform("map", base_link_frame_, ros::Time(0), ros::Duration(3.0));
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                return false;
            }
            start_pose.pose.position.x = start_transform.transform.translation.x;
            start_pose.pose.position.y = start_transform.transform.translation.y;
            start_pose.pose.position.z = start_transform.transform.translation.z;
            start_pose.pose.orientation.w = start_transform.transform.rotation.w;
            start_pose.pose.orientation.x = start_transform.transform.rotation.x;
            start_pose.pose.orientation.y = start_transform.transform.rotation.y;
            start_pose.pose.orientation.z = start_transform.transform.rotation.z;
            ROS_INFO_STREAM("start POSE= "<<start_pose.pose);

            return true;
        }
        //智能指针，用来加载类插件，保存类插件的地址。这里没有使用标准库是因为Classloder的createInstance函数支持boost库，并没有使用标准库
        boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
        //路径规划的vector类模板，用来保存路径
        std::vector<geometry_msgs::PoseStamped>* planner_plan_;
        geometry_msgs::PoseStamped goal_pose;
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::TransformStamped start_transform;
        ros::NodeHandle n;
        costmap_2d::Costmap2DROS* costmap;
        geometry_msgs::PoseStamped robot_pose;
        std::string global_planner;
        tf2_ros::Buffer& tf;
        ros::Subscriber make_plane;
        ros::Publisher hy_global_pub;

};




int main(int argc, char** argv) {
    ros::init(argc,argv,"test_hyA_node");
    //ros::Duration(10)代表10秒的时间，在这里表示'tf2_ros::Buffer'实例化的对象保留10秒内的转换数据
    tf2_ros::Buffer buffer(ros::Duration(10));                                 
    //监听转换
    tf2_ros::TransformListener tf(buffer);                                      
    TestPlanner test( buffer );

    ros::spin();
    return 0;
}





