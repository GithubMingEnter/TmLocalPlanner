/**************************************************************************
 * HybridAstarSearch.hpp
 * 
 * @Author： bornchow
 * @Date: 2022.07.26
 * 
 * @Description:
 *  本程序为hybrid A*路径搜索算法实现
 *  
 * 参考程序:
 *  ****************************************************/

#ifndef AB462D5A_A713_46A2_8D0B_A91004B08299
#define AB462D5A_A713_46A2_8D0B_A91004B08299

#include "ros/ros.h"
#include <iostream>

#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <queue>
#include <algorithm>
#include <thread>
#include <map>
#include "DubinsCurves.hpp"
#include "GraphGroupSearch.hpp"

//     1     2    3
//      \    |   /
//        \  |  /
//           |
//        /  |  \
//       /   |    \
//      4    5     6
// 混合A*节点定义
typedef struct HNode 
{
    float x;
    float y;
    float g; //cost so far
    float h; //cost to go
    float f; //total cost
    HNode* father_node;

    float yaw; //航向角
    int dir; //方向标识

    HNode(){}
    HNode(float hx, float hy, float hYaw){
        this->x = hx;
        this->y = hy;
        this->yaw = hYaw;
        this->g = 0.0;
        this->h = 0.0;
        this->f = 0.0;
        this->dir = 0; //0表示方向未标识
        this->father_node = NULL;
    }
    HNode(float hx, float hy, float hYaw, HNode* father){
        this->x = hx;
        this->y = hy;
        this->yaw = hYaw;
        this->g = 0.0;
        this->h = 0.0;
        this->f = 0.0;
        this->dir = 0; //0表示方向未标识
        this->father_node = father;
    }
}HNode;

typedef std::multimap<float, HNode*> HANodeSets;
typedef std::multimap<float, HNode*>::iterator HANodeSetsIter;

class HybridAstarSearch : public GraphGroupSearch
{
private:
    int dir_; //方向 3 或 6
    int iter_; //总的迭代次数
    int iterStep_;

    //车辆运动学参数
    float delta_;  // 最大前轮转角 幅度值
    float L_;      // 车辆长度
    float T_;      // 混合A*扩展时的仿真时间
    float R_;

    float dy_[3];
    float dx_[3];
    float dYaw_[3];

    bool mapUpdated_;
    bool searchFinished_;

    nav_msgs::OccupancyGrid map_;

    HANodeSets openSet_, closeSet_;

    // for vis
    std::mutex vis_mut_;
    std::map<int, std::queue<geometry_msgs::Point> > iter_vis_set_;
    std::deque<geometry_msgs::Point> p_final_que_;

    DubinsCurves dubins;

private:
    HNode* findNeighbor(HNode* thisNode, int dir);
    float callDis(HNode* n1, HNode* n2){
         return sqrt(pow(n1->x - n2->x, 2) + pow(n1->y - n2->y, 2));
    }
    bool dubisShot(HNode* curNode, HNode* ENode, float r, std::queue<geometry_msgs::Point>& visQue);

public:
    HybridAstarSearch(int dir, int iterStep, float delta, float vehicleLength);
    ~HybridAstarSearch();
    void search(geometry_msgs::Pose pStart, geometry_msgs::Pose pEnd);
    void updateMap(nav_msgs::OccupancyGrid mapData);
    void getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                    std::queue<geometry_msgs::Point> &p_final_q,
                    int iter);
};

HybridAstarSearch::HybridAstarSearch(int dir, int iterStep, float delta, float vehicleLength):
dir_(dir),
delta_(delta),
L_(vehicleLength),
iterStep_(iterStep)
{
    
    iter_ = 0;
    mapUpdated_ = false;
    searchFinished_ = false;

    // 计算车辆的转弯半径
    R_ = L_ / tan(delta_);

    //计算扩展模型
    float x, y, theta;
    int simStep = 10;
    float simTime = 0.1;
    float simVel = 1.0;

    for(int i=0; i<simStep; i++){
        x += theta * cos(theta);
        y += theta * sin(theta);
        theta += simTime*simVel / R_;
    }
    dx_[0] = x; dx_[1] = x; dx_[2] = x;
    dy_[0] = 0; dy_[1] = -y; dy_[2] = y;
    dYaw_[0] = 0; dYaw_[1]=theta, dYaw_[2]=-theta;


    std::cout <<"======= config ==============" << std::endl;
    std::cout <<"turing R | " << R_ << std::endl;
    std::cout <<"dx: " << dx_[0] << " " << dx_[1] << " " << dx_[2] << std::endl;
    std::cout <<"dy: " << dy_[0] << " " << dy_[1] << " " << dy_[2] << std::endl;
    std::cout <<"dyaw: " << dYaw_[0] << " " << dYaw_[1] << " " << dYaw_[2] << std::endl;

    ROS_INFO("\033[1;32m----> this is a [2D] [Hybrid A star] search method!! .\033[0m");
}

HybridAstarSearch::~HybridAstarSearch()
{
}


HNode* HybridAstarSearch::findNeighbor(HNode* thisNode, int dir){
    float xSucc;
    float ySucc;
    float yawSucc;

    // calculate successor positions forward
    if (dir < 3) {
        xSucc = thisNode->x + dx_[dir] * cos(thisNode->yaw) - dy_[dir] * sin(thisNode->yaw);
        ySucc = thisNode->y + dx_[dir] * sin(thisNode->yaw) + dy_[dir] * cos(thisNode->yaw);
        yawSucc = thisNode->yaw + dYaw_[dir];
    }
    // backwards
    else {
        xSucc = thisNode->x - dx_[dir - 3] * cos(thisNode->yaw) - dy_[dir - 3] * sin(thisNode->yaw);
        ySucc = thisNode->y - dx_[dir - 3] * sin(thisNode->yaw) + dy_[dir - 3] * cos(thisNode->yaw);
        yawSucc = thisNode->yaw - dYaw_[dir-3];
    }

    HNode* newNode = new HNode(xSucc, ySucc, yawSucc, thisNode);
    newNode->dir = dir;
    return newNode;
}

void HybridAstarSearch::search(geometry_msgs::Pose pStart, geometry_msgs::Pose pEnd){
    while (!mapUpdated_)
    {
        ROS_INFO("no map info !! use updateMap function first");
    }

    // 重置参数
    openSet_.clear();
    closeSet_.clear();
    iter_ = 0;
    searchFinished_ = false;

    //第一步 初始化open_set，将起始点加入open_set

    HNode* startNode = new HNode(pStart.position.x, pStart.position.y, tf::getYaw(pStart.orientation));
    HNode* endNode = new HNode(pEnd.position.x, pEnd.position.y, tf::getYaw(pEnd.orientation));

    openSet_.insert(make_pair(startNode->f, startNode));

    //for vis 
    std::queue<geometry_msgs::Point> visPointQue;

    int a;

    //第二步 如果open_set为空， 返回false， 否则进入循环
    while(!openSet_.empty()){
        std::this_thread::sleep_for(std::chrono::microseconds (1));

        while(!visPointQue.empty()){
            visPointQue.pop();
        }


        if(iter_ % iterStep_ == 0){
            std::cout << "input any to go on!!" << std::endl;
            a = cin.get();
        }
        
        std::cout << "************************** " << iter_ << std::endl;

        // 第三步 从open_set中弹出一个f(n)最小的节点，作为当前节点 Cur
        HNode* curNode = openSet_.begin()->second;

        // 如果距离小于阈值，使用dubis曲线链接
        if(callDis(curNode, endNode) < 3*R_){
            std::cout << " --------- shot -------" << std::endl;
            dubisShot(curNode, endNode, R_, visPointQue);
            std::cout << " --------- visPointQue -------" << visPointQue.size() << std::endl;
        }

        // for vis
        vis_mut_.lock();
        iter_vis_set_.insert(make_pair(iter_, visPointQue));
        vis_mut_.unlock();

        iter_++;
    }

}

bool HybridAstarSearch::dubisShot(HNode* curNode, HNode* ENode, float r, std::queue<geometry_msgs::Point>& visQue){
    double q0[] = {curNode->x, curNode->y, curNode->yaw};
    double q1[] = {ENode->x, ENode->y, ENode->yaw};
    // initialize the path
    DubinsPath path;
    dubins.dubins_shortest_path(&path, q0, q1, r);

    int retcode;
    
    double x = 0.0;
    double length = dubins.dubins_path_length(&path);
    std::cout << " ---- " << std::endl;
    while (x < length)
    {
        double q[3];
        dubins.dubins_path_sample(&path, x, q);
        HNode* newNode = new HNode(q[0], q[1], q[2]);
        geometry_msgs::Point p;
        p.x = q[0];
        p.y = q[1];
        visQue.push(p);
        x += 0.1;
    }

    return false;
    
}

//****************************************
//// 【返回】 std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info
//  iter_vis_info.first 为 算法迭代次数
//  iter_vis_info.second 为 需要显示的新加点, 
void HybridAstarSearch::getVisInfo(std::map<int, std::queue<geometry_msgs::Point> >& iter_vis_info,
                             std::queue<geometry_msgs::Point> &p_final_q,
                             int iter){
    
    vis_mut_.lock();
    if (!iter_vis_set_.empty())
    {
        copy(iter_vis_set_.begin(), iter_vis_set_.end(), inserter(iter_vis_info, iter_vis_info.begin()));
        iter_vis_set_.clear();
    }

    if (searchFinished_)
    {
        while (!p_final_que_.empty())
        {
            p_final_q.push(p_final_que_.back()); //因为p_final_que_是从终点往起点储存的，所以这里要从后往前读，p_final_q才是从起点往终点储存
            p_final_que_.pop_back();
        }
    }

    vis_mut_.unlock();

    iter = iter_;
}


void HybridAstarSearch::updateMap(nav_msgs::OccupancyGrid mapData){
    mapUpdated_ = true;
    map_ = mapData;
}


#endif /* AB462D5A_A713_46A2_8D0B_A91004B08299 */
