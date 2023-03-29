/**************************************************************************
 * ASatrSearch.hpp
 * 
 * @Author： bornchow
 * @Date: 2023.03.05
 * 
 * @Description:
 * new version of JPS with update vis and map process
 * 
 * - eigen lib
 * - priority queue
 *  
 * 参考程序:
 *  ****************************************************/
#include <Eigen/Core>
#include "ros/ros.h"
#include <iostream>
#include "map/GridMap.hpp"
#include "vis/visualization.hpp"
#include <queue>
#include "path_search/Common.h"

enum Dir{
    left, //0     (-1, 0)
    up,  //1      (0, 1)
    right,  //2   (1, 0)
    down, // 3    (0, -1)
    upLeft, // 4  (-1, 1)
    upRight, // 5 (1, 1)
    downRight,// 6  (1. -1)
    downLeft,// 7 (-1, -1)
};

// (-1, 0, 1, 0, -1, 1, 1, -1, -1)

// define in Common.h
// struct Node
// typedef priority_queue<Node*, vector<Node*> ,Compare > PQ;


class JPS
{
private:
    ros::NodeHandle nh_;
    std::shared_ptr<env::GridMap> env_ptr_;
    std::shared_ptr<vis::Visualization> vis_ptr_;

    // ros param
    bool is_debug_;
    int heuristic_func_type_, iter_inter_step_;

    int iter_;
    PQ open_set_;
    std::vector<Node*> close_set_;

    std::vector<int> DIR_;

    std::vector<Eigen::Vector3i> res_path_in_map_;
    std::vector<Eigen::Vector3d> res_path_in_world_;
private:
    bool isNodeEqual(Node* a, Node*b){ // the same to A*
        return a->p.isApprox(b->p, 1e-5);
    }

    void backSetNode(Node* node, double &sum){ // the same to A*
        sum = 0;
        while (node != nullptr)
        {
            if(node->father_node != nullptr){
                sum += (node->p - node->father_node->p).norm();
            }
            res_path_in_map_.push_back(node->p);
            res_path_in_world_.push_back(env_ptr_->mapToWorld(node->p));
            node = node->father_node;
        }
        reverse(res_path_in_map_.begin(), res_path_in_map_.end());
        reverse(res_path_in_world_.begin(), res_path_in_world_.end());

    }

public:
    JPS(ros::NodeHandle nh,
    std::shared_ptr<env::GridMap> &envPtr,
    std::shared_ptr<vis::Visualization> &visPtr) :
    nh_(nh),
    env_ptr_(envPtr),
    vis_ptr_(visPtr)
    {
        nh_.param<bool>("JPS/is_debug", is_debug_, false);
        nh_.param<int >("JPS/heuristic_func_type", heuristic_func_type_, 0);
        nh_.param<int >("JPS/iter_inter_step", iter_inter_step_, 100);

        ROS_INFO("\033[1;32m ----> this is a [2D] [ JPS ] search method!! .\033[0m");
        ROS_WARN_STREAM("[JPS] param | heuristic_func_type: " << heuristic_func_type_);
        ROS_WARN_STREAM("[JPS] param | is_debug: " << is_debug_);
        if(is_debug_){
            ROS_WARN_STREAM("[JPS] param | iter_inter_step: " << iter_inter_step_);
        }

        iter_ = 0;

        DIR_ = {-1, 0, 1, 0, -1, 1, 1, -1, -1};
    }

    vector<Eigen::Vector3i> getPath(){
        return res_path_in_map_;
    }

    vector<Eigen::Vector3d> getPathInWorld(){
        return res_path_in_world_;
    }

    void reset(){
        iter_ = 0;

        close_set_.clear();

        while (!open_set_.empty())
        {
            open_set_.pop();
        }

        res_path_in_map_.clear();
        res_path_in_world_.clear();
        
    }

    // main function
    bool search(Eigen::Vector3d start, Eigen::Vector3d end){
        // reset the global paramter
        reset();

        ros::Time search_start_time = ros::Time::now();

        // check the start and end node
        if(!env_ptr_->isStateValid(start)){
            ROS_ERROR_STREAM("[JPS]: invalid satrt point");
        }

        if(!env_ptr_->isStateValid(end)){
            ROS_ERROR_STREAM("[JPS]: invalid end point");
        }

        ROS_INFO_STREAM("[JPS] start search .....");

        // add start node to open set
        Eigen::Vector3i p_start_in_map = env_ptr_->worldToMap(start);
        Node* start_node = new Node(p_start_in_map);
        open_set_.push(start_node);

        // define end node
        Eigen::Vector3i p_end_in_map = env_ptr_->worldToMap(end);
        Node* end_node = new Node(p_end_in_map);

        Node* current_node;

        int c = 1;

        while (!open_set_.empty())
        {
            iter_++;

            // ROS_INFO_STREAM("iter--------------------- " << iter_ << " open set size: " << open_set_.size());
            // if(is_debug_){
            //     showOpenSet(open_set_);
            // }

            if(is_debug_){
                if(iter_ % iter_inter_step_ == 0){
                    std::cout << "input any to go on!!" << std::endl;
                    c = cin.get();
                }
            }

            // step 1: push a cost mini Node to process
            current_node = open_set_.top();

            if(is_debug_) ROS_INFO_STREAM("[JPS] currnet node: " << *current_node);

            // check if search over
            if(isNodeEqual(current_node, end_node)){
                double path_dis = 0;
                backSetNode(current_node, path_dis);

                //vis_ptr_->visualize2dPoints("final_nodes", res_path_in_world_, env_ptr_->getResolution(), vis::Color::red, 1, "map", true);
                ROS_INFO_STREAM("\033[1;32m[JPS] search: | over...... .\033[0m");
                ROS_INFO_STREAM("\033[1;32m[JPS] search: | waypoint nodes nu: " << res_path_in_map_.size() << " \033[0m");
                ROS_INFO_STREAM("\033[1;32m[JPS] search: | iter nu: " << iter_ << "  \033[0m");
                ROS_INFO_STREAM("\033[1;32m[JPS] search: | search time: " << (ros::Time::now() - search_start_time).toSec() * 1000 << " (ms) \033[0m");
                ROS_INFO_STREAM("\033[1;32m[JPS] search: | search dis(start to end): "<< (start - end).norm() <<" (m).\033[0m");
                ROS_INFO_STREAM("\033[1;32m[JPS] search: | path dis : "<< path_dis <<" (m).\033[0m");
                return true;
            }

            // step 2: remove the node from open_set
            open_set_.pop();
            
            // setp 3: jump point search
            searchJumpPoint(current_node, end_node);

            // setp 4: add the node to close_set
            close_set_.push_back(current_node);

        }

        return false;
        
    } //***** search()

    // *******************************
    // 搜索跳点
    // 这里是JPS不同于 A*的地方
    void searchJumpPoint(Node* thisNode, Node* endNode){

        std::vector<Eigen::Vector3d> vis_jps;
        // 定义当前节点的搜索方向序列
        std::queue<int> dirList; //添加方向的时候, 是先按直线方向，再按斜线方向

        if (thisNode->father_node == nullptr) //如果当前节点没有父节点,(起点)，那么搜索方向是八个方向
        {
            dirList.push(Dir::left);
            dirList.push(Dir::up);
            dirList.push(Dir::right);
            dirList.push(Dir::down);
            dirList.push(Dir::upLeft);
            dirList.push(Dir::upRight);
            dirList.push(Dir::downLeft);
            dirList.push(Dir::downRight);
        }else //如果当前节点有父节点
        {   
            // father --> thisNode
            nodeToDirs(thisNode->father_node, thisNode, dirList);
        }

        //如果当前节点有强迫邻居
        if(!thisNode->forcing_neis.empty()){
            for(int i=0; i<thisNode->forcing_neis.size(); i++){
                // thisNode --> forcing_node
                nodeToDirs(thisNode, thisNode->forcing_neis[i], dirList);
            }
        }

        if(is_debug_) ROS_INFO_STREAM("[JPS] Dirs size: " << dirList.size());

        //按方向进行搜索
        int searchState = 0;
        while (!dirList.empty())
        {
            int thisDir = dirList.front();
            dirList.pop();

            if(thisDir <= 3){ //这里是直线搜索
                std::vector<Eigen::Vector3d> vis_search_point;
                if(is_debug_) ROS_INFO_STREAM("[JPS] search dir --> " << thisDir << " ( " << DIR_[thisDir] << " , " << DIR_[thisDir+1] << " )");
                Node* checkNode = new Node(thisNode->p(0) + DIR_[thisDir], 
                                            thisNode->p(1) + DIR_[thisDir+1], 
                                            thisNode->p(2), 
                                            thisNode);
        
                searchState = dirSearchState(checkNode, endNode, thisDir);

                //for vis
                if(is_debug_){
                    vis_search_point.push_back(env_ptr_->mapToWorld(checkNode->p));
                }

                while(searchState == 0){
                    checkNode->p(0) += DIR_[thisDir];
                    checkNode->p(1) += DIR_[thisDir+1];
                    searchState = dirSearchState(checkNode, endNode, thisDir);

                    //for vis
                    if(is_debug_){
                        vis_search_point.push_back(env_ptr_->mapToWorld(checkNode->p));
                    }
                }

                if(searchState == 2){ //checkNode是跳点
                    addNodeToOpenSet(checkNode, endNode);

                    //vis  JP 
                    if(is_debug_){
                       vis_jps.push_back(env_ptr_->mapToWorld(checkNode->p));
                    }
                    
                }

                // vis search point
                if(is_debug_){
                    if(iter_ == 1){ // clear vis when the fisrt iter
                        vis_ptr_->visualize2dPoints("JPS_search_node", vis_search_point, env_ptr_->getResolution(), vis::Color::blue, 0.3, "map", true);
                    }else{
                        vis_ptr_->visualize2dPoints("JPS_search_node", vis_search_point, env_ptr_->getResolution(), vis::Color::blue, 0.3);
                    }
                }

            }else if(thisDir <= 7){ // 这里是斜向搜索
                std::vector<Eigen::Vector3d> vis_search_point;

                if(is_debug_) ROS_INFO_STREAM("[JPS] search dir diagonal--> " << thisDir << " ( " << DIR_[thisDir] << " , " << DIR_[thisDir+1] << " )");
                
                Node* checkNode = new Node(thisNode->p(0) + DIR_[thisDir], 
                            thisNode->p(1) + DIR_[thisDir+1], 
                            thisNode->p(2), 
                            thisNode); // 向该方向前进一步,注意斜向节点的父节点被设置为了thisNode
                // searchState = dirSearchState(checkNode, endNode, thisDir);
                
                //for vis
                if(is_debug_){
                    vis_search_point.push_back(env_ptr_->mapToWorld(checkNode->p));
                }

                int XYDir = 0; //记录分量方向
                bool hasJP = false; //

                while(dirSearchState(checkNode, endNode, thisDir) == 0){
                    // 斜向搜索转换为平行于坐标轴的两个分量方向的搜索
                    // 以upLeft为例，其需要转换为 向上搜索 和向左搜索 两个分量
                    // 两个分量的节点的父节点被设置为了斜向节点

                    //for vis
                    if(is_debug_){
                        vis_search_point.push_back(env_ptr_->mapToWorld(checkNode->p));
                    }

                    // cout << "checkNode in diagonal dir: " << *checkNode << std::endl;
                    // x方向分量的节点
                    Node* XNode = new Node(checkNode->p(0) + DIR_[thisDir], 
                            checkNode->p(1), 
                            checkNode->p(2), 
                            checkNode); // 向该分量方向前进一步,注意其父节点被设置为了斜向节点
                    
                    XYDir = DIR_[thisDir] > 0 ? Dir::right : Dir::left; // x方向只有向左和向右两种情况
                    // if(is_debug_) ROS_INFO_STREAM("[JPS] search x dir --> " << XYDir);

                    searchState = dirSearchState(XNode, endNode, XYDir);
                    //for vis
                    if(is_debug_){
                        vis_search_point.push_back(env_ptr_->mapToWorld(XNode->p));
                    }
                    
                    while (searchState == 0)
                    {
                        XNode->p(0) += DIR_[thisDir];
                        searchState = dirSearchState(XNode, endNode, XYDir);
                        //for vis
                        if(is_debug_){
                            vis_search_point.push_back(env_ptr_->mapToWorld(XNode->p));
                        }
                    }

                    if(searchState == 2){ //在某个方向上搜索到跳点

                        // 同时斜向节点checkNode 也是跳点
                        // 这里一定要先将斜向节点checkNode加入openset，因为当前checkNode的fgh都是0，加入时才能计算其数值
                        // 而XNode 跳点的fgh数值计算是依赖其父节点checkNode的fgh的
                        addNodeToOpenSet(checkNode, endNode); 

                        // XNode 是跳点
                        addNodeToOpenSet(XNode, endNode);

                        hasJP = true; //这里不能直接返回，因为还要搜索其他分量

                        // for vis
                        if(is_debug_){
                            vis_jps.push_back(env_ptr_->mapToWorld(checkNode->p));
                            vis_jps.push_back(env_ptr_->mapToWorld(XNode->p));
                        }
                    }

                    // Y方向分量的节点
                    Node* YNode = new Node(checkNode->p(0), 
                            checkNode->p(1) + DIR_[thisDir+1], 
                            checkNode->p(2), 
                            checkNode); // 向该分量方向前进一步,注意其父节点被设置为了斜向节点

                    XYDir = DIR_[thisDir+1] > 0 ? Dir::up : Dir::down; // Y方向只有向上和向下两种情况
                    // if(is_debug_) ROS_INFO_STREAM("[JPS] search y dir --> " << XYDir);

                    searchState = dirSearchState(YNode, endNode, XYDir);
                    //for vis
                    if(is_debug_){
                        vis_search_point.push_back(env_ptr_->mapToWorld(YNode->p));
                    }

                    while (searchState == 0)
                    {
                        YNode->p(1) += DIR_[thisDir+1];
                        searchState = dirSearchState(YNode, endNode, XYDir);
                        //for vis
                        if(is_debug_){
                            vis_search_point.push_back(env_ptr_->mapToWorld(YNode->p));
                        }
                    }
                    if(searchState == 2){ //在某个方向上搜索到跳点

                        // 斜向节点checkNode 也是跳点
                        addNodeToOpenSet(checkNode, endNode); //先加入checkNode

                        // YNode 是跳点
                        addNodeToOpenSet(YNode, endNode);

                        hasJP = true; //这里不能直接返回，因为还要搜索其他分量

                        // for vis
                        if(is_debug_){
                            vis_jps.push_back(env_ptr_->mapToWorld(checkNode->p));
                            vis_jps.push_back(env_ptr_->mapToWorld(YNode->p));
                        }
                    }

                    if(hasJP) break;

                    // 向斜向 前进一步
                    checkNode->p(0) += DIR_[thisDir];
                    checkNode->p(1) += DIR_[thisDir+1];

                }

                // vis search point
                if(is_debug_){
                    vis_ptr_->visualize2dPoints("JPS_search_node", vis_search_point, env_ptr_->getResolution(), vis::Color::blue, 0.3);
                }
            }

        }

        // for vis
        if(is_debug_){
            ROS_INFO_STREAM("[vis] ------- vis jps size: " << vis_jps.size());
            if(!vis_jps.empty()){
                if(iter_ == 1){ // clear vis when the fisrt iter
                    vis_ptr_->visualize2dPoints("JPS_node", vis_jps, env_ptr_->getResolution(), vis::Color::blue, 0.8, "map", true);
                }else{
                    vis_ptr_->visualize2dPoints("JPS_node", vis_jps, env_ptr_->getResolution(), vis::Color::blue, 0.8);
                }
            }
        }

    } //***** searchJumpPoint()

    //*************************************
    // search state in this dir
    // return : state
    // 1 -- search over node is beyoung map or collide
    // 2 -- search over node is jump point
    // 0 -- search continue in this dir
    //
    int dirSearchState(Node* node, Node* endNode, int dir){
        
        // 1. 判断节点是否超过边界 or 是否是障碍
        if(!env_ptr_->isPointValid(node->p)){
            if(is_debug_) ROS_INFO_STREAM("[Dir search] check over by out of map or collide");
            return 1;
        }


        if(dir <= 3){ //如果是斜向运动，暂不判定跳点,交由斜向运动的直线分量判定
            
            // 目标点也是跳点
            if(isNodeEqual(node, endNode)){
                if(is_debug_) ROS_INFO_STREAM("[Dir search] check over by JP");
                return 2;
            }


            //判断是否是跳点
            if(isJumpPoint(node, dir)){
                if(is_debug_) ROS_INFO_STREAM("[Dir search] check over by JP");
                return 2;
            }

        }

        return 0;

    } // function dirSearchState()

    // 将跳点加入openSet
    // 
    // Node* node 给node赋了f,g,h
    void addNodeToOpenSet(Node* node, Node* endNode){
        // 1. 判断跳点是否在close_set中
        for(size_t i=0; i<close_set_.size(); i++){
            if(isNodeEqual(close_set_[i], node)){
                if(is_debug_) ROS_INFO_STREAM("[add node] - in close set");
                return;
            }
        }

        // 2. 计算跳点代价
        countCost(node, endNode);

        // 3. 判断跳点是否在open_set中
        inOpenSetProcess(node, open_set_);
    
    }

    void showOpenSet(PQ & openSet){
        ROS_INFO_STREAM("=======================================");
        PQ backSet;
        while(!openSet.empty()){
            Node* openNode = openSet.top();
            openSet.pop(); 
            std::cout << *openNode << std::endl;
            backSet.push(openNode);
        }

        swap(openSet, backSet);
        ROS_INFO_STREAM("=======================================");
    }


    bool inOpenSetProcess(Node* node, PQ & openSet){
        if(is_debug_) ROS_INFO_STREAM("[add node] - node: " << *node );
        PQ backSet;
        bool inSet = false;
        while(!openSet.empty()){
            Node* openNode = openSet.top();
            openSet.pop();  
            if(isNodeEqual(node, openNode)){ // in openSet
                if(node->f < openNode->f){ // 当前节点代价更优
                    backSet.push(node);
                    if(is_debug_) ROS_INFO_STREAM("[add node] - in open set but change" );
                }else{
                    backSet.push(openNode);
                    if(is_debug_) ROS_INFO_STREAM("[add node]- in open set" );
                }
                inSet = true;
            }else{
                backSet.push(openNode);
            }
        }

        if(!inSet){
            backSet.push(node);
            if(is_debug_) ROS_INFO_STREAM("[add node] - not in open set" );
        } 

        swap(openSet, backSet);

        return inSet;
    }


    //**********************************
    // 计算当前节点的代价
    //【输入】 node 当前节点  node 值中的 f, g, h会被修改
    //【输入】 end_node 终点
    // 调用全局变量 heuristic_func_type 启发函数类型， 用什么方法计算当前节点与终点的启发值
    // 0 - Manhattan Distance 曼哈顿距离
    // 1 - Diagonal distance 对角距离
    // 2 - Euclidean distance 欧几里得距离(直线距离)
    //【输出】 node 赋值f,g,h的当前节点值
    // 参考： https://zhuanlan.zhihu.com/p/54510444
    void countCost(Node* node, Node* end_node){
        float f, g, h;
        float dx = abs(node->p(0) - end_node->p(0));
        float dy = abs(node->p(1) - end_node->p(1));

        // 计算 h
        if (heuristic_func_type_ == 0) 
        {
            h = dx + dy;
        }else if (heuristic_func_type_ == 1)
        {
            h = dx + dy + (sqrt(2) -2 ) * min(dx, dy);
        }else if (heuristic_func_type_ == 2)
        {
            h = pow((dx, 2) + pow(dy, 2), 0.5);
        }
        
        // 计算 g = father.g + node 到 father_node的距离
        g = node->father_node->g + (node->p - node->father_node->p).norm();

        f = g + h;

        node->f = f;
        node->g = g;
        node->h = h;

    }

    //*******************************************
    // 检查该节点是不是跳点                                          ^ y
    //     * * *     upLeftIndex    upIndex      upRightIndex    |
    //     * x *      leftIndex     thisIndex    rightIndex      |
    //     * * *    downLeftIndex   downIndex   downRightIndex   ------>x
    // 该函数只能检测水平和竖直方向的跳点
    bool isJumpPoint(Node* node, int dir){
        
        bool isJP = false;

        Eigen::Vector3i upLeftPoint    (node->p(0)-1, node->p(1)+1, node->p(2));
        Eigen::Vector3i upPoint        (node->p(0),   node->p(1)+1, node->p(2));
        Eigen::Vector3i upRightPoint   (node->p(0)+1, node->p(1)+1, node->p(2));
        Eigen::Vector3i leftPoint      (node->p(0)-1, node->p(1),   node->p(2));
        Eigen::Vector3i rightPoint     (node->p(0)+1, node->p(1),   node->p(2));
        Eigen::Vector3i downLeftPoint  (node->p(0)-1, node->p(1)-1, node->p(2));
        Eigen::Vector3i downPoint      (node->p(0),   node->p(1)-1, node->p(2));
        Eigen::Vector3i downRightPoint (node->p(0)+1, node->p(1)-1, node->p(2));


        switch (dir)
        {
        case Dir::left:{
            if(!env_ptr_->isPointValid(leftPoint)){
                return false;
            }

            // node 是跳点 upLeftPoint是强迫邻居
            if(!env_ptr_->isPointValid(upPoint) && env_ptr_->isPointValid(upLeftPoint)){
                //将强迫邻居赋给node节点
                Node* neiNode = new Node(upLeftPoint);
                node->forcing_neis.push_back(neiNode);
                isJP = true;
            }

            // node 是跳点 downLeftPoint是强迫邻居
            if(!env_ptr_->isPointValid(downPoint) && env_ptr_->isPointValid(downLeftPoint)){
                //将强迫邻居赋给node节点
                Node* neiNode = new Node(downLeftPoint);
                node->forcing_neis.push_back(neiNode);
                isJP = true;
            }

            break;
        }

        case Dir::right:{
            if(!env_ptr_->isPointValid(rightPoint)){
                return false;
            }

            // node 是跳点 upRightPoint是强迫邻居
            if(!env_ptr_->isPointValid(upPoint) && env_ptr_->isPointValid(upRightPoint)){
                //将强迫邻居赋给node节点
                Node* neiNode = new Node(upRightPoint);
                node->forcing_neis.push_back(neiNode);
                isJP = true;
            }

            // node 是跳点 downRightPoint是强迫邻居
            if(!env_ptr_->isPointValid(downPoint) && env_ptr_->isPointValid(downRightPoint)){
                //将强迫邻居赋给node节点
                Node* neiNode = new Node(downRightPoint);
                node->forcing_neis.push_back(neiNode);
                isJP = true;
            }

            break;
        }

        case Dir::up:{

            if(!env_ptr_->isPointValid(upPoint)){
                return false;
            }

            // node 是跳点 upRightPoint是强迫邻居
            if(!env_ptr_->isPointValid(rightPoint) && env_ptr_->isPointValid(upRightPoint)){
                //将强迫邻居赋给node节点
                Node* neiNode = new Node(upRightPoint);
                node->forcing_neis.push_back(neiNode);
                isJP = true;
            }


            // node 是跳点 upLeftPoint是强迫邻居
            if(!env_ptr_->isPointValid(leftPoint) && env_ptr_->isPointValid(upLeftPoint)){

                //将强迫邻居赋给node节点
                Node* neiNode = new Node(upLeftPoint);
                node->forcing_neis.push_back(neiNode);

                isJP = true;
            }

            break;
        }

        case Dir::down:{
            if(!env_ptr_->isPointValid(downPoint)){
                return false;
            }

            // node 是跳点 downRightPoint是强迫邻居
            if(!env_ptr_->isPointValid(rightPoint) && env_ptr_->isPointValid(downRightPoint)){
                //将强迫邻居赋给node节点
                Node* neiNode = new Node(downRightPoint);
                node->forcing_neis.push_back(neiNode);
                isJP = true;
            }


            // node 是跳点 downLeftPoint是强迫邻居
            if(!env_ptr_->isPointValid(leftPoint) && env_ptr_->isPointValid(downLeftPoint)){
                //将强迫邻居赋给node节点
                Node* neiNode = new Node(downLeftPoint);
                node->forcing_neis.push_back(neiNode);
                isJP = true;
            }
            break;
        }
        
        default:
            break;
        }

        if(isJP) return true;

        return false;

    } // isJumpPoint()


    //********************************************
    // 确定 a->b的方向 序列
    void nodeToDirs(Node* a, Node* b, std::queue<int>& dirs){
        int deltaX = b->p(0) - a->p(0);
        int deltaY = b->p(1) - a->p(1);
        if(deltaX > 0 && deltaY > 0){ //(1,1) --> 方向为 rightUp
            dirs.push(Dir::right);
            dirs.push(Dir::up);
            dirs.push(Dir::upRight);
        }else if(deltaX > 0 && deltaY == 0){ //(1, 0) --> 方向为 right
            dirs.push(Dir::right);
        }else if(deltaX > 0 && deltaY < 0){ //(1, -1) --> 方向为 rightDown 
            dirs.push(Dir::right);
            dirs.push(Dir::down);
            dirs.push(Dir::downRight);
        }else if (deltaX == 0 && deltaY > 0){ //(0, 1) --> 方向为 Up
            dirs.push(Dir::up);
        }else if(deltaX == 0 && deltaY < 0){ //(0, -1) --> 方向为 down
            dirs.push(Dir::down);
        }else if(deltaX < 0 && deltaY > 0){ //(-1, 1) --> 方向为 leftUp
            dirs.push(Dir::left);
            dirs.push(Dir::up);
            dirs.push(Dir::upLeft);
        }else if(deltaX < 0 && deltaY == 0){// (-1, 0) --> 方向为left
            dirs.push(Dir::left);
        }else if(deltaX < 0 && deltaY < 0){ //(-1, -1) --> 方向为leftDown
            dirs.push(Dir::left);
            dirs.push(Dir::down);
            dirs.push(Dir::downLeft);
        }else{
            ROS_ERROR("error deltaX and deltaY value");
        }  
    }
    

    ~JPS(){};
};




