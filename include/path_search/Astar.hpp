/**************************************************************************
 * ASatrSearch.hpp
 * 
 * @Author： bornchow
 * @Date: 2022.12.16
 * 
 * @Description:
 * new version of A-star with update vis and map process
 * 
 * - eigen lib
 * - priority queue
 *  
 * 参考程序:
 *  ****************************************************/

#include <Eigen/Core>
#include "ros/ros.h"
#include <iostream>
#include <queue>
#include "map/GridMap.hpp"
#include "vis/visualization.hpp"


using namespace std;

// define in Common.h
// struct Tm::Node
// typedef priority_queue<Tm::Node*, vector<Tm::Node*> ,Compare > PQ;
namespace Tm
{
typedef struct Node{
    Eigen::Vector3i p;
    double g;  // cost so far
    double h;  // cost to go 
    double f;  // total cost
    Node* father_node;
    std::vector<Node*> forcing_neis;

    
    Node(Eigen::Vector3i p){
        this->p = p;
        this->g = 0.0;
        this->h = 0.0;
        this->f = 0.0;
        this->father_node = NULL;
        this->forcing_neis = {}; // for JPS
    }

    Node(int x, int y, int z, Node* father){
        this->p(0) = x;
        this->p(1) = y;
        this->p(2) = z;
        this->g = 0.0;
        this->h = 0.0;
        this->f = 0.0;
        this->father_node = father;
        this->forcing_neis = {}; // for JPS
    }

    Node(Eigen::Vector3i p, Node* father){
        this->p = p;
        this->g = 0.0;
        this->h = 0.0;
        this->f = 0.0;
        this->father_node = father;
        this->forcing_neis = {}; // for JPS
    }

    Node() = default;


    // 重载输出函数
    friend std::ostream& operator << (std::ostream& ostr, Node &n){

        if (n.father_node == NULL)
        {
            std::cout << "[ " << n.p.transpose() << " ] -- " << "GHF: " << n.g << " " << n.h << " " << n.f 
            << " -- {  NULL  } " << std::endl;
        }else
        {
            std::cout << "[ " << n.p.transpose() << " ] -- " << "GHF: " << n.g << " " << n.h << " " << n.f 
            << " -- { " << (n.father_node)->p.transpose() << " } ";
        }

        return std::cout;
        
    }

}Node;

struct Compare
{
    bool operator()(const Node *a,const Node *b)const
    {
        if(a->f == b->f){
            return a->h > b->h;
        }else{
            return a->f > b->f;
        }
    }
};

typedef priority_queue<Node*, vector<Node*> ,Compare > PQ;  


} //namespace Tm

class Astar
{
private:
    ros::NodeHandle n_;
    std::shared_ptr<env::GridMap> env_ptr_;
    std::shared_ptr<vis::Visualization> vis_ptr_;
    std::priority_queue<Tm::Node*, vector<Tm::Node*> ,Tm::Compare > open_set_;
    vector<Tm::Node*> close_set_;

    bool is_debug_;
    int heuristic_func_type_;
    int iter_inter_step_, iter_;
    vector<int> node_dir_;

    vector<Eigen::Vector3i> res_path_in_map_;
    vector<Eigen::Vector3d> res_path_in_world_;

    void reset();
    bool isNodeEqual(Tm::Node* a, Tm::Node*b){
        return a->p.isApprox(b->p, 1e-5);
    }
    void backSetNode(Tm::Node* Tm_node, double &sum);
    void checkNeighbor(Tm::Node* thisNode, std::vector<Tm::Node*> &neiNodes, Tm::Node* endNode);
    bool isInSet(Eigen::Vector3i p_map, vector<Tm::Node*> set);
    double calHeuristicCost(Tm::Node* thisNode, Tm::Node* endNode);
    bool inOpenSetProcess(Tm::Node* Tm_node, Tm::PQ & openSet);
public:
    Astar(ros::NodeHandle nh, 
           std::shared_ptr<env::GridMap> &envPtr,
           std::shared_ptr<vis::Visualization> &visPtr);

    bool search(Eigen::Vector3d start, Eigen::Vector3d end);

    vector<Eigen::Vector3i> getPath(){
        return res_path_in_map_;
    }

    vector<Eigen::Vector3d> getPathInWorld(){
        return res_path_in_world_;
    }
    ~Astar(){};
};

Astar::Astar(ros::NodeHandle nh, 
              std::shared_ptr<env::GridMap> &envPtr, 
              std::shared_ptr<vis::Visualization> &visPtr): 
n_(nh),
env_ptr_(envPtr),
vis_ptr_(visPtr)
{
    bool is_four_neighborhood;
    nh.param<bool>("Astar/is_four_neighborhood", is_four_neighborhood, false);
    nh.param<int >("Astar/heuristic_func_type", heuristic_func_type_, 0);
    nh.param<bool>("Astar/is_debug", is_debug_, false);
    nh.param<int >("Astar/iter_inter_step", iter_inter_step_, 100);

    ROS_INFO("\033[1;32m ----> this is a [2D] [A star] search method!! .\033[0m");
    ROS_WARN_STREAM("[A*] param | is_four_neighborhood: " << is_four_neighborhood);
    ROS_WARN_STREAM("[A*] param | heuristic_func_type: " << heuristic_func_type_);
    ROS_WARN_STREAM("[A*] param | is_debug: " << is_debug_);
    if(is_debug_){
        ROS_WARN_STREAM("[A*] param | iter_inter_step: " << iter_inter_step_);
    }


    if(is_four_neighborhood){
        node_dir_ = {0, 1, 0, -1, 0};
    }else{
        node_dir_ = {0, 1, 0, -1, 1, 1, -1, -1, 0};
    }

    iter_ = 0;

    
}

bool Astar::isInSet(Eigen::Vector3i p_map, vector<Tm::Node*> set){
    for(auto n: set){
        if(p_map.isApprox(n->p, 1e-5)) return true;
    }
    return false;
}

double Astar::calHeuristicCost(Tm::Node* thisNode, Tm::Node* endNode){
    double dx = abs(thisNode->p(0) - endNode->p(0));
    double dy = abs(thisNode->p(1) - endNode->p(1));
    double h;

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
    }else{
        ROS_ERROR_STREAM("Error heuristic_func_type, support the 0 Manhattan Distance; \
        1 Diagonal distance; 2 Euclidean distance. But got "<< heuristic_func_type_);
    }

    return h;
}

bool Astar::inOpenSetProcess(Tm::Node* Tm_node, Tm::PQ & openSet){
    Tm::PQ backSet;
    bool inSet = false;
    while(!openSet.empty()){
        Tm::Node* openNode = openSet.top();
        openSet.pop();  
        if(isNodeEqual(Tm_node, openNode)){ // in openSet
            if(Tm_node->f < openNode->f){ // 当前节点代价更优
                backSet.push(Tm_node);
                if(is_debug_) ROS_INFO_STREAM("[check neighbor] in open set but change" );
            }else{
                backSet.push(openNode);
                if(is_debug_) ROS_INFO_STREAM("[check neighbor] in open set" );
            }
            inSet = true;
        }else{
            backSet.push(openNode);
        }
    }

    if(!inSet) backSet.push(Tm_node);

    swap(openSet, backSet);

    return inSet;
}

void Astar::checkNeighbor(Tm::Node* thisNode, std::vector<Tm::Node*> &neiNodes, Tm::Node* endNode){
    int x, y, z;
    for(int i=1; i<node_dir_.size(); i++){
        x = thisNode->p(0) + node_dir_[i-1];
        y = thisNode->p(1) + node_dir_[i];
        z = 0;
        Eigen::Vector3i nei(x, y, z);
        if(is_debug_) ROS_INFO_STREAM("[check neighbor] ......check new Tm_node....." << nei.transpose() );

        // check if the nei point is outside the map or collision
        if(!env_ptr_->isPointValid(nei)){
            if(is_debug_) ROS_INFO_STREAM("[check neighbor] out of map or collision" );
            continue;
        }

        // check if the nei point is in close set
        if(isInSet(nei, close_set_)){
            if(is_debug_) ROS_INFO_STREAM("[check neighbor] in close set" );
            continue;
        }

        // check if the nei point is in open set
        Tm::Node* nei_node = new Tm::Node(nei, thisNode);
        nei_node->g = thisNode->g + (nei_node->p - thisNode->p).norm();
        nei_node->h = calHeuristicCost(nei_node, endNode);
        nei_node->f = nei_node->g + nei_node->h;

        if(inOpenSetProcess(nei_node, open_set_)){
            continue;
        }

        if(is_debug_) ROS_INFO_STREAM("[check neighbor] add new Tm_node" << *nei_node);

        neiNodes.push_back(nei_node);
    }


}

void Astar::reset(){
    close_set_.clear();
    
    while(!open_set_.empty()){
        open_set_.pop();
    }

    iter_ = 0;

    res_path_in_map_.clear();
    res_path_in_world_.clear();

    // vis_ptr_->visualize2dPoints("A_star_node", res_path_in_world_, env_ptr_->getResolution(), vis::Color::yellow, 0.3, "map", true);

}

void Astar::backSetNode(Tm::Node* Tm_node, double &sum){
    sum = 0;
    while(Tm_node != nullptr){
        if(Tm_node->father_node != nullptr){
            sum += (Tm_node->p - Tm_node->father_node->p).norm();
        }
        res_path_in_map_.push_back(Tm_node->p);
        res_path_in_world_.push_back(env_ptr_->mapToWorld(Tm_node->p));
        Tm_node = Tm_node->father_node;
    }

    reverse(res_path_in_map_.begin(), res_path_in_map_.end());
    reverse(res_path_in_world_.begin(), res_path_in_world_.end());

}

bool Astar::search(Eigen::Vector3d start, Eigen::Vector3d end){
    // clear the global parameter
    reset();

    ros::Time search_start_time = ros::Time::now();
    // check the start node and end node
    if(!env_ptr_->isStateValid(start)){
        ROS_ERROR_STREAM("[Astar]: invalid satrt point");
    }

    if(!env_ptr_->isStateValid(end)){
        ROS_ERROR_STREAM("[Astar]: invalid end point");
    }

    ROS_INFO_STREAM("[A*] start search .....");

    // push start node into open_set
    Eigen::Vector3i p_start = env_ptr_->worldToMap(start);
    Tm::Node* start_node = new Tm::Node(p_start);
    open_set_.push(start_node);

    Eigen::Vector3i p_end = env_ptr_->worldToMap(end);
    Tm::Node* end_node = new Tm::Node(p_end);

    Tm::Node* current_node;

 
    int c = 1;


    while(!open_set_.empty()){

        iter_++;

        if(is_debug_){
            if(iter_ % iter_inter_step_ == 0){
                std::cout << "input any to go on!!" << std::endl;
                c = cin.get();
            }
        }

        // step 1: push a cost mini Tm::Node to process
        current_node = open_set_.top();

        if(is_debug_) ROS_INFO_STREAM("currnet node: " << *current_node);

        // check if search over
        if(isNodeEqual(current_node, end_node)){
            double path_dis = 0;
            backSetNode(current_node, path_dis);

            //vis_ptr_->visualize2dPoints("final_nodes", res_path_in_world_, env_ptr_->getResolution(), vis::Color::red, 1, "map", true);
            ROS_INFO_STREAM("\033[1;32m[A*] search: | over...... .\033[0m");
            ROS_INFO_STREAM("\033[1;32m[A*] search: | waypoint nodes nu: " << res_path_in_map_.size() << " \033[0m");
            ROS_INFO_STREAM("\033[1;32m[A*] search: | iter nu: " << iter_ << "  \033[0m");
            ROS_INFO_STREAM("\033[1;32m[A*] search: | search time: " << (ros::Time::now() - search_start_time).toSec() * 1000 << " (ms) \033[0m");
            ROS_INFO_STREAM("\033[1;32m[A*] search: | search dis(start to end): "<< (start - end).norm() <<" (m).\033[0m");
            ROS_INFO_STREAM("\033[1;32m[A*] search: | path dis : "<< path_dis <<" (m).\033[0m");

            return true;
        }

        // step 2: remove the node from open_set
        open_set_.pop();
        
        // step 3: 
        std::vector<Tm::Node*> neighborNodes;
        checkNeighbor(current_node, neighborNodes, end_node);

        // if vis
        if(is_debug_){
            std::vector<Eigen::Vector3d> neis_eigen;
            for(size_t i=0; i<neighborNodes.size(); i++){
                neis_eigen.push_back(env_ptr_->mapToWorld(neighborNodes[i]->p));
            }
            
            if(iter_ == 1){ // clear vis when the fisrt iter
                vis_ptr_->visualize2dPoints("A_star_node", neis_eigen, env_ptr_->getResolution(), vis::Color::green, 0.3, "map", true);
            }else{
                vis_ptr_->visualize2dPoints("A_star_node", neis_eigen, env_ptr_->getResolution(), vis::Color::green, 0.3);
            }
            
        }

        // setp 4: add the node to close_set
        close_set_.push_back(current_node);

    }

    return false;


}


