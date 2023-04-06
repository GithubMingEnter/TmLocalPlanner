/**************************************************************************
 * Common.h
 * 
 * @Author： bornchow
 * @Date: 2023.03.08
 * 
 * @Description:
 * 基础数据结构定义
 *  ****************************************************/
#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <queue>

using namespace std;


//*************************************************************
// define node for A*, JPS ...

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






#endif /* COMMON_H */