#ifndef A2E3B3C6_29AA_43CA_98A9_E22FD21E4C22
#define A2E3B3C6_29AA_43CA_98A9_E22FD21E4C22
// Grid node object used by the astar algorithm
#include <iostream>
#include <limits>
#include <vector>
#include "scene_obstacles.hpp"

#include "scene_base.hpp"
#include "common.h"

double Inf= numeric_limits<double>::max();
class gridNode{
public:    
    double x,y;
    Eigen::Vector2d pos;
    Eigen::Vector2i idx;
    int state;
    double f,g;//note cost
    gridNode* father;
    gridNode(Eigen::Vector2d _pos,Eigen::Vector2i _idx):
        state(0),father(nullptr),pos(_pos),g(Inf),idx(_idx),f(Inf),g(Inf){}
    gridNode(Eigen::Vector2d _pos):
        state(0),father(nullptr),pos(_pos),f(Inf),g(Inf){}
    ~gridNode(){};
    

};

class sceneAstar:public sceneBase{
public:
    double x0,y0,yaw0;
    double lx,ly,ld;
    double ild;
    double obs_num;
    size_t num_x=0;
    size_t num_y=0;
    std::vector<std::vector<gridNode>> map0;
    std::vector<baseObstacle*> obs;
    baseObstacleFactory* obs_factory_ptr;
    E2v pos_start,pos_goal;
    std::vector<gridNode*> grid_along_path;
    bool include_start,include_goal;
    Emx astar_path;

    sceneAstar(double _x0,double _y0,double _yaw0,double _lx,
               double _ly,double _ld,double _obs_d_min,double _obs_d_max,int _p_min,
               int _p_max)
    {
        /*randomObstacleFactory1(int _p_min, int _p_max, double _x_min, double _x_max,
                            double _y_min, double _y_max, double _d_min, double _d_max,
                            double _x0, double _y0, double _yaw0)
                            _x_min最小位置
        */
    obs_factory_ptr = new randomObstacleFactory1(_p_min, _p_max, 4., lx, 4., 
                        ly, _obs_d_min, _obs_d_max, x0, y0, yaw0);       
    }           
  ~sceneAstar() {
    obs_clear();
  }
  int obs_clear(){
    for(baseObstacle* & ob_ptr:obs){
        if(!ob_ptr){
            delete ob_ptr;
        }
    }
    obs.clear();
    return 0;
  }   

  int reset_prj(double _x0, double _y0, double _yaw0, double _lx, double _ly,
                double _ld) {
    x0 = _x0;
    y0 = _y0;
    yaw0 = _yaw0;
    lx = _lx;
    ly = _ly;
    ld = _ld;
    ild = 1.0 / _ld;//分辨率
    num_x=static_cast<size_t>(std::floor(lx*ild))+1;////向下取整数
    num_y=static_cast<size_t>(std::floor(ly*ild))+1;
    map0 = std::vector<std::vector<gridNode>>(num_x);
    for(size_t i=0;i<num_x;i++){
        for(size_t j=0;j<num_y;j++){
            Eigen::Vector2i idx0={i,j};
            map0[i].emplace_back(idx0,idx2pos(idx0));
        }
    }
    obs_clear();
    pos_start={x0,y0};
    pos_goal = map0[num_x-1][num_y-1].pos;

    return 0;

 }
  int reset_map() {
    for (size_t i = 0; i < num_x; ++i) {
      for (size_t j = 0; j < num_y; ++j) {
        map0[i][j].state = 0;
        map0[i][j].father = nullptr;
        map0[i][j].g = Inf;
        map0[i][j].f = Inf;
      }
    }
    return 0;
  }
    //索引转实际位置
  Eigen::Vector2d idx2pos(Eigen::Vector2i idx) {
    double lx1 = idx(0) * ld;
    double ly1 = idx(1) * ld;
    double x1 = x0 + lx1 * std::cos(yaw0) - ly1 * std::sin(yaw0);//旋转矩阵，绕Z轴
    double y1 = y0 + lx1 * std::sin(yaw0) + ly1 * std::cos(yaw0);
    return {x1, y1};
  }
  //实际位置转索引，逆矩阵
  Eigen::Vector2i pos2idx(Eigen::Vector2d pos){
    double dx=pos(0)-x0;
    double dy=pos(1)-y0;
    double lx1=dx*std::cos(yaw0)+dy*std::sin(yaw0);
    double ly1=dx*(-1)*std::sin(yaw0)+dy*std::cos(yaw0);
    int idx1=static_cast<int>(std::max(std::min(std::round(lx1*ild),num_x-1.0),0.0));
    int idy1=static_cast<int>(std::max(std::min(std::round(ly1*ild),num_y-1.0),0.0));
    return {idx1,idy1};
  }
  // check if a grid map index is collision free (overloading1)
  bool is_free(int ix, int iy) {
    if (ix < 0 || ix >= (int)num_x || iy < 0 || iy >= (int)num_y) {
      return false;
    }
    double x = map0[ix][iy].pos(0);
    double y = map0[ix][iy].pos(1);
    for (size_t k = 0; k < obs.size(); ++k) {
      if (!obs[k]->isfree(x, y)) {
        return false;
      }
    }
    return true;
  }

  // check if a grid map index is collision free (overloading2)
  bool is_free(Eigen::Vector2i idx) {
    int ix = idx(0);
    int iy = idx(1);
    return is_free(ix, iy);
  }
  // check if a position in real world is collision free (overloading1)
  bool is_free(double x, double y) {
    //TODO
    double cos_yaw = std::cos(yaw0);
    double sin_yaw = std::sin(yaw0);
    bool out1 = (x - x0) * sin_yaw - (y - y0) * cos_yaw >= 0;
    bool out2 =
        (x - x0 - lx * cos_yaw) * cos_yaw + (y - y0 - lx * sin_yaw) * sin_yaw >=
        0;
    bool out3 = -(x - x0 - lx * cos_yaw + ly * sin_yaw) * sin_yaw +
                    (y - y0 - lx * sin_yaw - ly * cos_yaw) * cos_yaw >=
                0;
    bool out4 = -(x - x0 + ly * sin_yaw) * cos_yaw -
                    (y - y0 - ly * cos_yaw) * sin_yaw >=
                0;
    if (out1 || out2 || out3 || out4) {
      return false;
    }
    for (size_t k = 0; k < obs.size(); ++k) {
      if (!obs[k]->isfree(x, y)) {
        return false;
      }
    }
    return true;
  }
  // check if a position in real world is collision free (overloading2)
  bool is_free(Eigen::Vector2d pos) {
    double x = pos(0);
    double y = pos(1);
    return is_free(x, y);
  }
  int random_obstacles(int _obs_num){
    obs_clear();
    obs_num=_obs_num;
    for(int k=0;k<_obs_num;++k){
        baseObstacle* ob_ptr;
        double dist_min;
        do{
            ob_ptr = obs_factory_ptr->create();
            dist_min=Inf;
            for(int i=0;i<static_cast<int>(obs.size());i++){
                double dist= obs[i]->distance(ob_ptr->x,ob_ptr->y);
                if(dist<dist_min) dist_min=dist;//最短距离判定
                std::cout << dist << std::endl;
            }
        }while(dist_min<1.15*ob_ptr->r);
        obs.emplace_back(ob_ptr);
    }
    return 0;

  }

  // set the goal position of path planning tasks
  int set_start(double _x, double _y) {
    pos_start(0) = _x;
    pos_start(1) = _y;
    return 0;
  }
  int set_goal(double _x, double _y) {
    pos_goal(0) = _x;
    pos_goal(1) = _y;
    return 0;
  }

  //TODO optimal
  double heu(gridNode* n1, gridNode* n2) {
    int d_ix = std::abs(n1->idx(0) - n2->idx(0));
    int d_iy = std::abs(n1->idx(1) - n2->idx(1));
    int diag = std::min(d_ix, d_iy);
    int parallel = std::max(d_ix, d_iy) - diag;
    return (diag * std::sqrt(2) + parallel);
  }

  int astar_search() {
    reset_map();
    std::multimap<double, gridNode*> opens;
    Eigen::Vector2i idx_start = pos2idx(pos_start);
    Eigen::Vector2i idx_goal = pos2idx(pos_goal);
    gridNode* node_start = &map0[idx_start(0)][idx_start(1)];//地图指针
    gridNode* node_goal = &map0[idx_goal(0)][idx_goal(1)];
    gridNode* node_current = nullptr;
    bool astar_path_found = 0;
    node_start->g = 0;
    node_start->f = heu(node_start, node_goal);
    node_start->state = 1;
    opens.insert({node_start->f, node_start});
    while (!opens.empty()) {
      node_current = opens.begin()->second;
      opens.erase(opens.begin());
      if (node_current->state == -1) {
        continue;
      }
      node_current->state = -1;
      if (node_current->idx == idx_goal) {
        astar_path_found = 1;
        break;
      }
      for (int d_ix = -1; d_ix <= 1; ++d_ix) {
        for (int d_iy = -1; d_iy <= 1; ++d_iy) {
          if (d_ix == 0 && d_iy == 0) continue; 
          int ix1 = d_ix + node_current->idx(0);
          int iy1 = d_iy + node_current->idx(1);
          if (!is_free(ix1, iy1) || map0[ix1][iy1].state == -1) continue;
          gridNode* node_neighbor = &map0[ix1][iy1];
          double edge = std::sqrt(d_ix * d_ix + d_iy * d_iy);
          double g = node_current->g + edge;
          double f =
              g + heu(node_neighbor, node_goal) * (1.0 + 1e-4);  // tie breaker
          if (node_neighbor->state == 0 ||
              (node_neighbor->state == 1 &&  // node unexplored
               node_neighbor->g > g)) {      // or node in open set with greater g
            node_neighbor->g = g;
            node_neighbor->f = f;
            node_neighbor->father = node_current;
            node_neighbor->state = 1;
            opens.insert({f, node_neighbor});
          }
        }
      }
    }
    if (astar_path_found) {
      double DIST_TOLERANCE = 0.6 * ld;
      grid_along_path.clear();//note 清楚容器中元素
      for (gridNode* n = node_current; n != nullptr; n = n->father) {
        grid_along_path.push_back(n);
      }
      size_t num = grid_along_path.size();
      //对于是否包含起始点和终止点
      if ((pos_start - grid_along_path[num - 1]->pos).norm() > DIST_TOLERANCE) {
        include_start = 1;
      } else {
        include_start = 0;
      }
      if ((pos_goal - grid_along_path[0]->pos).norm() > DIST_TOLERANCE) {
        include_goal = 1;
      } else {
        include_goal = 0;
      }
      astar_path.resize(num + include_start + include_goal, 2);
      std::cout<<"num + include_start + include_goal  "<<num + include_start + include_goal<<std::endl;
      astar_path.row(0) = pos_start;
      if (include_start) {
        astar_path.row(1) = grid_along_path[num - 1]->pos;
      }
      for (size_t i = num - 2; i >= 1; --i) {
        astar_path.row(num - i - 1 + include_start) = grid_along_path[i]->pos;
      }
      if (include_goal) {
        astar_path.row(num - 1 + include_start) = grid_along_path[0]->pos;
      }
      astar_path.row(num - 1 + include_start + include_goal) = pos_goal;
    }

    return !astar_path_found;
  }
  // random gennerate an Astar path
  int random_astar_path() {
    double dist_min = 0.7 * (map0[0][0].pos - map0[num_x - 1][num_y - 1].pos).norm();
    std::random_device dev;
    std::mt19937 re(dev());
    std::uniform_int_distribution<int> rand_x(0, num_x - 1);
    std::uniform_int_distribution<int> rand_y(0, num_y - 1);
    double dist = 0, x0, y0, x1, y1;
    bool astar_state;
    do {
      do {
        x0 = rand_x(re);
        y0 = rand_y(re);
        x1 = rand_x(re);
        y1 = rand_y(re);
        dist = (map0[x0][y0].pos - map0[x1][y1].pos).norm();
        // std::cout << "dist: " << dist << " occu_start:" << !is_free(map0[x0][y0].pos) << " occu_end:" << !is_free(map0[x1][y1].pos) << std::endl;
      } while (dist < dist_min || !is_free(map0[x0][y0].pos) || !is_free(map0[x1][y1].pos));
      // std::cout << dist_min/0.7 <<"[goal x0: " << x0 << ", y0: " << y0 << ", x1: " << x1 << ", y1: " << y1 << std::endl;
      // set_start(map0[x0][y0].pos(0), map0[x0][y0].pos(1));
      // set_goal(map0[x1][y1].pos(0), map0[x1][y1].pos(1));
      astar_state = astar_search();
    } while (astar_state);
    return astar_state;
  }

 // get value and grad info in distance field
  double dist_field(const double x1, const double y1, Vec* grad = nullptr) {
    double d0 = Inf, d1 = Inf;
    Vec g1(2);
    *grad << 0., 0.;
    #if 1
    for (int k = 0; k < static_cast<int>(obs.size()); ++k) {
      d1 = obs[k]->distance(x1, y1, &g1);
      if (d1 < d0) {//距离障碍物最小距离
        d0 = d1;
        if (grad != nullptr) {
          grad->operator()(0) = g1(0);
          grad->operator()(1) = g1(1);
        }
      }
    }
    #else
    #endif
    return d0;
  }

};






#endif /* A2E3B3C6_29AA_43CA_98A9_E22FD21E4C22 */
