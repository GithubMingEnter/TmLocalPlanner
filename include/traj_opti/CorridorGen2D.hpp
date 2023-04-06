/**************************************************************************
 * CorridorGen2D.hpp
 *
 * @Author： bornchow
 * @Date: 2022.12.26
 *
 * @Description:
 * 2-D corridor generation
 *
 * - eigen lib
 * 参考程序:
 *  ****************************************************/
#pragma once
#include <ros/ros.h>
#include <iostream>
#include <map/GridMap.hpp>
#include <vis/visualization.hpp>
#include "sdqp/sdqp.hpp"
using Emx=Eigen::MatrixXd;

using Evx=Eigen::VectorXd;
using Ev2=Eigen::Vector2d;
using Ev3=Eigen::Vector3d;
typedef struct Rectangle
{               // TODO 嵌入结构体
    Emx vertex; //// 矩形相当于4乘3的数组   vertex:4x3

    Ev3 center;
    bool isValid; // Does the corridor delete
    double t;     // time allocated to this Rectangle
    Rectangle *cross_rect;
    std::vector<std::pair<double, double>> box;

    Eigen::Matrix<double, 2, 2> Q;
    Eigen::Matrix<double, 2, 1> c;
    Eigen::Matrix<double, 2, 1> x;
    Emx A;
    Evx b;    
    int m=4; //edage num

    Rectangle(Emx vertex_)
    {
        vertex = vertex_;
        isValid = true;
        t = 0.0;
        cross_rect = nullptr;
        setBox();
    }
    Rectangle() 
    // :A(m,2),b(m)  //列表初始化不可行？
    {
        center = Evx::Zero(3);
        vertex = Emx::Zero(4, 3);
        Q<<2,0,
           0,2;
        A=Emx::Zero(m,2);
        b=Evx::Zero(m);
        isValid = true;
        t = 0.0;
        box.resize(2);
        cross_rect = nullptr;
    }
     /*
        p0--------p1   ^ y
        |         |    |
        |         |    |
        |         |    |
        p3--------p2   -----------> x

        vertex(0, 0) is p0.x

    */
    //    TODO reference transfer
    void setVertex(Emx vertex_, double resolution_)
    {
        vertex = vertex_;
        double res_2 = resolution_ / 2;
        // x
        vertex(1, 0) += res_2;
        vertex(2, 0) += res_2;

        vertex(0, 0) -= res_2;
        vertex(3, 0) -= res_2;

        // y
        vertex(0, 1) += res_2;
        vertex(1, 1) += res_2;

        vertex(3, 1) -= res_2;
        vertex(2, 1) -= res_2;

        setBox();
    }
    void setBox()
    {
        box.clear();
        box.resize(2);
        box[0] = std::make_pair(vertex(0, 0), vertex(1, 0)); // x方向的长度
        box[1] = std::make_pair(vertex(0, 1), vertex(3, 1)); // y方向的长度

        center(0) = (vertex(1, 0) + vertex(0, 0)) / 2.0;
        center(1) = (vertex(0, 1) + vertex(3, 1)) / 2.0;
        center(2) = 0;
    }   
    bool isfree(const double x1, const double y1) {
        x(0) = x1;
        x(1) = y1;
        double dxx=(A * x - b).maxCoeff();
        return (A * x - b).maxCoeff() > 0;
    }
    void setAb(){
        int j=m-1;
        // ROS_INFO_STREAM("XXXXXX2"); 
        for (int k = 0; k < m; ++k) {
            // ROS_INFO_STREAM("XXXXXX2"); 
        // 边对应起始点 一般式：Ax+By+C=O（AB≠0）
            A(k, 0) = vertex(k, 1) - vertex(j, 1);
            A(k, 1) = vertex(j, 0) - vertex(k, 0);
            b(k) = A(k, 0) * vertex(j, 0) + A(k, 1) * vertex(j, 1);
            j = k;
        }
    }
    double distance(const double x1, const double y1, Evx* grad = nullptr) {
        
     if (isfree(x1, y1)) {
        c(0) = -2 * x1;
        c(1) = -2 * y1;
        sdqp::sdqp<2>(Q, c, A, b, x);//SDQP求解
        double d, d2 = (x1 - x(0)) * (x1 - x(0)) + (y1 - x(1)) * (y1 - x(1));
        if (d2 > 1e-6) {//有效free space 
            d = std::sqrt(d2);
            if (grad != nullptr) {
            grad->operator()(0) = (x1 - x(0)) / d;//vector 指针对应位置梯度
            grad->operator()(1) = (y1 - x(1)) / d;
            }
        } else {
            d = 0;
            if (grad != nullptr) {
            grad->operator()(0) = grad->operator()(1) = 0;
            }
        }
        // std::cout << "[x1:" << x1 << " y1:" << y1 << " x:" << x(0) << " y:" << x(1) << " d:" << d << "]" << std::endl;
        return d;
        } else {
            // double d = 0;
            // if (grad != nullptr) {
            //   grad->operator()(0) = grad->operator()(1) = 0;
            // }
            double d = 1e20;
            // 计算点到各个边的距离，比较
            for (int k = 0; k < m; ++k) {
                double la = A(k, 0);//la*x+lb*y+lc0
                double lb = A(k, 1);
                double lc = -b(k);
                double l2 = la * la + lb * lb;
                double d1 = std::abs(la * x1 + lb * y1 + lc) / std::sqrt(l2);
                if (d1 < d) {
                d = d1;
                if (grad != nullptr) {
                    //点到直线的垂足到(x0,y0)对应的(x,y)分量
                    double dx = (lb * (lb * x1 - la * y1) - la * lc) / l2 - x1;
                    double dy = (la * (la * y1 - lb * x1) - lb * lc) / l2 - y1;
                    double norm_dxdy = std::max(std::sqrt(dx * dx + dy * dy), 0.01);//note 0.01
                    grad->operator()(0) = dx / norm_dxdy;//与中心距离的比值
                    grad->operator()(1) = dy / norm_dxdy;
                    cout << grad->operator()(0) << "  " << grad->operator()(1) << endl;
                }
                }
            }

        return -d;
        }
    }
        ~Rectangle() {}
}
Rectangle;





class CorridorGen2D
{
private:
    ros::NodeHandle n_;
    std::shared_ptr<env::GridMap> env_ptr_;
    std::shared_ptr<vis::Visualization> vis_ptr_;

    int max_iter_nu_, inflat_len_per_iter_;
    double res_;
    bool is_debug_;
    double MAX_Vel_, MAX_Acc_;
    Ev3 start_vel_;
    Ev3 start_point_, end_point_;
    std::vector<Rectangle> last_rect_list;

    std::pair<Rectangle, bool> inflateRect(Rectangle rect, Rectangle last_rect);
    bool isContain(Rectangle this_rect, Rectangle last_rect);
    bool isRectCross(Rectangle r1, Rectangle r2);

    void calRectCross(Rectangle r1, Rectangle r2, Rectangle *r3);

    std::vector<Rectangle> simpleCorridor(std::vector<Rectangle> rect_list);

    void timeAllocation(std::vector<Rectangle> &corridor);

    bool isPointInCorridor(Eigen::Vector3d p, Rectangle rect);

public:
    CorridorGen2D(ros::NodeHandle nh,
                  std::shared_ptr<env::GridMap> &envPtr,
                  std::shared_ptr<vis::Visualization> &visPtr,
                  Ev3 start_vel);
    ~CorridorGen2D(){};
    Rectangle generateRect(Ev3 pt);
    std::vector<Rectangle> corridorGeneration(std::vector<Ev3> path);
    void distance(const double x1,const double y1, Evx* grad);

};

#include "traj_opti/CorridorGen2D.hpp"

CorridorGen2D::CorridorGen2D(ros::NodeHandle nh,
                             std::shared_ptr<env::GridMap> &envPtr,
                             std::shared_ptr<vis::Visualization> &visPtr,
                             Ev3 start_vel) : n_(nh),
                                              env_ptr_(envPtr),
                                              vis_ptr_(visPtr),
                                              start_vel_(start_vel)
{
    n_.param<int>("corridor/max_iter_nu", max_iter_nu_, 5);
    n_.param<int>("corridor/inflat_len_per_iter", inflat_len_per_iter_, 3);
    n_.param<double>("corridor/max_vel", MAX_Vel_, 1.0);
    n_.param<double>("corridor/max_acc", MAX_Acc_, 0.5);
    n_.param<bool>("corridor/is_debug", is_debug_, false);

    res_ = env_ptr_->getResolution();
    ROS_INFO("\033[1; This is 2D corridor]---------.\033");

    ROS_WARN_STREAM("[corridor] param | max_iter_number    : " << max_iter_nu_);
    ROS_WARN_STREAM("[corridor] param | inflat_len_per_iter: " << inflat_len_per_iter_ * res_);
    ROS_WARN_STREAM("[corridor] param | map resolution     : " << res_);
    ROS_WARN_STREAM("[corridor] param | max vel            : " << MAX_Vel_);
    ROS_WARN_STREAM("[corridor] param | max acc            : " << MAX_Acc_);
    ROS_WARN_STREAM("[corridor] param | is debug           : " << is_debug_);
}

void CorridorGen2D::distance(const double x1,const double y1, Evx* grad=nullptr)
{
    int m=4;
    Emx A(m,2);
    Emx poly(m,4);
    Evx b(m);
    poly<<1,1,
          5,1,
          5,4,
          1,4;
    
    int j=4-1;//尾部
    for(int k=0;k<m;++k){
        // 边对应起始点 一般式：Ax+By+C=O（AB≠0）
        A(k,0)=poly(k,1)-poly(j,1);
        A(k,1)=poly(j,0)-poly(k,0);
        b(k)=A(k,0)*poly(j,0)+A(k,1)*poly(j,1);
        j=k;
    }


}



std::pair<Rectangle, bool> CorridorGen2D::inflateRect(Rectangle rect, Rectangle last_rect)
{
    Rectangle rectMax = rect;
    Emx vertex_idx(4, 3); // 4 vertex
    vertex_idx = rect.vertex;
    /*
    p0--------p1   ^ y
    |         |    |
    |         |    |
    |         |    |
    p3--------p2   -----------> x
*/
    double id_x, id_y;
    int iter = 1;
    bool collide;

    while (iter <= max_iter_nu_)
    {
        if (is_debug_)
            ROS_INFO_STREAM(" ======================================== -- " << iter);
        /*****x+
                p1
                |
                |
                |
                p2
         * *******/
        collide = false;
        double x_up = vertex_idx(1, 0) + inflat_len_per_iter_ * res_;
        if (is_debug_)
            ROS_INFO_STREAM("[X+] p1.x --> x_up: " << vertex_idx(1, 0) << " --> " << x_up);

        for (id_x = vertex_idx(1, 0); id_x <= x_up; id_x += res_)
        {
            if (collide)
                break;
            // p2.y-->p1.y
            for (id_y = vertex_idx(2, 1); id_y <= vertex_idx(1, 1); id_y += res_)
            {
                if (collide)
                    break;

                Ev3 p(id_x, id_y, 0);
                if (!env_ptr_->isStateValid(p))
                {
                    collide = true;
                    break;
                }
            }
        }
        if (collide)
        {
            // if(is_debug_) ROS_INFO_STREAM("[X+]    ------  collide" );
            vertex_idx(1, 0) = max(id_x - 2 * res_, vertex_idx(1, 0));
            vertex_idx(2, 0) = max(id_x - 2 * res_, vertex_idx(2, 0));
        }
        else
        {
            // if(is_debug_) ROS_INFO_STREAM("[X+]   ------  no collide" );
            vertex_idx(1, 0) = vertex_idx(2, 0) = id_x - res_;
        }
        if (is_debug_)
            ROS_INFO_STREAM("[X+] ----p1.x x+ after inflate: " << vertex_idx(1, 0));

        /*****x-
              p0
                |
                |
                |
                p3
         * *******/

        collide = false;
        double x_lo = vertex_idx(0, 0) - inflat_len_per_iter_ * res_;
        if (is_debug_)
            ROS_INFO_STREAM("[X-] p0.x --> x_lo: " << vertex_idx(0, 0) << " --> " << x_lo);

        for (id_x = vertex_idx(0, 0); id_x >= x_lo; id_x -= res_)
        {
            if (collide)
                break;
            // p0.y-->p3.y 逆时针
            for (id_y = vertex_idx(0, 1); id_y >= vertex_idx(3, 1); id_y -= res_)
            {
                if (collide)
                    break;

                Ev3 p(id_x, id_y, 0);
                if (!env_ptr_->isStateValid(p))
                {
                    collide = true;
                    break;
                }
            }
        }
        if (collide)
        {
            // if(is_debug_) ROS_INFO_STREAM("[X+]    ------  collide" );
            vertex_idx(0, 0) = min(id_x + 2 * res_, vertex_idx(0, 0));
            vertex_idx(3, 0) = min(id_x + 2 * res_, vertex_idx(3, 0));
        }
        else
        {
            // if(is_debug_) ROS_INFO_STREAM("[X+]   ------  no collide" );
            vertex_idx(0, 0) = vertex_idx(3, 0) = id_x + res_;
        }
        if (is_debug_)
            ROS_INFO_STREAM("[X-] ----p0.x x- after inflate: " << vertex_idx(0, 0));
        //////////       Y+  p0 -- p1
        //*****************************************************
        collide = false;
        double y_up = vertex_idx(0, 1) + inflat_len_per_iter_ * res_;
        if (is_debug_)
            ROS_INFO_STREAM("[Y+] p0.y --> y_up: " << vertex_idx(0, 1) << " --> " << y_up);
        // p0.y - >y_up
        for (id_y = vertex_idx(0, 1); id_y <= y_up; id_y += res_)
        {
            if (collide)
            {
                break;
            }
            
            // p1.x-->p0.x
            for (id_x = vertex_idx(1, 0); id_x >= vertex_idx(0, 0); id_x -= res_)
            {
                if (collide)
                {
                    break;
                }
                Ev3 p(id_x, id_y, 0);
                if (!env_ptr_->isStateValid(p))
                {
                    collide = true;
                    break;
                }
            }
        }
        if (collide)
        {
            if (is_debug_)
                ROS_INFO_STREAM("[Y+]   ------  collide");
            // 更新p0,p1的y方向边界在上一次迭代与id_y-2*res_选择一个最大的
            vertex_idx(0, 1) = max(id_y - 2 * res_, vertex_idx(0, 1));
            vertex_idx(1, 1) = max(id_y - 2 * res_, vertex_idx(1, 1));
        }
        else
        {
            if (is_debug_)
                ROS_INFO_STREAM("[Y+]   ------  no collide");
            vertex_idx(0, 1) = vertex_idx(1, 1) = id_y - res_;
        }

        if (is_debug_)
            ROS_INFO_STREAM("[Y+] ----p0.y after inflate: " << vertex_idx(0, 1));
        //////////       Y  p3 -- p2
        //*****************************************************
        collide = false;
        double y_lo = vertex_idx(3, 1) - inflat_len_per_iter_ * res_;
        if (is_debug_)
            ROS_INFO_STREAM("[Y-] p3.y --> y_lo: " << vertex_idx(3, 1) << " --> " << y_lo);
        // p3.y - >y_lo
        for (id_y = vertex_idx(3, 1); id_y >= y_lo; id_y -= res_)
        {
            if (collide)
            {
                break;
            }
           
            // p2.x--> p3.x
            for (id_x = vertex_idx(2, 0); id_x >= vertex_idx(3, 0); id_x -= res_)
            {
                if (collide)
                {
                    break;
                }
                Ev3 p(id_x, id_y, 0);
                if (!env_ptr_->isStateValid(p))
                {
                    collide = true;
                    break;
                }
            }
        }
        if (collide)
        {
            if (is_debug_)
                ROS_INFO_STREAM("[Y-]   ------  collide");
            vertex_idx(3, 1) = min(id_y + 2 * res_, vertex_idx(3, 1));
            vertex_idx(2, 1) = min(id_y + 2 * res_, vertex_idx(2, 1));
        }
        else
        {
            if (is_debug_)
                ROS_INFO_STREAM("[Y-]   ------  no collide");
            vertex_idx(3, 1) = vertex_idx(2, 1) = id_y + res_;
        }

        if (is_debug_)
            ROS_INFO_STREAM("[Y-] ----p3.y after inflate: " << vertex_idx(3, 1));

        rectMax.setVertex(vertex_idx, res_);
        if (iter == 1)
        {
            // 检查当前膨胀出来的rectMax是否包含在last_rect中
            // 这种方法只能用在路径点密集的情况
            if (isContain(rectMax, last_rect))
            {
                if (is_debug_)
                {
                    ROS_INFO_STREAM(" ---- contain check-----");
                    return make_pair(rectMax, false);
                }
            }
        }
        iter++;
    }
    return make_pair(rectMax, true);
}
/*
    p0--------p1   ^ y
    |         |    |
    |         |    |
    |         |    |
    p3--------p2   -----------> x
*/
bool CorridorGen2D::isContain(Rectangle this_rect, Rectangle last_rect)
{
    if (last_rect.vertex(0, 0) <= this_rect.vertex(0, 0) && last_rect.vertex(0, 1) >= this_rect.vertex(0, 1) &&
        last_rect.vertex(2, 0) >= this_rect.vertex(2, 0) && last_rect.vertex(2, 1) <= this_rect.vertex(2, 1))
    {
        return true; // this_rect in last_rect
    }
    else
    {
        return false;
    }
}
std::vector<Rectangle> CorridorGen2D::corridorGeneration(std::vector<Ev3> path)
{
    ros::Time corridor_start_time = ros::Time::now();
    // recored
    start_point_ = path.front();
    end_point_ = path.back();

    std::vector<Rectangle> rect_list;
    Rectangle last_rect;
    for (size_t i = 0; i < path.size(); i++)
    {
        Rectangle rect = generateRect(path[i]);
        auto result = inflateRect(rect, last_rect);
        if (!result.second)
        {
            continue;
        }
        rect_list.emplace_back(result.first);
        last_rect = result.first;
    }
    std::vector<Rectangle> rect_list_new = simpleCorridor(rect_list);
    // cross rect
    for (size_t i = 1; i < rect_list_new.size(); i++)
    {
        Rectangle *cross_rect = new Rectangle();
        calRectCross(rect_list_new[i - 1], rect_list_new[i], cross_rect);
        rect_list_new[i - 1].cross_rect = cross_rect;
    }
    timeAllocation(rect_list_new);
    if (is_debug_)
    {
        ROS_INFO_STREAM(" corridor time allocation: ");
        for (size_t i = 0; i < rect_list_new.size(); i++)
        {
            std::cout << " " << rect_list_new[i].t;
        }
        std::cout << std::endl;
    }
    ROS_INFO_STREAM("\033[1;32m[corridor] | corridor size: " << rect_list_new.size() << ".\033[0m");
    ROS_INFO_STREAM("\033[1;32m[corridor] | corridor gen time: " << (ros::Time::now() -
                                                                     corridor_start_time)
                                                                            .toSec() *
                                                                        1000
                                                                 << " (ms) \033[0m");
    last_rect_list.assign(rect_list_new.begin(),rect_list_new.end()); //赋值
    return rect_list_new;
}
/*
  如何高效的判断两个矩形相交?
  在本示例中，矩形在坐标系中没有旋转
  refer: https://zhuanlan.zhihu.com/p/526649229
  p3 left_down  p1 right_up
  in x aixs: max(r1.p3.x, r2.p3.x) <= min(r1.p1.x, r2.p1.x)
  in y aixs: max(r1.p3.y, r2.p3.y) <= min(r1.p1.y, r2.p1.y))

    p0--------p1   ^ y
    |         |    |
    |         |    |
    |         |    |
    p3--------p2   -----------> x

  相交矩形:
  [max(r1.p3.x, r2.p3.x), max(r1.p3.y, r2.p3.y)] new left_down p3
  [min(r1.p1.x, r2.p1.x), min(r1.p1.y, r2.p1.y)] new right_up p1
*/

bool CorridorGen2D::isRectCross(Rectangle r1, Rectangle r2)
{
    if (max(r1.vertex(3, 0), r2.vertex(3, 0)) < min(r1.vertex(1, 0), r2.vertex(1, 0)) &&
        max(r1.vertex(3, 1), r2.vertex(3, 1)) < min(r1.vertex(1, 1), r2.vertex(1, 1)))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void CorridorGen2D::calRectCross(Rectangle r1, Rectangle r2, Rectangle *r3)
{
    if (max(r1.vertex(3, 0), r2.vertex(3, 0)) < min(r1.vertex(1, 0), r2.vertex(1, 0)) &&
        max(r1.vertex(3, 1), r2.vertex(3, 1)) < min(r1.vertex(1, 1), r2.vertex(1, 1)))
    {
        double p3_x = max(r1.vertex(3, 0), r2.vertex(3, 0));
        double p3_y = max(r1.vertex(3, 1), r2.vertex(3, 1));
        double p1_x = min(r1.vertex(1, 0), r2.vertex(1, 0));
        double p1_y = min(r1.vertex(1, 1), r2.vertex(1, 1));

        r3->vertex.row(0) = Ev3(p3_x, p1_y, 0);
        r3->vertex.row(1) = Ev3(p1_x, p1_y, 0);
        r3->vertex.row(2) = Ev3(p1_x, p3_y, 0);
        r3->vertex.row(3) = Ev3(p3_x, p3_y, 0);
        r3->setBox();
    }
}
std::vector<Rectangle> CorridorGen2D::simpleCorridor(std::vector<Rectangle> rect_list)
{
    Rectangle this_rect = rect_list[0];
    std::vector<Rectangle> new_rect_list;
    new_rect_list.emplace_back(this_rect);

    for (size_t i = 1; i < rect_list.size(); i++)
    {
        if (!isRectCross(this_rect, rect_list[i]))
        {
            this_rect = rect_list[i - 1];
            new_rect_list.emplace_back(this_rect);
        }
    }
    if (!isPointInCorridor(end_point_, new_rect_list.back()))
        new_rect_list.emplace_back(rect_list.back());
    if (isPointInCorridor(start_point_, new_rect_list[1]))
        new_rect_list.erase(new_rect_list.begin());
    if (is_debug_)
        ROS_INFO_STREAM("[corridor] | the start point in second rect: " << isPointInCorridor(start_point_, new_rect_list[1]));

    return new_rect_list;
}

/*
    p0--------p1   ^ y
    |         |    |
    |         |    |
    |         |    |
    p3--------p2   -----------> x
*/
Rectangle CorridorGen2D::generateRect(Ev3 pt)
{
    Rectangle rect;
    rect.center(0) = pt(0);
    rect.center(1) = pt(1);
    rect.center(2) = pt(2);
    // double x_up = pt(0) + res_;
    // double x_lo = pt(0) - res_;

    // double y_up = pt(1) + res_;
    // double y_lo = pt(1) - res_;
    // x
    double x_up = pt(0);
    double x_lo = pt(0);

    double y_up = pt(1);
    double y_lo = pt(1);

    rect.vertex.row(0) = Ev3(x_lo, y_up, 0);
    rect.vertex.row(1) = Ev3(x_up, y_up, 0);
    rect.vertex.row(2) = Ev3(x_up, y_lo, 0);
    rect.vertex.row(3) = Ev3(x_lo, y_lo, 0);

    rect.setBox();
    return rect;
}
/*以p0,p2为两个坐标轴原点，根据向量内积进行判断*/
bool CorridorGen2D::isPointInCorridor(Ev3 p, Rectangle rect)
{
    Ev3 p0p3 = rect.vertex.row(3) - rect.vertex.row(0);
    Ev3 p0p = p.transpose() - rect.vertex.row(0);
    Ev3 p0p1 = rect.vertex.row(1) - rect.vertex.row(0);
    Ev3 p2p1 = rect.vertex.row(1) - rect.vertex.row(2);
    Ev3 p2p = p.transpose() - rect.vertex.row(2);
    Ev3 p2p3 = rect.vertex.row(3) - rect.vertex.row(2);

    return p0p3.dot(p0p) >= 0 && p0p1.dot(p0p) >= 0 &&
           p2p1.dot(p2p) >= 0 && p2p3.dot(p2p) >= 0;
}

void CorridorGen2D::timeAllocation(std::vector<Rectangle> &corridor)
{
    std::vector<Ev3> points;

    points.emplace_back(start_point_);

    for (size_t i = 0; i < corridor.size(); i++)
    {
        if (corridor[i].cross_rect)
        { // 指针判断
            points.emplace_back(corridor[i].cross_rect->center);
        }
    }
    points.emplace_back(end_point_);

    double _Vel = MAX_Vel_ * 0.6;
    double _Acc = MAX_Acc_ * 0.6;
    for (int k = 0; k < (int)points.size() - 1; k++)
    {
        double dtxyz;
        Ev3 p0 = points[k];
        Ev3 p1 = points[k + 1];
        Ev3 d = p1 - p0;
        Ev3 v0(0.0, 0.0, 0.0);
        if (k == 0)
        {
            v0 = start_vel_;
        }
        double D = d.norm();
        double V0 = v0.dot(d / D);
        double fV0 = fabs(V0);

        double acct = (_Vel - V0) / _Acc * ((_Vel > V0) ? 1 : -1);
        double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0) ? 1 : -1);
        double dcct = _Vel / _Acc;
        double dccd = _Acc * dcct * dcct / 2;

        if (D < fV0 * fV0 / (2 * _Acc)) //<末端速度为0
        {
            double t1 = (V0 < 0) ? 2.0 * fV0 / _Acc : 0.0;
            double t2 = fV0 / _Acc;
            dtxyz = t1 + t2;
        }
        else if (D < accd + dccd) // 速
        {
            double t1 = (V0 < 0) ? 2.0 * fV0 / _Acc : 0.0; // 减速为-
            double t2 = (-fV0 + sqrt(fV0 * fV0 + _Acc * D - fV0 * fV0 / 2)) / _Acc;
            double t3 = (fV0 + _Acc * t2) / _Acc;
            dtxyz = t1 + t2 + t3;
        }
        else
        {
            double t1 = acct;
            double t2 = (D - accd - dccd) / _Vel;
            double t3 = dcct;
            dtxyz = t1 + t2 + t3;
        }
        corridor[k].t = dtxyz;
    }
}

