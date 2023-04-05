#include "traj_opti/CorridorGen2D.hpp"
#include "sdqp/sdqp.hpp"
struct test_dist{


    Eigen::Matrix<double, 2, 1> x;
    Eigen::Matrix<double, 2, 2> Q;
    Eigen::Matrix<double, 2, 1> c;
    Emx A;
    Evx b;
    int m;
    Emx poly;
public:
    test_dist( const Emx& _poly):
        m(_poly.rows()),Q (Emx::Identity(2, 2) * 2) ,A(m,2),b(m),poly(_poly)
    {
        ROS_INFO("INIT DIST");
        int j = m - 1; // 尾部
        for (int k = 0; k < m; ++k)
        {
            // 边对应起始点 一般式：Ax+By+C=O（AB≠0）
            A(k, 0) = poly(k, 1) - poly(j, 1);
            A(k, 1) = poly(j, 0) - poly(k, 0);
            b(k) = A(k, 0) * poly(j, 0) + A(k, 1) * poly(j, 1);
            j = k;
            ROS_INFO_STREAM(A(k,0));
            ROS_INFO_STREAM(b(k));
        }
        // std::cout<<A<<std::endl<< " "<<b<<std::endl;
        

    }
    ~test_dist(){};
    double distance(const double x1, const double y1, Evx *grad)
    {

        // return (A*x-b).maxCoeff()>0;
        if ( isfree(x1,y1))
        {
            ROS_INFO_STREAM("XXXXXX1"); 
            c(0) = -2 * x1;
            c(1) = -2 * y1;
            sdqp::sdqp<2>(Q, c, A, b, x);
            double d;
            double d2 = (x1 - x(0)) * (x1 - x(0)) + (y1 - x(1)) * (y1 - x(1));
            if (d2 >= 1e-6)
            {
                d = std::sqrt(d2);
                if (grad != nullptr)
                {
                    grad->operator()(0) = (x1 - x(0)) / d;
                    grad->operator()(1) = (y1 - x(1)) / d;
                }
                else
                {
                    grad->operator()(0) = grad->operator()(1) = 0;
                }
                cout << grad->operator()(0) << "  " << grad->operator()(1) << endl;
            }
            // return -1000;//外部点
            return -d;
        }
        else
        {
            ROS_INFO_STREAM("XXXXXX2"); 
            double d = 1e20;
            // 计算点到各个边的距离，进行比较
            for (int k = 0; k < m; ++k)
            {
                double la = A(k, 0); // la*x+lb*y+lc0
                double lb = A(k, 1);
                double lc = -b(k);
                double l2 = la * la + lb * lb; // a^2+b^2
                double d1 = std::abs(la * x1 + lb * y1 + lc) / std::sqrt(l2);
                if (d1 < d)
                {
                    d = d1;
                    if (grad != nullptr)
                    {
                        // 点到直线的垂足到(x0,y0)对应的(x,y)分量
                        double dx = (lb * (lb * x1 - la * y1) - la * lc) / l2 - x1;
                        double dy = (la * (la * y1 - lb * x1) - lb * lc) / l2 - y1;
                        double norm_dxdy = std::max(std::sqrt(dx * dx + dy * dy), 0.01); // note 0.01
                        // 避免分母
                        grad->operator()(0) = dx / norm_dxdy;
                        grad->operator()(1) = dy / norm_dxdy;
                        cout << grad->operator()(0) << "  " << grad->operator()(1) << endl;
                    }
                }
            }
            return d;
        }
    }
    bool isfree(const double x1, const double y1)
    {
           
        x(0) = x1;                    
        x(1) = y1;       
     
        
        std::cout<< " "<<b(0)<<std::endl;

        
        return (A * x - b).maxCoeff()>0;
    }
    
};
typedef Eigen::MatrixXd Mat;
typedef Eigen::VectorXd Vec;

// base classes of obstacles
struct baseObstacle {
  double x;
  double y;
  double r; //diameter or 
  int type; //0 cylinderObstacle 1 convex obstacle
  virtual bool isfree(const double x1, const double y1) = 0;//1 is free
  virtual double distance(const double x1, const double y1, Vec* grad = nullptr) = 0;
  baseObstacle(int _t, double _x, double _y, double _r) : x(_x), y(_y), r(_r), type(_t) {}
  virtual ~baseObstacle() {}
};

// classes for poly prism obstacle
struct polyObstacle : public baseObstacle {
  int m;
  Mat poly;
  Eigen::Matrix<double, 2, 2> Q;
  Eigen::Matrix<double, 2, 1> c;
  Eigen::Matrix<double, 2, 1> x;
  Mat A;
  Vec b;
  polyObstacle(const Mat& _poly, const double _x = 0, const double _y = 0, const double _r = 1)
      : baseObstacle(1, _x, _y, _r), m(_poly.rows()),
       poly(_poly), Q(Mat::Identity(2, 2) * 2), A(m, 2), b(m) {
    int j = m - 1;//尾部开始
    for (int k = 0; k < m; ++k) {
      // 边对应起始点 一般式：Ax+By+C=O（AB≠0）
      A(k, 0) = poly(k, 1) - poly(j, 1);
      A(k, 1) = poly(j, 0) - poly(k, 0);
      b(k) = A(k, 0) * poly(j, 0) + A(k, 1) * poly(j, 1);
      j = k;
    }
    // std::cout << "[random poly object OK!]" << std::endl;
  }

  ~polyObstacle(){};
  bool isfree(const double x1, const double y1) {
    x(0) = x1;
    x(1) = y1;
    return (A * x - b).maxCoeff() > 0;
  }
  double distance(const double x1, const double y1, Vec* grad = nullptr) {
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
            double norm_dxdy = std::min(std::sqrt(dx * dx + dy * dy), 0.01);//note 0.01
            grad->operator()(0) = dx / norm_dxdy;//与中心距离的比值
            grad->operator()(1) = dy / norm_dxdy;
          }
        }
      }

      return d;
    }
  }
};
void test_obs(){
    Emx poly(4,2);

    poly << 1, 1,
            5, 1,
            5, 4,
            1, 4;
    baseObstacle* obs_ptr;
 
    obs_ptr=new polyObstacle(poly,0,0,0);
    Evx g1(2);
    g1 << 0, 0;
    double free0 = obs_ptr->distance(100, 100, &g1);
    ROS_INFO_STREAM("(0,3) is " << free0);
    std::vector<baseObstacle*> obs;
    obs.emplace_back(obs_ptr);
    double free1 = obs[0]->distance(1, 1, &g1);
    ROS_INFO_STREAM("(1,1) is " << free1);

    Emx poly1(4,2);
    poly1<<5,4,
           8,4,
           8,8,
           5,8;
    obs_ptr=new polyObstacle(poly1,0,0,0);       
    double d0=1e6,d1=1e6;
    for(int k=0;k<static_cast<int>(obs.size());k++){
        d1= obs[k]->distance(1, 1, &g1);
        
    }


    // test_dist obs1(poly);
    
    //  free0 = obs1.distance(100, 100, &g1);
    // ROS_INFO_STREAM("(0,3) is " << free0);
    //     double free1 = obs1.distance(1, 1, &g1);
    // ROS_INFO_STREAM("(1,1) is " << free1);
    // double free2 = obs1.distance(2, 3, &g1);
    // ROS_INFO_STREAM("(2,3) is " << free2);
    //  double free3 = obs1.distance(3, 2.5, &g1);
    // ROS_INFO_STREAM("(3,2.5) is " << free2);
    // delete obs1;
}

Rectangle* createRect(Emx & poly)
{
    Rectangle* rect1=new Rectangle();
 
    rect1->setVertex(poly,0.05);
 
    rect1->setAb();
    return rect1;
}
void testRect(double xt,double yt,Vec* grad)
{
    double d0=1e6,d1=1e6;
    Emx poly(4,3);

    poly << 1, 1,0,
            5, 1,0,
            5, 4,0,
            1, 4,0;

 
    Emx poly1(4,3);
    poly1<<3,2,0,
           8,2,0,
           8,8,0,
           3,8, 0  ;
    Rectangle* rect1=createRect(poly1);
    std::vector<Rectangle*> obs;
    obs.emplace_back(rect1);
    rect1=createRect(poly);
    obs.emplace_back(rect1);
    Evx g1(2);
    g1 << 0, 0;
    //1.如果有点在任意一个矩形内则 free
    // 2.如果在交错矩形中，那么以交错矩形作为构造
    for(int k=0;k<static_cast<int>(obs.size());k++){
      d1= obs[k]->distance(xt, yt, &g1);
      ROS_INFO_STREAM("D1 ="<<d1);
      if (d1 < d0) {//距离障碍物最小距离 内部为负，但需要取相反数
        d0 = d1;
        if (grad != nullptr) {
          grad->operator()(0) = g1(0);
          grad->operator()(1) = g1(1);
        }
        cout <<"update grad "<< grad->operator()(0) << "  " << grad->operator()(1) << endl;
      }
    }
    ROS_INFO_STREAM("----------------D0 ="<<d0);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    // test_obs();
    Vec dist_grad(2);
    

    testRect(2,3,&dist_grad);
    cout<<"testRect(3,2,&dist_grad)"<<endl;
    testRect(3,2,&dist_grad);
    
    cout<<"testRect(5,4,&dist_grad);"<<endl;
    testRect(5,4,&dist_grad);
    cout<<"testRect(4,2.5,&dist_grad);"<<endl;
    testRect(4,2.5,&dist_grad);
    testRect(6.5,6,&dist_grad);

}