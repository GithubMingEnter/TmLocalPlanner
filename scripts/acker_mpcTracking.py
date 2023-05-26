# coding=utf-8‘
# ipopt 路径记得修改
from pdb import runcall
from sys import executable
import time
import atexit
from numpy import dtype
import rospy
import copy
import tf
import numpy as np
from scipy import spatial
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from pyomo.environ import *
from pyomo.dae import *
from scipy.interpolate import interp1d
import matplotlib
import matplotlib.pyplot as plt
from ackermann_msgs.msg import AckermannDrive
import math
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import *  # 与gazebo交互的server头文件
from geometry_msgs.msg import Pose
from time import sleep

# path = '~/thirdParty/ipopt-linux64/ipopt'

path = '/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/scripts/ipopt-linux64/ipopt'
file_time=open('/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/data/mpc2_time.csv','w')
odom_topic="/odom"
is_ipython = 'inline' in matplotlib.get_backend()
# 检查内联matplotlib：
t0=time.time()
if is_ipython:
    from IPython import display

plt.ion()  # 开启interactive mode 成功的关键函数
txt_path ='/home/lwm/ros1_workspace/tm_local_planner_ws/src/topp2/scripts/action.txt'


wps = np.loadtxt(txt_path)

x = wps[:, 0]
y = wps[:, 1]

t = np.linspace(0, 1, num=len(x))
f1 = interp1d(t, x, kind='cubic')
f2 = interp1d(t, y, kind='cubic')  # note 1 not l
# 拆分
inter_point = 500
newt = np.linspace(0, 1, inter_point)
nwps = np.zeros((inter_point, 2))
nwps[:, 0] = f1(newt)
nwps[:, 1] = f2(newt)
wpstree = spatial.KDTree(nwps)
# ?


def getcwps(rp):
    _, nindex = wpstree.query(rp)
    cwps = np.zeros((5, 2))  # ?
    for i in range(5):
        cwps[i] = nwps[(nindex+i) % len(nwps)]
    return cwps


def cubic_fun(coeffs, x):
    return coeffs[0]*x**3+coeffs[1]*x**2+coeffs[2]*x+coeffs[3]  # ? 3次多项式


def plot_durations(cwps, prex, prey):
    plt.figure(2)
    plt.clf()
    plt.plot(cwps[:, 0], cwps[:, 1])
    plt.plot(prex, prey)
    plt.scatter(x, y)
    if is_ipython:
        display.clear_output(wait=True)
        display.display(plt.gcf())


N = 19  # forward predict steps
ns = 5  # state num /x,y,psi: f(x) differential,cte横向偏差,epsi 航向偏差
na = 2  # out put number /steering angle omega
def degToRad(angle):
    return angle/180*math.pi
def RadToDeg(rad):
    return rad/math.pi*180
def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def convert_trans_steering_angle_to_rot_vel(v, steering_angle, wheelbase):
  if  v == 0:
    return 0
  omega= v * math.tan(steering_angle)/ wheelbase
  return omega

class MPC(object):
    def __init__(self):
        m = ConcreteModel()
        m.sk = RangeSet(0, N-1)  # k=0,1,N-1
        m.uk = RangeSet(0, N-2)
        m.uk1 = RangeSet(0, N-3)

        m.wg = Param(RangeSet(0, 3), initialize={
                     0: 1., 1: 10., 2: 100., 3: 1000}, mutable=True)  # mutable表示是可变的参数
        m.dt = Param(initialize=0.1, mutable=True)  # 离散时间
        m.ref_v = Param(initialize=1.0, mutable=True) # 参考速度
        m.ref_cte = Param(initialize=0.0, mutable=True) # 参考偏差
        m.ref_epsi = Param(initialize=0.0, mutable=True) #参考航向误差
        m.Lf = Param(initialize=0.14 , mutable=True) # wheel_base
        m.s0 = Param(RangeSet(0, ns-1),
                     initialize={0: 0., 1: 0., 2: 0., 4: 0.}, mutable=True) #状态列表
        m.coeffs = Param(RangeSet(0, 3),
                         initialize={0: -0.000458316, 1: 0.00734257, 2: 0.0538795, 3: 0.080728}, mutable=True) # 参数列表

        m.s = Var(RangeSet(0, ns-1), m.sk)  # state variable 前向模拟变量数
        m.f = Var(m.sk)  # 前向模拟索引
        m.psides = Var(m.sk)    # 变量量
        m.uv = Var(m.uk, bounds=(0.01, 1.05))  # 速度变量范围
        m.uw = Var(m.uk, bounds=(-0.6, 0.6) # 角度变化

        # update
        m.s0_update =  Constraint(RangeSet(0,ns-1),rule=lambda m,i:m.s[i,0]==m.s0[i])
        m.x_update = Constraint(m.sk, rule=lambda m, k:
                                m.s[0, k+1] == m.s[0, k] +
                                m.uv[k]*cos(m.s[2, k])*m.dt
                                if k < N-1 else Constraint.Skip)
        m.y_update = Constraint(m.sk, rule=lambda m, k:
                                m.s[1, k+1] == m.s[1, k] +
                                m.uv[k]*sin(m.s[2, k])*m.dt
                                if k < N-1 else Constraint.Skip)
        m.psi_update = Constraint(m.sk, rule=lambda m, k:
                                  m.s[2, k+1] == m.s[2, k] +
                                  m.uv[k]*tan(m.uw[k])/m.Lf*m.dt  # note ==
                                  if k < N-1 else Constraint.Skip)  # ignore tan()/L
        m.f_update = Constraint(m.sk, rule=lambda m, k:
                                m.f[k] == m.coeffs[0]*m.s[0, k]**3+m.coeffs[1]*m.s[0, k]**2 +
                                m.coeffs[2]*m.s[0, k]+m.coeffs[3])
        m.psides_update = Constraint(m.sk, rule=lambda m, k:
                                     m.psides[k] == atan(3*m.coeffs[0]*m.s[0, k]**2
                                                         + 2*m.coeffs[1]*m.s[0, k]+m.coeffs[2]))
        m.cte_update = Constraint(m.sk, rule=lambda m, k:
                                  m.s[3, k+1] == (m.f[k]-m.s[1, k] +
                                                  m.uv[k]*sin(m.s[2, k])*m.dt)
                                  if k < N-1 else Constraint.Skip)
        m.epsi_update = Constraint(m.sk, rule=lambda m, k:
                                   m.s[4, k+1] == m.psides[k] -
                                   m.s[2, k]+m.uv[k]*tan(m.uw[k])/m.Lf*m.dt
                                   if k < N-1 else Constraint.Skip)

        m.cteobj = m.wg[3]*sum((m.s[3, k]-m.ref_cte) **
                               2 for k in m.sk)  # why wg[3]
        m.epsiobj = m.wg[3]*sum((m.s[4, k]-m.ref_epsi)**2 for k in m.sk)
        m.vobj = m.wg[2]*sum((m.uv[k]-m.ref_v)**2 for k in m.uk)
        m.uvobj = m.wg[1]*sum(m.uv[k]**2 for k in m.uk)
        m.uwobj = m.wg[1]*sum(m.uw[k]**2 for k in m.uk)
        m.sudobj = m.wg[0]*sum((m.uv[k+1]-m.uv[k])**2 for k in m.uk1)
        m.suaobj = m.wg[0]*sum((m.uw[k+1]-m.uw[k])**2 for k in m.uk1)
        m.obj = Objective(expr=m.cteobj+m.epsiobj+m.vobj +
                          m.uvobj+m.uwobj+m.sudobj+m.suaobj, sense=minimize)

        self.iN = m

    def Solve(self, state, coeffs):
        self.iN.s0.reconstruct(
            {0: state[0], 1: state[1], 2: state[2], 3: state[3], 4: state[4]})
        self.iN.coeffs.reconstruct(
            {0: coeffs[0], 1: coeffs[1], 2: coeffs[2], 3: coeffs[3]})
        self.iN.f_update.reconstruct()
        self.iN.s0_update.reconstruct()
        self.iN.psides_update.reconstruct()
        SolverFactory('ipopt', executable=path).solve(self.iN)
        x_pred_vals = [self.iN.s[0, k]() for k in self.iN.sk]
        y_pred_vals = [self.iN.s[1, k]() for k in self.iN.sk]
        pre_path = np.zeros((N, 2))
        pre_path[:, 0] = np.array(x_pred_vals)
        pre_path[:, 1] = np.array(y_pred_vals)
        v = self.iN.uv[0]()
        w = convert_trans_steering_angle_to_rot_vel(v,self.iN.uw[0](),0.04) #RadToDeg(self.iN.uw[0]())  
        return pre_path, v, w


class Turtlebot_core():
    def __init__(self):
        rospy.init_node("Turtlebot_core", anonymous=True)
        self.cme_vel=rospy.get_param('~cmd_vel',default='cmd_vel')
        self.listener = tf.TransformListener()
        rospy.Subscriber(odom_topic, Odometry, self.odomCallback)
        self.pub_refpath = rospy.Publisher("/ref_path", Path, queue_size=1)
        self.pub_prepath = rospy.Publisher("/pre_path", Path, queue_size=1)
        self.pub_cmd=rospy.Publisher(self.cme_vel,Twist,queue_size=10)
        self.car_point_vis_pub = rospy.Publisher(
            "car_point", Marker, queue_size=10)
        self.rp = np.zeros(3)
        self.crv = 0.0
        self.crw = 0.0
        self.mpc = MPC()
        self.marker_sample = Marker()
        self.marker_sample.header.frame_id = "map"
        self.marker_sample.ns = "car_point"
        self.marker_sample.type = Marker.CUBE
        self.marker_sample.scale.x = 1.0
        self.marker_sample.scale.y = 0.8
        self.marker_sample.scale.z = 0.7
        self.marker_sample.action = Marker.ADD
        self.marker_sample.color.r = 1.0
        self.marker_sample.color.g = 0.0
        self.marker_sample.color.b = 1.0
        self.marker_sample.color.a = 0.7
        self.marker_sample.lifetime = rospy.Duration()
        rate = rospy.Rate(10)  # 10 hz
        self.run_step = 0
        self.gazebo_model_name = 'ackerman'
        self.set_model_state_service = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)
        
        while not rospy.is_shutdown():
            self.run_step += 1
            rospy.loginfo("run step =%d", self.run_step)
            self.getrobotpose()
            cwps = getcwps(self.rp[0:2])
            px = self.rp[0]+self.crv*np.cos(self.rp[2])*0.1
            py = self.rp[1]+self.crw*np.sin(self.rp[2])*0.1
            psi = self.rp[2]+self.crw*0.1

            self.rp[0] = px
            self.rp[1] = py
            self.rp[2] = psi

            cwps_robot = np.zeros((len(cwps), 2))
            for i in range(len(cwps)):
                dx = cwps[i, 0]-px
                dy = cwps[i, 1]-py

                cwps_robot[i, 0] = dx*np.cos(psi)+dy*np.sin(psi)
                cwps_robot[i, 1] = dy*np.cos(psi)-dx*np.sin(psi)
            coeffs = np.polyfit(cwps_robot[:, 0], cwps_robot[:, 1], 3)
            cte = cubic_fun(coeffs, 0)

            f_prime_x = coeffs[2]
            epsi = np.arctan(f_prime_x)
            s0 = np.array([0.0, 0.0, 0.0, cte, epsi])
            ts=time.time()
            pre_path, v, w = self.mpc.Solve(s0, coeffs)
            te=time.time()
            time_run=te-t0
            file_time.write('%f\n'%(time_run))
            

            self.pub_ref_path(cwps_robot)
            self.pub_pre_path(pre_path)
            self.pub_Command(v, w)
            print(v, w)

            rate.sleep()
        rospy.spin()

    def resetOrigin(self, t):
        set_model_state_service = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState)
        model_state_msg = SetModelStateRequest()

        model_state_msg.model_state.model_name = self.gazebo_model_name
        pose_ = Pose()

        pose_.orientation.x = 0
        pose_.orientation.y = 0
        pose_.orientation.z = 0
        pose_.orientation.w = 0

        model_state_msg.model_state.pose = pose_
        set_model_state_service(model_state_msg)
        sleep(t)    
        
    def getrobotpose(self):
        try:
            (trans, rot) = self.listener.lookupTransform(
                '/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        self.rp[0] = trans[0]
        self.rp[1] = trans[1]

        r, p, y = tf.transformations.euler_from_quaternion(rot)  # note s
        self.rp[2] = y
        self.marker_sample.pose.position.x = trans[0]
        self.marker_sample.pose.position.y = trans[1]
        self.marker_sample.pose.position.z = 0.0
        self.marker_sample.id = self.run_step
        self.marker_sample.pose.orientation.x = rot[0]
        self.marker_sample.pose.orientation.y = rot[1]  # note分开赋值
        self.marker_sample.pose.orientation.z = rot[2]
        self.marker_sample.pose.orientation.w = rot[3]
        for i in range(2):
            self.car_point_vis_pub.publish(self.marker_sample)

    def odomCallback(self, data):
        self.crv = data.twist.twist.linear.x
        self.crw = data.twist.twist.angular.z

    def pub_ref_path(self, ref_path):
        msg_ref_path = Path()
        msg_ref_path.header.stamp = rospy.Time.now()
        msg_ref_path.header.frame_id = "base_link"
        for i in range(len(ref_path)):
            pose = PoseStamped()
            pose.pose.position.x = ref_path[i, 0]
            pose.pose.position.y = ref_path[i, 1]
            msg_ref_path.poses.append(copy.deepcopy(pose))
        self.pub_refpath.publish(msg_ref_path)

    def pub_pre_path(self, pre_path):
        msg_pre_path = Path()
        msg_pre_path.header.stamp = rospy.Time.now()
        msg_pre_path.header.frame_id = "base_link"
        for i in range(len(pre_path)):
            pose = PoseStamped()
            pose.pose.position.x = pre_path[i, 0]
            pose.pose.position.y = pre_path[i, 1]
            msg_pre_path.poses.append(copy.deepcopy(pose))
        self.pub_prepath.publish(msg_pre_path)

    def pub_Command(self, v, w):
        twist=Twist()
        twist.linear.x=v
        twist.angular.z=w
        
        # print("x=%f", x)
        print("steer angle=%f", w)
        self.pub_cmd.publish(twist)

def shutdown():
    file_time.close()
    print("save file ")

if __name__ == "__main__":
    atexit.register(shutdown)
    try:
        # plot_durations()
        turtlebot_core = Turtlebot_core()
    except rospy.ROSInterruptException:
        pass
