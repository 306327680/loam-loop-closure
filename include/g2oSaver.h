//
// Created by echo on 18-7-21.
//

#ifndef G2OODOMSAVE_G2OSAVER_H
#define G2OODOMSAVE_G2OSAVER_H

#include <iostream>
#include <fstream>
#include <string>

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix标准函数定义*/
#include <sys/types.h>  /**/
#include <sys/stat.h>   /**/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/core/factory.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

//G2O_USE_TYPE_GROUP(slam3d);
//G2O_USE_TYPE_GROUP(slam2d);
using namespace std;
using namespace g2o;

class G2oSaver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  G2oSaver();
  //初始化订阅的topic
  void setup(ros::NodeHandle &node,
             ros::NodeHandle &privateNode);
  //ros spin
  int spin();
  //处理接收到的信息
  void process();
  //处理订阅到的里程计信息
  void lidarOdomHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry);
  //处理ndt 产生的时间trans
  void ndtTrans(const nav_msgs::Odometry::ConstPtr &ndtTrans);
  void readg2o(std::string g2o_path);
  //保存g2o
  bool saveg2o(Eigen::Isometry3d cur);
  //存闭环边的
  bool saveg2o();
  //保存g2o文件
  bool savefile();
  //Eigen----->TF
  void poseEigenToTF(const Eigen::Isometry3d &e, tf::Pose &t);
  //TF----->Eigen
  void poseTFToEigen(const tf::Pose &t, Eigen::Isometry3d &e);
  //Eigen---->Transform
  void transformEigenToTF(const Eigen::Isometry3d &e, tf::Transform &t);
  //Transform--->Eigen
  void transformTFToEigen(const tf::Transform &t, Eigen::Isometry3d &e);
  void odometryToEigen(nav_msgs::Odometry &o, Eigen::Isometry3d &e);
  void eigenToOdom(Eigen::Isometry3d &e, nav_msgs::Odometry &o);
  //输出matrix
  void print4x4Matrix(const Eigen::Isometry3d &matrix);
  //最后一步
  void addEstimate();
  //计算真正的TF
  Eigen::Isometry3d calculateRealTF(Eigen::Isometry3d Tndt, VertexSE3 from, VertexSE3 to);
  //
  void pubPotentialLoopclosure();

  //串口相关函数
  void set_speed(int fd, int speed);
  int OpenDev(char *Dev);
  int serialProcess();

  //
//    void loopClosure();
  //odom->eigen
  //void odometryToEigen(nav_msgs::Odometry &o, Eigen::Isometry3d &e);

 private:
  //订阅里程计信息
  ros::Subscriber _odomLidar;

  ros::Subscriber _NDT_temp;
  ros::Publisher _toLM;
  ros::Publisher _loop_current;
  ros::Publisher _loop_precedent;
  //节点总数目
  int _vertexnum = 0;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> _odom_buffer;
  //存放时间戳和第几个节点
  vector<pair<int, ros::Time>> _timestamp;
  //闭环开始结束vertex
  //在回调函数中记录
  vector<int> _vertxfromnum;
  vector<int> _vertextonum;
  //回环边的数量
  int _edgeloopnumbers = 0;

  //用来存储顶点和边
  vector<VertexSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _vertices;
  vector<EdgeSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _odometryEdges;
  vector<EdgeSE3 *, Eigen::aligned_allocator<g2o::Isometry3D>> _edges;
//vector<VertexSE3 *> _vertices;
//vector<EdgeSE3 *> _odometryEdges;
//vector<EdgeSE3 *>  _edges;
  //检测新里程计
  bool _newodom = false;
  //检测新的差值计算
  bool _newtrans = false;
  //新的回环
  bool _newloop = false;
  bool _newloop_from = false;
  bool _newloop_to = false;
  //保存文件名
  string _outFilename = "LidarOdom.g2o";
  //odom缓存
  nav_msgs::Odometry _odomtmp;
  nav_msgs::Odometry _odomlast;
  nav_msgs::Odometry _temp;
  //TODO 测试
  nav_msgs::Odometry _rotated;
  //两次tf之差
  Eigen::Isometry3d _transdiffer = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d _transdiffertrans = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d _curpose = Eigen::Isometry3d::Identity();
  //旋转Loam里程计到正常位置
  Eigen::Isometry3d _odom_rotate_temp = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d _odom_rotate_matrix = Eigen::Isometry3d::Identity();
  //保存闭环边
  Eigen::Isometry3d _loop_from = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d _loop_to = Eigen::Isometry3d::Identity();
  //两帧旋转矩阵
  Eigen::AngleAxisd _rotation_vector = Eigen::AngleAxisd::Identity();
  Eigen::AngleAxisd _cur_rotation_vector = Eigen::AngleAxisd::Identity();
  //两帧平移
  Eigen::Vector3d _trans_vector = Eigen::Vector3d::Identity();
  Eigen::Vector3d _cur_trans_vector = Eigen::Vector3d::Identity();
  //四元数 两帧之间的
  Eigen::Quaterniond _quat_differ;
  //默认的信息矩阵TODO回头要设置初值
  Eigen::Matrix<double, 6, 6> _information = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> _information_loopclosure = Eigen::Matrix<double, 6, 6>::Identity();
  //存两个闭环边的tf
  vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> _trans_loopclosure;
  //储存时间戳
  uint32_t nsec_from = 0;
  uint32_t sec_from = 0;
  uint32_t nsec_to = 0;
  uint32_t sec_to = 0;
  //串口的
  int speed_arr[15] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
                       B38400, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[15] = {115200, 38400, 19200, 9600, 4800, 2400, 1200, 300,
                      38400, 19200, 9600, 4800, 2400, 1200, 300};
  int fd;
  int nread;
  int buff[2000];
  char buff_char[2000];
  char *dev = "/dev/ttyUSB0";

  //constant parameter
  const int _k_search_before = 30;
  const double _k_search_radius = 20;
  const int _k_search_interval = 5;
  const int _k_search_from = -10;
};

#endif //G2OODOMSAVE_G2OSAVER_H
