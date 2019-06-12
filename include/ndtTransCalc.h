//
// Created by echo on 18-7-25.
//
#ifndef G2OODOMSAVE_NDTTRANSCALC_H
#define G2OODOMSAVE_NDTTRANSCALC_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>

//#include <g2o/types/slam3d/types_slam3d.h>
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
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <pcl/registration/icp.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

class ndtcalc {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ndtcalc() {};
  void setup(ros::NodeHandle &node,
             ros::NodeHandle &privateNode);
  //ros spin
  int spin();
  //处理接收到的信息
  void process();
  void picNumHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry);
  void pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg);
  void pointCloudCurHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg);
  bool _new_loop = false;
  bool _process = true;
  //bool _new_data = true;
  bool _new_data = true;

  Eigen::Isometry3d _odomCoarse = Eigen::Isometry3d::Identity();

 private:
  void showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud);
  void groundZFilter(int max, int min, pcl::PointCloud<pcl::PointXYZ> &pointIn);
  void groundZExtract(int max, int min, pcl::PointCloud<pcl::PointXYZ> &pointIn);
  void print4x4Matrix(const Eigen::Matrix4d &matrix);
  void print4x4Matrix(const Eigen::Isometry3d &matrix);
  void publishNdt(const Eigen::Matrix4d &matrix, int cur_process);
  void publishNdt(const Eigen::Matrix4d &matrix);
  void odometryToEigen(nav_msgs::Odometry &o, Eigen::Isometry3d &e);

  nav_msgs::Odometry _ndtTfed;
  nav_msgs::Odometry _transedOdomFunction;
  typedef pcl::PointXYZ PointT;
  //读取文件 名称(测试用)
//    std::string in1 = "/home/echo/rosbag_test/oneframg2o/1512294989.398242000.pcd";
//    std::string in2 = "/home/echo/rosbag_test/oneframg2o/1512294995.446301000.pcd";
//    std::string in1 = "/home/echo/rosbag_test/lidarodomLoopOrigin/pcdandg2o/vtk_test_ndt/1512294925.597238000.pcd";
//    std::string in2 = "/home/echo/rosbag_test/lidarodomLoopOrigin/pcdandg2o/vtk_test_ndt/1512295647.263673000.pcd";
  std::string in1 =
      "/home/echo/pcl/doc/tutorials/content/sources/normal_distributions_transform/cmake-build-debug/validation/3.pcd";
  std::string in2 =
      "/home/echo/pcl/doc/tutorials/content/sources/normal_distributions_transform/cmake-build-debug/validation/4.pcd";

  ros::Subscriber _picNumSub;
  ros::Subscriber _pointCloudSub;
  ros::Subscriber _pointCloudSubCur;

  ros::Publisher _pubLoopClosureTrans;
  ros::Publisher _curPointForTf;
  ros::Publisher _pastPointForTf;

  int _loop_times = 0;
  int _cur_process = 0;

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_gnd;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_ground;

  pcl::PointCloud<pcl::PointXYZ> _pcd_cur_after_ndt;
  pcl::PointCloud<pcl::PointXYZ> _pcd_past_after_ndt;
  //缓存 点云 1 2 初值
  std::vector<pcl::PointCloud<pcl::PointXYZ>> _cloud_last;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> _cloud_curr;
  std::vector<nav_msgs::Odometry> _odom_seq;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> _relativeTF;
  Eigen::Isometry3d _final_tf = Eigen::Isometry3d::Identity();
  bool _new_cloud_curr = false;
  bool _new_cloud_last = false;
  //之后
};

#endif //G2OODOMSAVE_NDTTRANSCALC_H
