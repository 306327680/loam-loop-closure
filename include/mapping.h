//
// 需求
// 为correct预留, 可有可无
// 1) 有correct, 接收来自loam/原始.g2o 和 ndt的数据, 直到建图
// 2) 无correct, 接收来自原始loam/原始.g2o 和 优化后.g2o的数据, 直到建图
// (接收一个bool消息类型)

// 依赖库, 包含g2o, Eigen, pcl, ROS

#ifndef PROJECT_MAPPING_H
#define PROJECT_MAPPING_H

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

// ROS
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

// pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// g2o  // Thirdparty...
#include "g2o/config.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/stuff/command_args.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"

class Mapping {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Mapping(const std::string &filename1, const std::string &filename2,
          const std::string &pcdPath);

 private:
  // 获取所有的pcd文件
  bool GetFileNames(const std::string directory, const std::string suffix);

  // 从.g2o中读取位姿
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> getEigenPoseFromg2oFile(
      std::string &g2ofilename);
  // 调用MapCombination()这个函数
  void Opt_DoneCallback(const std_msgs::Bool::ConstPtr &Info);

  // 优化之后的建图函数
  bool MapCombination();

  /*
  * correct the pose of individual map by g2o file generated after loop closure
  * (in) idx: the index of the map
  */
  void G2OCorrection(size_t idx);

  /*
  * remove dynamic object by kdtree simple and stupid
  */
  void RemoveDynamicObj();

  /*
  * remove the point out of the box(specifec in the function) from the point cloud
  * (in) cloud_in_out: the point cloud need to handle
  * (out) cloud_in_out: the treated point cloud
  */
  void BoxLimit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_out);

 private:
  //这个是如果g2o相等,则认为点云为raw data,之后只进行 优化后的点云拼接
  bool _g2o_same = false;
  std::string filename_g2obef_;
  std::string filename_g2oaft_;
  // pcd文件的路径
  std::string directory_;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT_bef_;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT_opt_;

  std::string suffix_;// suffix = "pcd"
  std::vector<std::string> file_names_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_opt_;
  ros::Publisher pub_map_;
  ros::Publisher pub_done_map_;

  sensor_msgs::PointCloud2 pc2_output_;
  nav_msgs::Odometry odom_output_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>> seq_cloud_;

  int idx_final_map_;
};
#endif //PROJECT_MAPPING_H
