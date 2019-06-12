//
// Created by echo on 18-8-6.
//
#ifndef G2OODOMSAVE_FINDPCD_H
#define G2OODOMSAVE_FINDPCD_H

#include <string>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class PCDFinder {
 public:

  /*
   * initialize the publisher, subscriber, and current_received_ flag 
   */
  PCDFinder();

  ~PCDFinder();

  /*
   * Initialization
   * directory(in): the directory of pcd file
   * prior(in): -5 means 5 frame before the current frame
   * after(in): 5 means 5 frame after the current frame
   */
  bool Init(const std::string &directory, const char *prior, const char *after);

  /*
   * NDT
   */
  Eigen::Matrix4d NDTPointCloud(
      pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

  /*
   * return joint cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();

  /*
   * return joint cloud
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(int begin, int end);

  /*
   * find the sequence of the file from received message
   * header(in) header received from ROS
   */
  int64_t FindFileSeq(const std_msgs::Header::ConstPtr &header);

  int64_t FindFileSeq(int64_t seq);

  /*
   * store the neighbour pointcloud to neigh_cloud_
   * seq(in): the sequence in the file_names_
   * joint_cloud(out): the cloud add neighbour together
   */
  void StoreNeighbourPointCloud(int64_t seq);

  /*
   * remove dynamic obj
   */
  void FilterDynamicObj();

  /*
   * return the size of neigh_cloud_
   */
  int64_t GetSizeofFiles();

 private:
  /*
   * store the file names in vector file_names_
   * directory(in): the directory of pcd file
   * suffic(in): "pcd"
   */
  bool GetFileNames(const std::string directory, const std::string suffix);

  /*
   * callback function when receive sub_current_
   */
  void CurrentCallback(const std_msgs::Header::ConstPtr &header_curr);

  /*
   * callback function when receive sub_precedent_
   */
  void PrecedentCallback(const std_msgs::Header::ConstPtr &header_prec);

 private:
  nav_msgs::Odometry output_odom_;
  sensor_msgs::PointCloud2 output_pc2_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_current_;
  ros::Subscriber sub_precedent_;
  ros::Publisher pub_odom_;
  ros::Publisher pub_precedent_;
  ros::Publisher pub_current_;
  std_msgs::Header::ConstPtr header_current_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> neigh_cloud_;

  std::vector<std::string> file_names_;
  std::string directory_;

  bool current_received_;

  int frame_prior_;
  int frame_after_;
};
#endif  // G2OODOMSAVE_FINDPCD_H
