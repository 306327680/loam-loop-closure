//
// Created by echo on 18-8-6.
//
#include "pcd_finder.h"

#include <dirent.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>

PCDFinder::PCDFinder() {
  current_received_ = false;

  sub_current_ = nh_.subscribe<std_msgs::Header>(
      "/loop_info_current", 5, &PCDFinder::CurrentCallback, this);
  sub_precedent_ = nh_.subscribe<std_msgs::Header>(
      "/loop_info_precedent", 5, &PCDFinder::PrecedentCallback, this);

  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/loop_coarse_tf", 5);
  pub_precedent_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/ndt_points_past", 5);
  pub_current_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_points_cur", 5);
}

PCDFinder::~PCDFinder() {}

bool PCDFinder::Init(const std::string &directory, const char *prior,
                     const char *after) {
  frame_prior_ = atoi(prior);
  frame_after_ = atoi(after);
  directory_ = directory;
  if (frame_after_ - frame_prior_ < 3) {
    std::cerr << "at least 4 frames\n";
    return false;
  }
  std::cout << "PCDFinder initialization done\n";
  return true;
}

bool PCDFinder::GetFileNames(const std::string directory,
                             const std::string suffix) {
  file_names_.clear();
  DIR *dp;
  struct dirent *dirp;
  if ((dp = opendir(directory.c_str())) == NULL) {
    std::cerr << "cannot open directory: " << directory << std::endl;
    return false;
  }
  std::string file;
  while ((dirp = readdir(dp)) != NULL) {
    file = dirp->d_name;
    if (file.find("_") != std::string::npos) {
      file = directory + '/' + file;
      if (suffix == file.substr(file.size() - suffix.size())) {
        file_names_.push_back(file);
      }
    }
  }
  closedir(dp);
  std::sort(file_names_.begin(), file_names_.end());
  if (file_names_.empty()) {
    std::cerr << "directory: " << directory << " is empty" << std::endl;
    return false;
  }
  return true;
}

int64_t PCDFinder::FindFileSeq(const std_msgs::Header::ConstPtr &header) {
  int64_t idx_file;
  std::string filename;
  for (int i = 0; i < 5; i++) {
    for (idx_file = 0; idx_file < file_names_.size(); idx_file++) {
      filename = file_names_.at(idx_file);
      uint64_t idx_seperator = filename.find_last_of('/');
      filename = filename.substr(idx_seperator + 1);

      std::string time_stamp = filename;
      uint64_t dot_idx = filename.find_first_of('_');
      time_stamp.replace(dot_idx, 1, " ");
      dot_idx = time_stamp.find_first_of('.');
      time_stamp.replace(dot_idx, 1, " ");

      char *p_end;
      uint32_t sec = strtoul(time_stamp.c_str(), &p_end, 10);
      uint32_t nsec = strtoul(p_end, &p_end, 10);

      // find file stamp which is larger than header.stamp the first time
      if (sec < header->stamp.sec) {
        continue;
      }
      if (sec == header->stamp.sec) {
        if (nsec < header->stamp.nsec) {
          continue;
        }
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
          new pcl::PointCloud<pcl::PointXYZ>);
      do {
        idx_file--;
        if (idx_file < 1) {
          return INT64_MAX;
        }
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_names_[idx_file],
                                                *cloud) == -1) {
          PCL_ERROR("Couldn't read file. \n");
          continue;
        }
      } while (!cloud->is_dense);

      idx_seperator = file_names_[idx_file].find_last_of('/');
      filename = file_names_[idx_file].substr(idx_seperator + 1);
      std::cout << "PCD file is " << filename << std::endl;
      std::cout << "seq is " << idx_file << std::endl;
      return idx_file;
    }
    if (!GetFileNames(directory_, "pcd")) {
      return INT64_MAX;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  std::cerr << "not found yet\n";
  return INT64_MAX;
}

int64_t PCDFinder::FindFileSeq(int64_t seq) {
  int64_t idx_file = seq;
  if (idx_file > file_names_.size() - 1) {
    return INT64_MAX;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  idx_file++;
  do {
    idx_file--;
    if (idx_file < 0) {
      return INT64_MAX;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_names_.at(idx_file), *cloud) ==
        -1) {
      PCL_ERROR("Couldn't read file. \n");
      continue;
    }
  } while (!cloud->is_dense);
  uint64_t idx_seperator = file_names_[idx_file].find_last_of('/');
  std::string filename = file_names_[idx_file].substr(idx_seperator + 1);
  std::cout << "PCD file is " << filename << std::endl;
  std::cout << "seq is " << idx_file << std::endl;
  return idx_file;
}

void PCDFinder::StoreNeighbourPointCloud(int64_t seq) {
  int64_t seq_prior = seq + frame_prior_ > 0 ? seq + frame_prior_ : 0;
  int64_t seq_after = seq + frame_after_ < file_names_.size() - 1
                      ? seq + frame_after_
                      : file_names_.size() - 1;

  neigh_cloud_.clear();
  int idx_seq = 0;  // index of cloud in neigh_cloud_ which is related to seq
  for (int64_t i = seq_prior; i <= seq_after; i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr single_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_names_[i], *single_cloud) ==
        -1) {
      PCL_ERROR("Couldn't read file. \n");
      std::cout << file_names_[i] << std::endl;
    }
    if (single_cloud->is_dense) {
      neigh_cloud_.push_back(single_cloud);
    }
    if (i == seq) {
      idx_seq = i - seq_prior;
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr ndt_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr joint_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  auto begin = std::chrono::system_clock::now();
  std::cout << "NDT ...\n";

  *joint_cloud = *neigh_cloud_.at(idx_seq);
  for (int i = idx_seq - 1; i >= 0; i--) {
    NDTPointCloud(joint_cloud, neigh_cloud_.at(i), ndt_cloud);
    *neigh_cloud_.at(i) = *ndt_cloud;
    *joint_cloud += *ndt_cloud;
  }
  for (int i = idx_seq + 1; i < neigh_cloud_.size(); i++) {
    NDTPointCloud(joint_cloud, neigh_cloud_.at(i), ndt_cloud);
    *neigh_cloud_.at(i) = *ndt_cloud;
    *joint_cloud += *ndt_cloud;
  }

  auto end = std::chrono::system_clock::now();
  auto duration = (end - begin).count() / 1000000.0;
  std::cout << "computation time: " << duration << "ms" << std::endl;
}

Eigen::Matrix4d PCDFinder::NDTPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {

  // filters
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(0.3, 0.3, 0.3);
  voxel_filter.setInputCloud(cloud_in);
  voxel_filter.filter(*filtered_cloud);

  // ndt
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.5);
  ndt.setResolution(2.0);
  ndt.setMaximumIterations(100);
  ndt.setInputSource(filtered_cloud);
  ndt.setInputTarget(target_cloud);

  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  ndt.align(*cloud_out, init_guess);
  pcl::transformPointCloud(*cloud_in, *cloud_out, ndt.getFinalTransformation());

  return ndt.getFinalTransformation().cast<double>();
}

void PCDFinder::FilterDynamicObj() {
  auto begin = std::chrono::system_clock::now();
  std::cout << "removing dynamic obj by kdtree...\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr joint_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < neigh_cloud_.size(); i++) {
    joint_cloud->clear();
    for (size_t j = 0; j < neigh_cloud_.size(); j++) {
      if (j != i && j != i - 1 && j != i + 1) {
        *joint_cloud += *neigh_cloud_.at(j);
      }
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    std::vector<int> point_idx;
    std::vector<float> point_distance;
    double radius = 0.3;
    kdtree.setInputCloud(joint_cloud);

    for (auto it_point : *neigh_cloud_.at(i)) {
      if (kdtree.radiusSearch(it_point, radius, point_idx, point_distance) >
          0) {
        inlier->push_back(it_point);
      }
    }
    *neigh_cloud_.at(i) = *inlier;
  }
  auto end = std::chrono::system_clock::now();
  auto duration = (end - begin).count() / 1000000.0;
  std::cout << " " << duration << "ms" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCDFinder::GetCloud() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (auto it_cloud : neigh_cloud_) {
    *cloud_out += *it_cloud;
  }
  return cloud_out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCDFinder::GetCloud(int begin, int end) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = begin; i <= end; i++) {
    *cloud_out += *neigh_cloud_.at(i);
  }
  return cloud_out;
}

void PCDFinder::CurrentCallback(const std_msgs::Header::ConstPtr &header_curr) {
  if (current_received_ == false) {
    if (GetFileNames(directory_, "pcd")) {
      std::cout<<"file size "<<file_names_.size()<<std::endl;
      std::cout << "received current stamp" << std::endl;
      header_current_ = header_curr;
      current_received_ = true;
    }
  }
}

void PCDFinder::PrecedentCallback(
    const std_msgs::Header::ConstPtr &header_prec) {
  if (current_received_ == true) {
    std::cout << "received precedent stamp" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);

    int64_t seq_current = FindFileSeq(header_current_->seq);
    if (seq_current != INT64_MAX) {
      StoreNeighbourPointCloud(seq_current);
//      FilterDynamicObj();
      cloud = GetCloud();
      pcl::toROSMsg(*cloud, output_pc2_);
      output_pc2_.header.frame_id = "camera_init";
      pub_current_.publish(output_pc2_);
      std::cout << "published the current PointCloud " << std::endl;
    } else {
      std::cout << "didn't find relevant current file" << std::endl;
    }

    int64_t seq_precedent = FindFileSeq(header_prec->seq);
    if (seq_precedent != INT64_MAX) {
      StoreNeighbourPointCloud(seq_precedent);
//      FilterDynamicObj();
      cloud = GetCloud();
      pcl::toROSMsg(*cloud, output_pc2_);
      output_pc2_.header.frame_id = "camera_init";
      pub_precedent_.publish(output_pc2_);
      std::cout << "published the precedent PointCloud " << std::endl;
    } else {
      std::cout << "didn't find relevant precedent file" << std::endl;
    }
    std::cout << std::endl;
    if (seq_current != INT64_MAX && seq_precedent != INT64_MAX) {
      output_odom_.twist.covariance[0] = 1;
      output_odom_.twist.covariance[2] = 1;
      output_odom_.twist.covariance[1] = seq_current;
      output_odom_.twist.covariance[3] = seq_precedent;
      pub_odom_.publish(output_odom_);
    }
    current_received_ = false;
  }
}

int64_t PCDFinder::GetSizeofFiles() { return file_names_.size(); }
