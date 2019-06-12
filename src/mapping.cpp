#include "mapping.h"

#include <dirent.h>
#include <stdint.h>
#include <string.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include <tf_conversions/tf_eigen.h>

Mapping::Mapping(const std::string &filename1, const std::string &filename2,
                 const std::string &pcd_path) {
  filename_g2obef_ = filename1;
  filename_g2oaft_ = filename2;
  if(filename1 == filename2){
    _g2o_same = true;
  }
  directory_ = pcd_path;
  suffix_ = "pcd";

  idx_final_map_ = 0;

  sub_opt_ =
      nh_.subscribe("/start_mapping", 2, &Mapping::Opt_DoneCallback, this);

  pub_map_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/mapping/pointcloud2_", 1);
  pub_done_map_ = nh_.advertise<std_msgs::Bool>("/mapping/done", 1);
}

bool Mapping::GetFileNames(const std::string directory_,
                           const std::string suffix_) {
  file_names_.clear();
  DIR *dp;
  struct dirent *dirp;
  if ((dp = opendir(directory_.c_str())) == NULL) {
    std::cerr << "cannot open directory: " << directory_ << std::endl;
    return false;
  }
  std::string file;
  while ((dirp = readdir(dp)) != NULL) {
    file = dirp->d_name;
    if (file.find(".") != std::string::npos) {
      file = directory_ + '/' + file;
      if (suffix_ == file.substr(file.size() - suffix_.size())) {
        file_names_.push_back(file);
      }
    }
  }

  closedir(dp);
  std::sort(file_names_.begin(), file_names_.end());
  if (file_names_.empty()) {
    std::cerr << "directory: " << directory_ << " is empty" << std::endl;
    return false;
  }
  mkdir((directory_ + "/finalmap").c_str(), 0777);
  return true;
}

std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> Mapping::getEigenPoseFromg2oFile(
    std::string &g2ofilename) {
  std::cout<<"getEigenPoseFromg2oFile start"<<std::endl;
  std::ifstream fin(g2ofilename);
  std::cout<<"fin"<<std::endl;
  if (!fin) {
    std::cerr << "file " << g2ofilename << " does not exist." << std::endl;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT;     //todo
    vT.resize(1);
    vT[0] = Eigen::Matrix4d::Identity(4, 4);
    return vT;
  }

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> vT;
  while (!fin.eof()) {
    std::string name;
    fin >> name;
    if (name == "VERTEX_SE3:QUAT") {
      g2o::VertexSE3 v;

      int index = 0;
      fin >> index;
      v.setId(index);
      v.read(fin);
      Eigen::Isometry3d T = v.estimate();
      vT.push_back(T);
    } else if (name == "EDGE_SE3:QUAT") {
      continue;
    } else
      continue;
    if (!fin.good()) break;
  }

  std::cout << "read total " << vT.size() << " vertices\n";

  return vT;
}

void Mapping::Opt_DoneCallback(const std_msgs::Bool::ConstPtr &Info) {
  if (Info->data == true) {
    // 从g2o文件中读取的方式, 只读取bef即可
    std::cout << "准备从g2o文件中读取原始的loam位姿vT_bef_" << std::endl;
    vT_bef_ = getEigenPoseFromg2oFile(filename_g2obef_);    //todo
    std::cout << "vT_bef_'s size is " << vT_bef_.size() << std::endl;

    std::cout << "准备从g2o文件中读取优化后的位姿vT_opt_" << std::endl;
    vT_opt_ = getEigenPoseFromg2oFile(filename_g2oaft_);    //todo
    std::cout << "vT_opt_'s size is " << vT_opt_.size() << std::endl;

    if (vT_bef_.size() != vT_opt_.size()) {
      std::cerr
          << "why vT_bef_'s size is not equal to vT_opt_'s size??? CHECK!!!"
          << std::endl;
    }
    std::cout << "准备建图" << std::endl;
    MapCombination();
  }
}
//todo 这里应当进行修改
bool Mapping::MapCombination() {
  GetFileNames(directory_, suffix_);

  // pcd文件的数量应该与位姿矩阵的数量相同
  if (file_names_.size() != vT_bef_.size()) {
    std::cerr << "why? pcd_file's size " << file_names_.size()
              << " do not equal to pose size" << vT_bef_.size()
              << " from g2o file!" << std::endl;
    return false;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr joint_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (unsigned int i = 0; i < file_names_.size(); i++) {
    //读转换g2o??

    G2OCorrection(i);
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));

    pcl::toROSMsg(*joint_cloud, pc2_output_);
    pc2_output_.header.frame_id = "map";
    pub_map_.publish(pc2_output_);
//    std::cout << "第" << i << "帧点云pub成功" << std::endl;
  }
  std::cout<<"finish file_names"<<std::endl;
  // RemoveDynamicObj();
  joint_cloud->clear();
  int u = 0;
  for (const auto &it_cloud : seq_cloud_) {
    u++;
    try{
        *joint_cloud += it_cloud;
        if(u%1000 == 0){
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
                    new pcl::PointCloud<pcl::PointXYZ>);
            voxel_filter.setLeafSize(0.2, 0.2, 0.2);
            voxel_filter.setInputCloud(joint_cloud);
            voxel_filter.filter(*filtered_cloud);
            joint_cloud = filtered_cloud;
        }

    }catch (std::bad_alloc){
        std::cout<<"pcd break:    "<<u<<std::endl;
    }
  }
  std::cout<<"155"<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  voxel_filter.setInputCloud(joint_cloud);
  voxel_filter.filter(*filtered_cloud);



  std::string save_dir =
      directory_ + "/finalmap/finalmap" + std::to_string(idx_final_map_) + ".pcd";
  pcl::io::savePCDFile(save_dir, *filtered_cloud, true);
  std::cerr << "Saved " << filtered_cloud->points.size()
            << " data points to finalmap" + std::to_string(idx_final_map_++) +
                ".pcd"
            << std::endl;
  seq_cloud_.clear();
  std_msgs::Bool msg_done;
  msg_done.data = true;
  pub_done_map_.publish(msg_done);
  std::cout << "发布建图结束信息" << std::endl;

  return true;
}

void Mapping::G2OCorrection(size_t idx) {
  std::string file_name = file_names_.at(idx);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bef(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud_bef) ==
      -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file, skip this Cloud. \n");
    return;
  }
  //todo cloud_bef downsample
  //2018/11/14 add new functions to down sample the input map
  //changed by echo
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud1(
//          new pcl::PointCloud<pcl::PointXYZ>);
//  voxel_filter.setLeafSize(0.1, 0.1, 0.1);
//  voxel_filter.setInputCloud(cloud_bef);
//  //voxel_filter.filter(*cloud_bef);
//  voxel_filter.filter(*filtered_cloud1);
  //2018/11/14 检测两个g2o文件是否相同如果相同就直接读取第二个g2o file 把 点云转换过去
  if(_g2o_same){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aft(
            new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Isometry3d T_opt = vT_opt_[idx];
    //*cloud_bef = *cloud_aft;
    pcl::transformPointCloud(*cloud_bef, *cloud_aft, T_opt.matrix());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    voxel_filter.setInputCloud(cloud_aft);
    voxel_filter.filter(*filtered_cloud);
    seq_cloud_.push_back(*filtered_cloud);


  }else{

    //如果两个g2o不相同就计算之前的  TF, 这个tf 是两个g2o文件的差值
    // 以T_bef为参考系, 算变换矩阵
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aft(
            new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Isometry3d T_bef = vT_bef_[idx];
//  std::cout<<T_bef.matrix()<<std::endl;
    Eigen::Isometry3d T_opt = vT_opt_[idx];
//  std::cout<<T_opt.matrix()<<std::endl;
    pcl::transformPointCloud(*cloud_bef, *cloud_aft, T_bef.matrix().inverse().cast<float>());

    // BoxLimit(cloud_aft);
    *cloud_bef = *cloud_aft;
    pcl::transformPointCloud(*cloud_bef, *cloud_aft, T_opt.matrix());

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    voxel_filter.setInputCloud(cloud_aft);
    voxel_filter.filter(*filtered_cloud);
    seq_cloud_.push_back(*filtered_cloud);
    // seq_cloud_.push_back(*cloud_aft);
  }
}

void Mapping::RemoveDynamicObj() {
  std::cout << "removing dynamic obj by kdtree...\n";
  pcl::PointCloud<pcl::PointXYZ>::Ptr neigh_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < seq_cloud_.size(); i++) {
    size_t start = i > 5 ? i - 5 : 0;
    size_t end = i < seq_cloud_.size() - 5 ? i + 4 : seq_cloud_.size() - 1;
    neigh_cloud->clear();
    for (size_t j = start; j < end; j++) {
      if (i != j) {
        *neigh_cloud += seq_cloud_.at(j);
      }
    }
    pcl::PointCloud<pcl::PointXYZ> inlier;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    std::vector<int> point_idx;
    std::vector<float> point_distance;
    double radius = 0.2;
    kdtree.setInputCloud(neigh_cloud);
    for (const auto &it_point : seq_cloud_.at(i)) {
      if (kdtree.radiusSearch(it_point, radius, point_idx, point_distance) >
          0) {
        inlier.push_back(it_point);
      }
    }
    seq_cloud_.at(i) = inlier;
  }
}

void Mapping::BoxLimit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_out) {
  pcl::PointCloud<pcl::PointXYZ> inlier;
  float box_len = 80 / 2;
  for (auto it_point : *cloud_in_out) {
    if (std::abs(it_point.x) < box_len && std::abs(it_point.y) < box_len && std::abs(it_point.z) < box_len) {
      inlier.push_back(it_point);
    }
  }
  *cloud_in_out = inlier;
}
