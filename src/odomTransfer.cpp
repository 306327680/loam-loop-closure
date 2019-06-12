//
// Created by echo on 18-7-28.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h> // For geometry_msgs:: Twist
#include <stdlib.h> // For rand() and RAND_MAX
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

bool newOdom = false;
nav_msgs::Odometry transedOdom;
ros::Publisher pub;
ros::Publisher pcpub;
std::string path;
bool newOdom01hz = false;
bool _able_to_publish = false;
nav_msgs::Odometry transedOdom01hz;
ros::Publisher pub_01hz;
ros::Publisher pcpub_01hz;
int _odom_count = 0;
//这里原先是10, 每10帧来一个点云
int _rate = 1;
//todo 降采样新loam的pointcloud和odom
void odometryToEigen(const nav_msgs::Odometry &o, Eigen::Isometry3d &e) {
    Eigen::Isometry3d is_test;
    is_test = Eigen::Isometry3d::Identity();
    tf::Transform tf_test;
    tf::poseMsgToTF(o.pose.pose,tf_test);
    tf::transformTFToEigen(tf_test,e);
}

void eigenToOdom(Eigen::Isometry3d &e, nav_msgs::Odometry &o)
{
    tf::Transform tf_test;
    tf_test.setIdentity();
    tf::transformEigenToTF(e,tf_test);
    tf::poseTFToMsg(tf_test,o.pose.pose);
}
//here odom to down sample
void poseMessageReceived ( const nav_msgs::Odometry::ConstPtr &laserOdometry )
{
    _odom_count ++;
    if(_odom_count%_rate == 0) {
        newOdom = true;
        Eigen::Isometry3d _odom_rotate_temp = Eigen::Isometry3d::Identity();

        //当前的 odom到eigen  ==== =_odom_rotate_temp
        odometryToEigen(*laserOdometry, _odom_rotate_temp);

        Eigen::Vector3d axis(1, 1, 1);
        axis.normalize();

        _odom_rotate_temp.rotate(Eigen::AngleAxisd(-M_PI / 3 * 2, axis));
        _odom_rotate_temp = Eigen::AngleAxisd(M_PI / 3 * 2, axis) * _odom_rotate_temp;

        //复制头啥的
        transedOdom.header = laserOdometry->header;
        eigenToOdom(_odom_rotate_temp, transedOdom);
        pub.publish(transedOdom);
        _odom_count = 0;
        _able_to_publish = true;
    }
}
//here point to down sample
void polintcloudHandle(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg){
    //_timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;
    //点云旋转矩阵

    //每 _rate 贞 存 pub 一次
    if(_able_to_publish){

    sensor_msgs::PointCloud2 topub;
    Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
//    pcl::PointCloud<pcl::PointXYZ> a ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _laserCloud (new pcl::PointCloud<pcl::PointXYZ>());
    _laserCloud->clear();
    pcl::fromROSMsg(*laserCloudFullResMsg, *_laserCloud);

    Eigen::AngleAxisd odom_angle2 (M_PI/2, Eigen::Vector3d (1 ,0, 0));
    Eigen::AngleAxisd odom_angle3 (M_PI/2, Eigen::Vector3d (0 ,0, 1));
    pcd_rotate = odom_angle2*pcd_rotate;
    pcd_rotate = odom_angle3*pcd_rotate;
    pcl::transformPointCloud(*_laserCloud,*_laserCloud,pcd_rotate.matrix());
    pcl::toROSMsg(*_laserCloud,topub);
    std::ostringstream os;
    os.clear();
    auto sec = laserCloudFullResMsg->header.stamp.sec;
    auto nsec = laserCloudFullResMsg->header.stamp.nsec;
    os<<path<<sec<<"_"<<nsec<<".pcd";
    // todo 添加去除nan的点云
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*_laserCloud, *_laserCloud, indices);
    pcl::PointCloud<pcl::PointXYZ> pointCloud = *_laserCloud ;
    std::cout <<  _laserCloud->size() <<std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (_laserCloud);
    sor.setLeafSize (0.1f, 0.1f, 0.1f);
    sor.filter (*_laserCloud);
    //过滤nan点云
/*    if(!pcl_isfinite(pointCloud.points[1].x )){
        ROS_ERROR("wrong data");
        std::cout<<"skip data:  "<<os.str()<<std::endl;
    }else{
        std::cout<<"PCD saved as : " << os.str()<<std::endl;
        pcpub.publish(topub);
        pcl::io::savePCDFileASCII (os.str(), *_laserCloud);
    }*/
    std::cout<<"PCD saved as : " << os.str()<<std::endl;
    pcpub.publish(topub);
//    pcl::io::savePCDFileASCII (os.str(), *_laserCloud);
    indices.clear();
    _able_to_publish = false;
    }
}
//
void poseMessageReceived01hz ( const nav_msgs::Odometry::ConstPtr &laserOdometry ) {
    newOdom01hz = true;
    Eigen::Isometry3d _odom_rotate_temp = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d totalTf = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d rotation_vector = Eigen::Matrix3d::Identity();

    nav_msgs::Odometry transedOdomFunction;
    nav_msgs::Odometry a;
    transedOdomFunction = *laserOdometry;
    //当前的 odom到eigen  ==== =_odom_rotate_temp
    odometryToEigen(transedOdomFunction, _odom_rotate_temp);
    //y轴900 ,-1 ,0
    Eigen::AngleAxisd odom_angle(M_PI / 2, Eigen::Vector3d(0, -1, 0));
    //纠正loam自己tf
    _odom_rotate_temp.rotate(odom_angle);

    Eigen::AngleAxisd odom_angle2(M_PI / 2, Eigen::Vector3d(1, 0, 0));
    Eigen::AngleAxisd odom_angle3(M_PI / 2, Eigen::Vector3d(0, 0, 1));
    //复制头啥的
    a.header = transedOdomFunction.header;
    _odom_rotate_temp = odom_angle2 * _odom_rotate_temp;
    _odom_rotate_temp = odom_angle3 * _odom_rotate_temp;
    eigenToOdom(_odom_rotate_temp, a);
    transedOdom01hz = a;
    pub_01hz.publish(transedOdom01hz);
}

void polintcloudHandle01hz(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg){
    //_timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;
    //点云旋转矩阵
    sensor_msgs::PointCloud2 topub;
    topub = *laserCloudFullResMsg;

    Eigen::Isometry3d pcd_rotate = Eigen::Isometry3d::Identity();
//    pcl::PointCloud<pcl::PointXYZ> a ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _laserCloud (new pcl::PointCloud<pcl::PointXYZ>());//(new pcl::PointXYZ);
    _laserCloud->clear();
    pcl::fromROSMsg(*laserCloudFullResMsg, *_laserCloud);

    Eigen::AngleAxisd odom_angle2 (M_PI/2, Eigen::Vector3d (0 ,1, 0));
    Eigen::AngleAxisd odom_angle3 (M_PI/2, Eigen::Vector3d (1 ,0, 0));

    pcd_rotate = odom_angle2*pcd_rotate;
    pcd_rotate = odom_angle3*pcd_rotate;
    pcl::transformPointCloud(*_laserCloud,*_laserCloud,pcd_rotate.matrix());
    pcl::toROSMsg(*_laserCloud,topub);
    //保存
    std::ostringstream os;
    os.clear();
    auto sec = laserCloudFullResMsg->header.stamp.sec;
    auto nsec = laserCloudFullResMsg->header.stamp.nsec;
    os<<sec<<"."<<nsec<<".pcd";

    pcpub_01hz.publish(topub);
    //pcl::io::savePCDFileASCII (os.str(), *_laserCloud);
}

int main (int argc, char** argv) {

    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;
    if(argc == 4 || argc == 2){
        path = argv[1];
    }
    pub = nh.advertise<nav_msgs::Odometry>("/after_rotate", 1000) ;
    pcpub = nh.advertise<sensor_msgs::PointCloud2>("/pc_after_rotate", 1000) ;
    pub_01hz = nh.advertise<nav_msgs::Odometry>("/after_rotate_01hz", 1000) ;
    pcpub_01hz = nh.advertise<sensor_msgs::PointCloud2>("/pc_after_rotate_01hz", 1000) ;

    ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1000, &poseMessageReceived);
    ros::Subscriber sub_pcd = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 1000, &polintcloudHandle);
    //全速率的校准
    ros::Subscriber sub_01hz = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init", 1000, &poseMessageReceived01hz);
    ros::Subscriber sub_pcd_01hz = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 1000, &polintcloudHandle01hz);

    ros::Rate rate(10);
    while(ros::ok()) {

        ros::spinOnce();
        rate.sleep();
    }
}
