//
// Created by echo on 18-7-25.
//

//TODO 把这些的ros初始化 写了
//input 读入哪两个pcd 初始位姿估计 ///velodyne_cloud_3 这个是每一帧对应的点云,可以进行降采样保存/
// integrated_to_init 也是每一帧的,可以进行保存
//output 两个图之间的位姿变换
//print 输出 score

#include "ndtTransCalc.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "NdtOdom");
    ros::NodeHandle node;
    ros::NodeHandle privateNode("~");
    ndtcalc ndtcalc;
    ndtcalc.setup(node,privateNode);
    ndtcalc.spin();
}

void ndtcalc::groundZFilter(int max, int min, pcl::PointCloud<pcl::PointXYZ> &pointIn){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
    *cloudIn = pointIn;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloudIn;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.2, 10.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    pointIn = *cloud_filtered;
}

void ndtcalc::groundZExtract(int max, int min, pcl::PointCloud<pcl::PointXYZ> &pointIn)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
    *cloudIn = pointIn;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    cloud = cloudIn;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.3, 10.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    pointIn = *cloud_filtered;
}
void ndtcalc::print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("\e[0;32m""Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}
void ndtcalc::print4x4Matrix (const Eigen::Isometry3d & matrix)
{
    printf ("\e[0;32m""Rotation matrix of current TF :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


void ndtcalc::setup(ros::NodeHandle &node, ros::NodeHandle &privateNode)
{
    _picNumSub = node.subscribe<nav_msgs::Odometry>("/loop_coarse_tf",5, &ndtcalc::picNumHandler, this);
    //点云读取
    _pointCloudSub = node.subscribe<sensor_msgs::PointCloud2>("/ndt_points_past",5, &ndtcalc::pointCloudHandler, this);
    _pointCloudSubCur = node.subscribe<sensor_msgs::PointCloud2>("/ndt_points_cur",5, &ndtcalc::pointCloudCurHandler, this);
    _curPointForTf = node.advertise<sensor_msgs::PointCloud2>("/ndted_points_cur_test",10);
    _pastPointForTf = node.advertise<sensor_msgs::PointCloud2>("/ndted_points_past_test",10);
    //
    _pubLoopClosureTrans = node.advertise<nav_msgs::Odometry> ("/ndt_loop_tf", 1);
}

int ndtcalc::spin()
{
    ros::Rate rate(0.5);
    while (ros::ok())
    {
        process();
        ros::spinOnce();
        rate.sleep();
    }
        //储存 需要的结果
    if(!ros::ok())
    {

    }
    return 0;
}

void ndtcalc::process()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_gnd (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_gnd (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_gnd (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_gnd (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_test (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_output (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> result;

    if(_cur_process < _cloud_last.size()  && _cur_process <_cloud_curr.size() && _cur_process< _odom_seq.size()){
        //if(_new_cloud_last && _new_cloud_curr){
        _new_cloud_last = false;
        _new_cloud_curr = false;
        cout<<"New loop closure and Total loop times :   "<<_loop_times<<endl;

        bool show = true;
        auto curr_nsec = ros::Time::now().nsec;
        auto curr_sec = ros::Time::now().sec;
        //todo 设计一个vector长度比较, 确保两贞对应;

        *target_cloud = _cloud_last[_cur_process];
        *input_cloud = _cloud_curr[_cur_process];
        cout<<"The current input_cloud size is :"<<input_cloud->size()<<endl;
        cout<<"The current target_cloud size is :"<<target_cloud->size()<<endl;

//        voxel_filter.setLeafSize (0.8, 0.8, 0.2);
//        voxel_filter.setInputCloud (input_cloud);
//        voxel_filter.filter (*filtered_cloud);
//        voxel_filter.setLeafSize (0.8, 0.8, 0.2);
//        voxel_filter.setInputCloud (target_cloud);
//        voxel_filter.filter (*filtered_target_cloud);
        voxel_filter.setLeafSize (0.2, 0.2, 0.2);
        voxel_filter.setInputCloud (input_cloud);
        voxel_filter.filter (*filtered_cloud);
        voxel_filter.setLeafSize (0.2, 0.2, 0.2);
        voxel_filter.setInputCloud (target_cloud);
        voxel_filter.filter (*filtered_target_cloud);

        //把参考点云也进行滤波
        cout<<"The current filtered_cloud size is :"<<filtered_cloud->size()<<endl;


        std::cout << "Filtered cloud contains " << filtered_cloud->size ()
                  << " data points from findPCD" << std::endl;
//ndt
//        ndt.setTransformationEpsilon (0.01);//0.001
//        ndt.setStepSize (1);
//        //ndt.setResolution (100.0);
//        ndt.setResolution (2.0);
//        ndt.setMaximumIterations (100);
//        ndt.setInputSource (filtered_cloud);
//        //改动8/22
//        ///ndt.setInputTarget (filtered_target_cloud);
//        ndt.setInputTarget (target_cloud);
//        Eigen::Matrix4f init_guess;
//        Eigen::Matrix4d init_guess1;
//        //note 这里先设成单位阵
//        //init_guess1 =_relativeTF.back().matrix();
//        init_guess1 = Eigen::Isometry3d::Identity().matrix();
//        init_guess = init_guess1.cast<float>();// = (init_translation * init_rotation).matrix ();
//
//        cout<<" ndt.align"<<endl;
//        ndt.align (*output_cloud, init_guess);
//        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
//                  << " score: " << ndt.getFitnessScore () << std::endl;
//
//        _final_tf = ndt.getFinalTransformation ().cast<double>();
//        pcl::transformPointCloud (*filtered_cloud, *output_cloud, ndt.getFinalTransformation ());
//        print4x4Matrix(ndt.getFinalTransformation ().cast<double>());

        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setTransformationEpsilon(1e-6);
        icp.setMaximumIterations (200);
        icp.setEuclideanFitnessEpsilon(1e-6);
        //改动8/22
//        icp.setInputSource (output_cloud);
        icp.setInputSource (filtered_cloud);
        icp.setInputTarget (filtered_target_cloud);
        icp.align (*icp_output);
        //icp.setMaximumIterations (100);  // We set this variable to 1 for the next time we will call .align () function
        //测试新旋转矩阵

        if (icp.hasConverged () && icp.getFitnessScore()<1)
        {
            _loop_times++;
            std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
            print4x4Matrix (icp.getFinalTransformation ().cast<double>());
            //this->showPointCloud(target_cloud,filtered_cloud,icp_output);
            //改回来
            _final_tf = icp.getFinalTransformation ().cast<double>();// * _final_tf.matrix();

            pcl::transformPointCloud (*filtered_cloud, *output_cloud_test, icp.getFinalTransformation ());

            std::cout << " The total tf is :"<<std::endl;
            print4x4Matrix(_final_tf);
            //show = false;
            if(show){
                sensor_msgs::PointCloud2 to_pub_cur;
                sensor_msgs::PointCloud2 to_pub_past;
                pcl::PCLPointCloud2 pcl_pc1;
                pcl::PCLPointCloud2 pcl_pc2;
                pcl::toPCLPointCloud2(*output_cloud_test,pcl_pc1);
                pcl::toPCLPointCloud2(*target_cloud,pcl_pc2);
                pcl_conversions::fromPCL(pcl_pc1,to_pub_cur);
                pcl_conversions::fromPCL(pcl_pc2,to_pub_past);
                to_pub_cur.header.frame_id = "/camera_init";
                to_pub_past.header.frame_id = "/camera_init";
                ROS_WARN("Publish TFed cloud");
                _curPointForTf.publish(to_pub_cur);
                _pastPointForTf.publish(to_pub_past);
                //this->showPointCloud(target_cloud,filtered_cloud,output_cloud_test);
            }
        } else {
            PCL_WARN ("ICP has not converged.");
            std::cout<<"score is "<<icp.getFitnessScore()<<std::endl;
            _cur_process++;
            _new_loop = false;
            return;
        }
        //////

        //this->showPointCloud(target_cloud,filtered_cloud,output_cloud);
        //TODO 显示到rviz
        result.clear();
        result = *input_cloud+*output_cloud;
        publishNdt(icp.getFinalTransformation ().cast<double>(),_cur_process);

        curr_nsec = ros::Time::now().nsec - curr_nsec;
        curr_sec = ros::Time::now().sec - curr_sec;
        cout <<" Current calculation time is :  " << curr_sec<<"."<<curr_nsec<<endl;
        _cur_process++;
    }

    _new_loop = false;
}



void ndtcalc::showPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    //************************ 可视化
    pcl::visualization::PCLVisualizer viewer ("compare");
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h
            (target_cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
             (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (target_cloud, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (target_cloud, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (input_cloud, 20, 180, 20);
    viewer.addPointCloud (input_cloud, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (output_cloud, 180, 20, 20);
    viewer.addPointCloud (output_cloud, cloud_icp_color_h, "cloud_icp_v2", v2);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    // viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce (100);
       //boost::this_thread::sleep (boost::posix_time::microseconds (10000));

    }
    viewer.close();
}

void ndtcalc::publishNdt(const Eigen::Matrix4d &matrix, int cur_process)
{
    //不知道转化的对不对
    Eigen::Isometry3d a(matrix);
    Eigen::Quaterniond q(a.rotation());
    _ndtTfed.twist.covariance =_odom_seq[cur_process].twist.covariance;
    //_ndtTfed.twist.covariance = _transedOdomFunction.twist.covariance;
    _ndtTfed.header.frame_id = "map";

    _ndtTfed.pose.pose.position.x = matrix (0, 3);
    _ndtTfed.pose.pose.position.y = matrix (1, 3);
    _ndtTfed.pose.pose.position.z = matrix (2, 3);
    _ndtTfed.pose.pose.orientation.x = q.x();
    _ndtTfed.pose.pose.orientation.y = q.y();
    _ndtTfed.pose.pose.orientation.z = q.z();
    _ndtTfed.pose.pose.orientation.w = q.w();
    _pubLoopClosureTrans.publish(_ndtTfed);
}

void ndtcalc::publishNdt(const Eigen::Matrix4d &matrix)
{
    //不知道转化的对不对
    Eigen::Isometry3d a(matrix);
    Eigen::Quaterniond q(a.rotation());
    _ndtTfed.twist.covariance = _transedOdomFunction.twist.covariance;
    _ndtTfed.header.frame_id = "map";

    _ndtTfed.pose.pose.position.x = matrix (0, 3);
    _ndtTfed.pose.pose.position.y = matrix (1, 3);
    _ndtTfed.pose.pose.position.z = matrix (2, 3);
    _ndtTfed.pose.pose.orientation.x = q.x();
    _ndtTfed.pose.pose.orientation.y = q.y();
    _ndtTfed.pose.pose.orientation.z = q.z();
    _ndtTfed.pose.pose.orientation.w = q.w();
    _pubLoopClosureTrans.publish(_ndtTfed);
}
//订阅初值
void ndtcalc::picNumHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
    //两贞之间相对的tf,存为eigen
    Eigen::Isometry3d _odom_rotate_temp = Eigen::Isometry3d::Identity();

    _transedOdomFunction = *laserOdometry;
    odometryToEigen(_transedOdomFunction, _odom_rotate_temp);
    _odom_seq.push_back(_transedOdomFunction);
    _relativeTF.push_back(_odom_rotate_temp);
    _new_loop = true;
    _new_data = true;
}
//订阅过去点云
void ndtcalc::pointCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg) {
    sensor_msgs::PointCloud2 topub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _laserCloud (new pcl::PointCloud<pcl::PointXYZ>());
    _laserCloud->clear();
    pcl::fromROSMsg(*laserCloudFullResMsg, *_laserCloud);
    _cloud_last.push_back(*_laserCloud);
    _new_cloud_last = true;
    _new_data = true;
    ROS_WARN("New past PC");
}
//订阅当前点云
void ndtcalc::pointCloudCurHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullResMsg) {
    sensor_msgs::PointCloud2 topub;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _laserCloud (new pcl::PointCloud<pcl::PointXYZ>());
    _laserCloud->clear();
    pcl::fromROSMsg(*laserCloudFullResMsg, *_laserCloud);
    _cloud_curr.push_back(*_laserCloud);
    _new_cloud_curr = true;
    _new_data = true;
    ROS_WARN("New cur PC");
}

void ndtcalc::odometryToEigen(nav_msgs::Odometry &o, Eigen::Isometry3d &e) {
    Eigen::Isometry3d is_test;
    is_test = Eigen::Isometry3d::Identity();
    tf::Transform tf_test;
    tf::poseMsgToTF(o.pose.pose,tf_test);
    tf::transformTFToEigen(tf_test,e);
}