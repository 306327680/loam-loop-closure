//
// Created by echo on 18-7-25.
//

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include "g2o/core/factory.h"
#include "g2o/stuff/command_args.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

#include "g2o/core/factory.h"
//#include "g2o/types/slam3d/types_slam3d.h"
//#include "g2o/types/slam2d/types_slam2d.h" //must必须注释
#include "g2o/stuff/command_args.h"

#ifndef G2OODOMSAVE_LMOPTIMIZER_H
#define G2OODOMSAVE_LMOPTIMIZER_H


class LMoptimizer {
public:
    LMoptimizer(){};
    void setup(ros::NodeHandle& node,
               ros::NodeHandle& privateNode);
    void lmStart(const std_msgs::Bool::ConstPtr &loopClosure);
    void print4x4Matrix (const Eigen::Isometry3d & matrix);

    //ros spin
    int spin();
    int process();
private:
    int maxIterations = 100;
    std::string outputFilename = "LidarOdomOut.g2o";
    std::string inputFilename = "LidarOdom.g2o";
    ros::Subscriber _loopclosure;
    ros::Publisher _mappping;
    bool _g2ofinish = false;
    std_msgs::Bool _to_mapping;

};


#endif //G2OODOMSAVE_LMOPTIMIZER_H
