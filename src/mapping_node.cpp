#include <ros/ros.h>
#include "mapping.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping");

  if (argc != 6 && argc != 4) {
    std::cerr << "Usage: rosrun mapping mapping path_to_g2ofile "
                 "path_to_g2ofile path_to_pcdfile"
              << std::endl;
    std::cerr << argc << std::endl;
    return 0;
  }

  std::cout << "构造函数前测试" << std::endl;
  // 1 loam位姿(0) 2. (0) 读优化好的
  ROS_INFO("Mapping !!");
  Mapping Mapping((std::string)argv[1], (std::string)argv[2],
                  (std::string)argv[3]);

  ros::spin();

  return 0;
}
