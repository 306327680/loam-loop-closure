//
// Created by alter on 8/23/18.
//
#include "pcd_finder.h"

#include <iostream>

int main(int argc, char **argv) {
  if (argc != 4 && argc != 6) {
    for (int u = 0; argc > u; u++) {
      std::cerr << "  argv :: " << argv[u] << std::endl;
    }

    std::cerr << "Usage: rosrun find find PATH_TO_PCD_BUFFER prior after"
              << std::endl;
    return 1;
  }
  ros::init(argc, argv, "find");
  PCDFinder finder;
  if (!finder.Init(std::string(argv[1]), argv[2], argv[3])) {
    return 1;
  }
  ros::spin();
  return 0;
}
