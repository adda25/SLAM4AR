#ifndef __ASVS_HPP__
#define __ASVS_HPP__

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
#include <fstream>
#include "slam.hpp"
#include "marker.hpp"

class SlamAPI
{
public:
  SlamAPI(std::string cam_path) {
    camera_path = cam_path;
  }
  
  std::string camera_path;

  Map  slam_map(cv::VideoCapture capture, int count, float min_dist);
  void slam_localize(cv::VideoCapture capture_test, int count_test, Map map);  
  void slam_find_objects(cv::VideoCapture capture, int count_test, std::vector<Map> objects_maps);
};


#endif // End __ASVS_HPP__
