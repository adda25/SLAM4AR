#ifndef __ASVS_HPP__
#define __ASVS_HPP__

#include <opencv2/opencv.hpp>
#include <vector>
#include "slam.hpp"
#include "marker.hpp"

class EyesHelper
{
public:
  EyesHelper(std::string cam_path) {
    slam = Slam(cam_path);
    camera_path = cam_path;
  }
  
  Slam slam;
  std::string camera_path;
  
  std::vector<std::vector<Match>> slam_map(cv::VideoCapture capture, int count, float min_dist);
  
  void slam_localize(cv::VideoCapture capture_test, int count_test, std::vector<std::vector<Match>> matches);
  
};


#endif // End __ASVS_HPP__
