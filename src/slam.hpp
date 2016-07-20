//
//  slam.hpp
//  Eyes
//
//  Created by Amedeo Setti on 20/06/16.
//  Copyright (c) 2015 Amedeo Setti. All rights reserved.
//

#ifndef __SLAM_HPP__
#define __SLAM_HPP__
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <chrono>
#include <string>
#include <cstdint>
#include <fstream>
//#include "/usr/local/include/eigen/Eigen/Dense"
#include "camera_param_reader.hpp"
#include "map.hpp"

typedef struct 
{
  cv::FeatureDetector *features_detector;
  cv::DescriptorExtractor *descriptions_extractor;
  MyCamereParamReader camera;  
} SlamSystem;


void slam__init(SlamSystem &slam_sys, const std::string camera_path);

std::vector<MapPoint> slam__map(const SlamSystem &slam_sys, 
                                cv::Mat &image_1, 
                                cv::Mat &image_2, 
                                const cv::Mat &pose_1, 
                                const cv::Mat &pose_2);

cv::Mat slam__localize(const SlamSystem &slam_sys, 
                       const Map &map, 
                       cv::Mat &image);

std::vector<cv::Rect> slam__find_objects(const SlamSystem &slam_sys, 
                                         std::vector<Map> objects_maps, 
                                         cv::Mat &image);

// TODO
cv::Mat slam_localize_and_update(const SlamSystem &slam_sys, Map &map, cv::Mat &image);
//

cv::Mat slam__estimated_pose(std::vector<MapPoint> matches, MyCamereParamReader camera); 
cv::Mat slam__estimated_pose(cv::vector<cv::Point2f> img_points_vector, 
                             cv::vector<cv::Point3f> obj_points_vector, 
                             MyCamereParamReader camera); 

void draw_cube_on_ref_sys(cv::Mat &image, 
                          cv::Mat camera_matrix, 
                          cv::Mat camera_dist, 
                          cv::Mat pose, 
                          uint side_lenght, 
                          cv::Scalar color); 

// AUX
cv::Mat tr_vec_from_pose(cv::Mat pose);
cv::Mat rot_mat_from_pose(cv::Mat pose);

class MapMatchFt_C 
{
public:
  MapPoint new_found_ft; 
  MapPoint new_found_ft_in_new_image; 
  MapPoint ref_match;
  int accuracy_index;
  bool operator>(MapMatchFt_C rhs) const { return accuracy_index > rhs.accuracy_index; }
  bool operator<(MapMatchFt_C rhs) const { return accuracy_index < rhs.accuracy_index; }
};

#endif // End __SLAM_HPP