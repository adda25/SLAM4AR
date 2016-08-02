/*
 ____  _                 
/ ___|| | __ _ _ __ ___  
\___ \| |/ _` | '_ ` _ \ 
 ___) | | (_| | | | | | |
|____/|_|\__,_|_| |_| |_|
                         
*/
//
//  slam.hpp
//  Eyes
//
//  Created by Amedeo Setti on 20/06/16.
//  Copyright (c) 2016 Amedeo Setti. All rights reserved.
//

#ifndef __SLAM_HPP__
#define __SLAM_HPP__
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <cstdint>
#include <fstream>
#include <future>
#include <thread>
#include "map.hpp"

void slam__init(SlamSystem &slam_sys, const std::string camera_path);

#if CV_MAJOR_VERSION == 3
std::vector<MapPoint> slam__map(const SlamSystem &slam_sys, 
                                cv::Mat image_1, 
                                cv::Mat image_2);
#endif

std::vector<MapPoint> slam__map(const SlamSystem &slam_sys, 
                                cv::Mat &image_1, 
                                cv::Mat &image_2, 
                                const cv::Mat &pose_1, 
                                const cv::Mat &pose_2);

void slam__adjust_map(const SlamSystem &slam_sys);

cv::Mat slam__localize(const SlamSystem &slam_sys, 
                       const Map &map, 
                       cv::Mat &image);

std::vector<cv::Rect> slam__find_objects(const SlamSystem &slam_sys, 
                                         std::vector<Map> objects_maps, 
                                         cv::Mat &image);

cv::Mat slam__estimated_pose(std::vector<MapPoint> matches, CameraSystem camera); 
cv::Mat slam__estimated_pose(std::vector<cv::Point2f> img_points_vector, 
                             std::vector<cv::Point3f> obj_points_vector, 
                             CameraSystem camera); 

void draw_cube_on_ref_sys(cv::Mat &image, 
                          cv::Mat camera_matrix, 
                          cv::Mat camera_dist, 
                          cv::Mat pose, 
                          uint side_lenght, 
                          cv::Scalar color); 

// AUX FOR EXTERNAL
void compute_params_for_image(const SlamSystem &slam_sys, ImageForMapping &ifm);


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