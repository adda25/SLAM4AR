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
#include <cstdint>
#include "/usr/local/include/eigen/Eigen/Dense"
#include "camera_param_reader.hpp"
#include "robust_matcher.hpp"

typedef struct
{
  cv::KeyPoint keypoint;
  cv::Mat descriptor;
} SingleMatchPoint;

typedef struct 
{
  cv::Point3d coords3d;
  cv::Point3d coords3d_2;
  cv::Mat reference_pose;
  cv::Mat reference_pose_2;
  cv::Point2f coords2d;
  cv::Point2f coords2d_2;
  cv::KeyPoint keypoint1;
  cv::KeyPoint keypoint2;
  cv::Mat descriptor1;
  cv::Mat descriptor2;
  int sector;
  cv::Vec3b colour;
  
  cv::vector<cv::Point3f> match_point_in_marker_frame;
  
  void match_position_in_marker_frame(cv::Mat camera_pose) {
    cv::Mat from_cam_2_match = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat points = cv::Mat(3, 1, CV_64F);
    cv::Mat rot = cv::Mat(3, 3, CV_64F);
    from_cam_2_match.at<double>(0,3) = coords3d.x;
    from_cam_2_match.at<double>(1,3) = coords3d.y;
    from_cam_2_match.at<double>(2,3) = coords3d.z;
    cv::Mat res = camera_pose * from_cam_2_match;
    cv::Point3f match_in_marker = cv::Point3f(res.at<double>(0,3), res.at<double>(1,3), res.at<double>(2,3));
    match_point_in_marker_frame.push_back(match_in_marker);
  }
      
  cv::vector<cv::Point2f> calc_image_point_for_frame(MyCamereParamReader& camera, cv::Mat rvec, cv::Mat tvec) {
    cv::Mat result(4, 4, CV_64FC1);
    cv::Mat rot;
    cv::Rodrigues(rvec, rot);
    for (int i = 0; i < 3; i++) {
      for (int k = 0; k < 3; k++) {
        result.at<double>(i,k) = rot.at<double>(i,k);
      }
    }
    result.at<double>(0,3) = tvec.at<double>(0,0);
    result.at<double>(1,3) = tvec.at<double>(1,0);
    result.at<double>(2,3) = tvec.at<double>(2,0);
    result.at<double>(3,0) = 0;
    result.at<double>(3,1) = 0;
    result.at<double>(3,2) = 0;
    result.at<double>(3,3) = 1;
    match_position_in_marker_frame(reference_pose.inv());
    cv::vector<cv::Point2f> image_point_2_return;
    cv::projectPoints(match_point_in_marker_frame, rvec, tvec, camera.getCameraMatrix(), camera.getDistorsion(), image_point_2_return);
    return image_point_2_return;
  }
} Match;


class Slam 
{  
public:
  Slam() {};
  Slam(std::string camera_path) {
    robust_matcher = RobustMatcher();
    features_detector = new cv::OrbFeatureDetector(4000);
    descriptions_extractor = new cv::OrbDescriptorExtractor();
    camera = MyCamereParamReader();
    camera.readFromXMLFile(camera_path);
  }
  
  std::vector<Match> map_2_images(cv::Mat image1, cv::Mat image2, cv::Mat relative_pose);
  std::vector<Match> map_2_images(cv::Mat image1, cv::Mat pose1, cv::Mat image2, cv::Mat pose2, std::vector<Match> old_matches);
  cv::Mat calc_pose_with_matches(cv::Mat image, std::vector<Match> matches);
  
  std::vector<cv::KeyPoint> search_features(cv::Mat image);
  cv::Mat extract_descriptors(cv::Mat image, std::vector<cv::KeyPoint> keypoints);
  std::vector<cv::DMatch> match_features(std::vector<cv::KeyPoint> keypoints1, 
                                         cv::Mat descriptors1, 
                                         std::vector<cv::KeyPoint> keypoints2, 
                                         cv::Mat descriptors2);
                                         
  void split_matches(std::vector<cv::DMatch>   matches, 
                     std::vector<cv::KeyPoint> keypoints1,
                     cv::Mat descriptors1, 
                     std::vector<cv::KeyPoint> keypoints2, 
                     cv::Mat descriptors2,
                     std::vector<SingleMatchPoint>& match1, 
                     std::vector<SingleMatchPoint>& match2);
                     
  std::vector<Match> triangulate(cv::Mat image1,
                     std::vector<SingleMatchPoint> matched_points1, 
                     cv::Mat pose1, 
                     std::vector<SingleMatchPoint> matched_points2, 
                     cv::Mat pose2);
                   
  std::vector<cv::DMatch> match_with_features(std::vector<cv::KeyPoint> keypoints1, 
                                              cv::Mat descriptor1, 
                                              std::vector<cv::KeyPoint> keypoints2, 
                                              cv::Mat descriptor2);
  
  cv::Mat estimated_pose(std::vector<Match> matches);
  cv::Mat estimated_pose(cv::vector<cv::Point2f> img_points_vector, 
                         cv::vector<cv::Point3f> obj_points_vector);
  
  void draw_estimated_map(std::vector<Match> matches, 
                          cv::Mat& image, 
                          cv::Mat tr_vec, 
                          cv::Mat rot_vec);
  
  void refine_old_matches();
    
  cv::Ptr<cv::FeatureDetector> features_detector;
  cv::Ptr<cv::DescriptorExtractor> descriptions_extractor;
  RobustMatcher robust_matcher;
  MyCamereParamReader camera;  
  float closest_match_distance = 1.0;

private:
  std::vector<double> solve_linear_system(cv::Point3f p11, cv::Point3f p12, cv::Point3f p21, cv::Point3f p22);
  bool find_closest_match(SingleMatchPoint point_to_search, std::vector<Match> reference_matches, Match& closest_match);
};

#endif // End __SLAM_HPP