#ifndef __ASVS_HPP__
#define __ASVS_HPP__

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <thread>
#include <future>
#include "slam.hpp"
#include "marker.hpp"

class SlamAPI
{
public:
  SlamAPI(std::string camera_path) 
  {
    slam_sys_init(camera_path);
  }

  void slam_sys_init(std::string camera_path);

  void map_init(); 

  /* Compute the map using the
  5 point algorithm  */
  //void map_update(cv::Mat &frame, cv::Mat pose, int kk);

  /* Compute the map given the pose
  of the current frame */
  void map_update(cv::Mat &frame, cv::Mat pose);

  /* TODO: Compute the map with stereo cameras */
  void map_update_stereo(cv::Mat &frame_1, cv::Mat &frame_2);

  void map_remove_old_sectors();
  void map_reset();
  void map_write_point_cloud(std::string filename, bool grid = false);
  void map_save(std::string filename);
  void map_save(std::string filename, const Map &map_to_save);
  void map_load(std::string filename);
  void map_load(std::string filename, Map &map_to_load);

  void localize(cv::Mat &frame, cv::Mat &estimated_pose);
  void localize_and_update(cv::Mat &frame, cv::Mat &estimated_pose);
  void localize_object(cv::Mat &frame, const Map &object_map, std::vector<cv::Rect> &objects_rects); 
    
  void visualize(cv::Mat &frame, cv::Mat &pose, bool draw_map, bool draw_sr);

  Map map;
  SlamSystem slam_sys;
  double min_dist = 30.0;
  int map_point_size = 0;
private:
  

};

/*
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
*/

#endif // End __ASVS_HPP__
