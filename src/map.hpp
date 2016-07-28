/*
 __  __             
|  \/  | __ _ _ __  
| |\/| |/ _` | '_ \ 
| |  | | (_| | |_) |
|_|  |_|\__,_| .__/ 
             |_|    
*/

#ifndef __MAP_HPP__
#define __MAP_HPP__
#include <vector>
#include <string>
#include <fstream>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#include "camera_param_reader.hpp"

typedef struct
{
  cv::Mat frame;
  std::vector<cv::KeyPoint> keypoint;
  cv::Mat descriptor;
  cv::Mat pose;
} ImageForMapping;

typedef struct 
{
  #if CV_MAJOR_VERSION == 2
  cv::FeatureDetector *features_detector;
  cv::DescriptorExtractor *descriptions_extractor;
  #elif CV_MAJOR_VERSION == 3
  cv::Ptr<cv::ORB> features_detector;
  cv::Ptr<cv::ORB> descriptions_extractor;
  #endif
  cv::BFMatcher matcher;
  CameraSystem camera;  
} SlamSystem;

typedef struct 
{
  cv::Point3f coords_3D;
  cv::Point3f coords_3D_camera_sys;
  cv::KeyPoint keypoint;
  cv::Mat descriptor = cv::Mat(1, 128, CV_8U);   //// TODO CHANGE 32
  uint hits = 0;
  cv::Point2f point_pixel_pos;
  cv::Vec3b pixel_colors;
  cv::Mat pose;
} MapPoint;

typedef struct 
{
  int sector_bounds[6];
  std::vector<MapPoint> sector_points;
  std::vector<cv::Mat>  sector_frames;
  std::vector<cv::Mat>  sector_poses;
} MapSector;


typedef std::vector<MapSector> Map;

/*  sectors[3] = [n_x, n_y, n_z]
sector_size[3] = [s_x, s_y, s_z] 
*/
Map  map__create(uint sectors[3], uint sector_size[3]);

void map__update(Map &map, std::vector<MapPoint> points, const cv::Mat pose, const cv::Mat &frame);

void 
map__only_enlarge(Map &map, 
            std::vector<MapPoint> points, 
            const cv::Mat pose, 
            const cv::Mat &frame);

void map__remove_empty_sectors(Map &map, uint min_size = 0);

int  map__sector_for_coords(Map &map, cv::Point3f coords_3D);

/* Returns the partial map that the
camera is viewing */
Map  map__sectors_in_view(CameraSystem &camera, const Map &map, cv::Mat camera_pose);

void map__draw(cv::Mat &frame_to_draw, 
               const Map &map_to_draw, 
               cv::Mat tr_vec, 
               cv::Mat rot_vec, 
               const cv::Mat camera_matrix, 
               const cv::Mat camera_distorsion,
               bool draw_grid = false);

/* Write in the file the coords
of the map points. If write_grid is true,
the function also write the map grid.
*/
void map__write(std::string filename, const Map &map_to_write, bool write_grid = false);

void map__save_on_file(std::string filename, const Map &map_to_write);
Map  map__load_from_file(std::string filename);

/* Fills the keypoint vector and
the descriptor Mat with the keypoints and descriptors
of all sectors of the map.
*/
void map__merge_keypoints_and_descriptors(const Map &map, 
                                          std::vector<cv::KeyPoint> &keypoints_out, 
                                          cv::Mat &descriptors_out);

void map__merge(const Map &map, std::vector<MapPoint> &me_map);

void map__image_points_for_pose(const Map &map, 
                                cv::Mat current_tr,
                                cv::Mat current_rot, 
                                cv::Mat camera_matrix,
                                cv::Mat camera_distorsion,
                                std::vector<MapPoint> &image_points_out);


#endif // End __MAP_HPP__

