#ifndef __MAP_HPP__
#define __MAP_HPP__
#include <vector>
#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>


typedef struct 
{
  cv::Point3f coords_3D;
  cv::Point3f coords_3D_camera_sys;
  cv::KeyPoint keypoint;
  cv::Mat descriptor;	
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
Map map__create(uint sectors[3], uint sector_size[3]);

void map__update(Map &map, std::vector<MapPoint> points, const cv::Mat pose, const cv::Mat &frame);

void map__remove_empty_sectors(Map &map);

int map__sector_for_coords(Map &map, cv::Point3f coords_3D);

/* Returns the partial map that the
camera is viewing */
Map map__sectors_in_view(const Map &map, cv::Mat camera_pose);

void map__draw(cv::Mat &frame_to_draw, const Map &map_to_draw, bool draw_grid = false);
void map__write(std::string filename, const Map &map_to_write, bool write_grid = false);




#endif // End __MAP_HPP__

