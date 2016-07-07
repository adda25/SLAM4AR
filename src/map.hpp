#ifndef __MAP_HPP__
#define __MAP_HPP__
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

class KeyframePoint
{
public:
  KeyframePoint() {};
  KeyframePoint(cv::Mat &initial_descriptor, cv::Mat &initial_keypoint) {
  	this->initial_descriptor = initial_descriptor;
  	this->initial_keypoint   = initial_keypoint;
  }
  
  cv::Point3f 
  compute_coord_in_3d_space(cv::Mat &pose);
  
  cv::Mat initial_descriptor;
  cv::Mat initial_keypoint;
  cv::Point3f coord_in_3d_space;

  std::string identifier;
private:
  
};

class SectorKeyframe
{
public:
  SectorKeyframe() {};
  
  cv::Mat keyframe_pose;
  std::vector<KeyframePoint> keyframe_points;

private:

};

/* One sector
of the slam map
*/
class MapSector
{
public:
  MapSector() {};
  MapSector(int sector_id) {
  	identifier = sector_id;
  };
  
  bool 
  is_empty() { return sector_keyframes.size() == 0; }

  bool 
  is_coord_inside_sector(cv::Point3f);

  std::vector<SectorKeyframe> sector_keyframes;
  int identifier;
  
private:
  int x_bounds[2];
  int y_bounds[2];
  int z_bounds[2];

};

class SlamMapGeneral
{
public:
  virtual void init() = 0;
  virtual void update() = 0;
  virtual void retrive() = 0;
  virtual std::vector<KeyframePoint> keyframes_points() = 0;

  virtual void write(std::string filename);
  virtual void draw(cv::Mat frame);
};


/* This class is the
recostructed map of the slam
system
*/
class SlamMap
{
public:
  SlamMap() {};
  /* Init with the desired number of map sectors 
  and sector size (mm) 
  */
  SlamMap(uint sectors_for_each_axis, uint sector_side_lenght) {
  	uint total_sectors = sectors_for_each_axis * sectors_for_each_axis * sectors_for_each_axis;
    for (int i = 0; i < total_sectors; i++) {
      sectors.push_back(MapSector(i));
    }
  };
  
  /* Rescale entire map in order to center
  the map in this sector 
  */
  void 
  fill_initial_sector();
  
  void
  set_new_data();

  /* Returns the sectors in which the 
  camera could be seeing 
  */
  std::vector<MapSector>
  sectors_for_pose(cv::Mat pose);

  /* Returns the sectors around the
  specified center coords and size 
  */
  std::vector<MapSector>
  sectors_for_coords_and_size(int xc, int yc, int zc, int size);

  // 
  void
  write_point_cloud_to_file();
  //
  void 
  draw_map_on_image();


  std::vector<MapSector> sectors;
  uint axis_map_sectors;
  uint sector_lenght; // mm
private:

};






#endif // End __MAP_HPP__