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
#include <ctime>
#include <stdint.h>
#include <opencv2/opencv.hpp>
#include "bundle_adjuster.hpp"
#include "camera_param_reader.hpp"

typedef unsigned int uint;

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

///////////////////////////////////////////////////////////////
// A new map type
// based on keyframes
class Keyframe
{
public:
  std::vector<cv::Point3f> coords_3D;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<int> keypoint_index;
  cv::Mat descriptors;
  cv::Mat pose;
  cv::Mat frame;
  Keyframe() {};
  Keyframe(cv::Mat p, cv::Mat f, int descriptor_size): pose(p), frame(f) 
  {
    time(&key_frame_init_time);
    descriptors = cv::Mat(0, descriptor_size, CV_8U);
  };
  bool is_good();

private:
  time_t key_frame_init_time;
  double keyframe_mean_error;
};

class KeyframesMap
{
public:
  std::vector<Keyframe> keyframes;
  KeyframesMap() {};
  KeyframesMap(std::vector<Keyframe> kf) : keyframes(kf) {}; 
  void operator<<(Keyframe keyframe) 
  {
    keyframes.push_back(keyframe);
  }
  Keyframe end() {
    if (keyframes.empty()) { return Keyframe(); }
    return keyframes.back();
  }

  void adjust(cv::Mat camera_matrix) {
    // Convert to right structure
    std::vector<cv::Mat> cams;
    std::vector<cv::Point3f> point3d;
    std::vector<cv::Point2f> meas;
    std::vector<int> cam_idx;
    std::vector<int> pt3d_idx;
    
    for (int n_kf = 0; n_kf < keyframes.size(); n_kf++) {
      cams.push_back(keyframes[n_kf].pose);
    }
    
    for (int n_kf = 0; n_kf < keyframes.size(); n_kf++) {
      for (int n_3dpt = 0; n_3dpt < keyframes[n_kf].coords_3D.size(); n_3dpt++) {
        point3d.push_back(keyframes[n_kf].coords_3D[n_3dpt]);

        cv::Point2f ppo = keyframes[n_kf].keypoints[n_3dpt].pt;
        ppo.x -= 640.0f;
        ppo.y -= 360.0f;
        meas.push_back(ppo);
        cam_idx.push_back(n_kf);
        pt3d_idx.push_back(point3d.size() - 1);
      }
    }
    bundle_adjustement_pba(cams, point3d, meas, cam_idx, pt3d_idx);
  }

  Map convert_to_map();
  

private:

  cv::Mat 
tr_vec_from_pose(cv::Mat pose)
{
  cv::Mat tr_vec(3, 1, CV_64F);
  tr_vec.at<double>(0,0) = pose.at<double>(0,3);
  tr_vec.at<double>(0,1) = pose.at<double>(1,3);
  tr_vec.at<double>(0,2) = pose.at<double>(2,3);
  return tr_vec;
}

cv::Mat 
rot_mat_from_pose(cv::Mat pose)
{
  cv::Mat rot_mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      rot_mat.at<double>(row,col) = pose.at<double>(row, col);
    }
  }
  return rot_mat;
}

};
///////////////////////////////////////////////////////////////


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

/*
// Convert to right structure
    std::vector<cv::Mat> cams;
    std::vector<cv::Point3f> point3d;
    std::vector<cv::Point2f> meas;
    std::vector<int> cam_idx;
    std::vector<int> pt3d_idx;
    for (int n_kf = 0; n_kf < keyframes.size(); n_kf++) {
      cams.push_back(keyframes[n_kf].pose);
      for (int n_3dpt = 0; n_3dpt < keyframes[n_kf].coords_3D.size(); n_3dpt++) {
        point3d.push_back(keyframes[n_kf].coords_3D[n_3dpt]);
        meas.push_back(keyframes[n_kf].keypoints[n_3dpt].pt);
        cam_idx.push_back(n_kf);
        pt3d_idx.push_back(n_3dpt);
      }
    }
    bundle_adjustement_pba(cams, point3d, meas, cam_idx, pt3d_idx);
*/
/*
std::vector<CloudPoint> pointcloud;
    std::vector<std::vector<cv::KeyPoint>> imgpts;
    std::map<int, cv::Matx34d> Pmats;
    int index = 0;
    int total_size = 0;
    for (auto &kf : keyframes) {
      for (int i = 0; i < kf.coords_3D.size(); i++) {
        CloudPoint cp;
        cp.pt = kf.coords_3D[i];
        cp.imgpt_for_img.push_back(i);
        pointcloud.push_back(cp);
      }
      total_size += kf.coords_3D.size();  
      imgpts.push_back(kf.keypoints);      
      cv::Mat R = rot_mat_from_pose(kf.pose);
      cv::Mat T = tr_vec_from_pose(kf.pose);
      cv::Matx34d P;
      P(0,0) = R.at<double>(0,0); P(0,1) = R.at<double>(0,1); P(0,2) = R.at<double>(0,2); P(0,3) = T.at<double>(0,0);
      P(1,0) = R.at<double>(1,0); P(1,1) = R.at<double>(1,1); P(1,2) = R.at<double>(1,2); P(1,3) = T.at<double>(1,0);
      P(2,0) = R.at<double>(2,0); P(2,1) = R.at<double>(2,1); P(2,2) = R.at<double>(2,2); P(2,3) = T.at<double>(2,0);
      Pmats[index] = P;
      index++;
    }
*/