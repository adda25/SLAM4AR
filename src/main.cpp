#include <iostream>
#include <stdlib.h>
#include <fstream>
#include "asvs.hpp"

void test_map_system(std::vector<int> params);
void test_map_and_pose_system(std::vector<int> params);

int 
main(int argc, char** argv) {
  using namespace std;
  cout << "###  Test Eyes  ###" << endl;
  vector<int> params;
  for (int i = 0; i < argc; ++i) { params.push_back(atoi(argv[i])); }
  //test_map_system(params);
  test_map_and_pose_system(params);
  return 0;
}

void 
test_map_system(std::vector<int> params) {
  std::string camera_path = "/Users/adda/Work/TDOR2/out_camera_data_iOS_720_3.yml";
  cv::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0433.m4v";
  EyesHelper helper = EyesHelper(camera_path);
  cv::VideoCapture capture(video_path);
  std::vector<std::vector<Match>> total_matches = helper.slam_map(capture, params[1], params[2]);
  
  // Write point cloud to file
  std::ofstream ofs;
  ofs.open ("point_cloud.txt", std::ofstream::out | std::ofstream::app);
  for (auto &sm : total_matches) {
    for (auto &m : sm) {
      //std::cout << m.match_point_in_marker_frame << std::endl;
      ofs << m.match_point_in_marker_frame[0].x << " " 
        << m.match_point_in_marker_frame[0].y << " " 
          << m.match_point_in_marker_frame[0].z << " " 
            << (int)m.colour.val[2] << " " << (int)m.colour.val[1] << " " << (int)m.colour.val[0] << "\n";
    }
  }
  ofs.close();
}

void 
test_map_and_pose_system(std::vector<int> params) {
  std::string camera_path = "/Users/adda/Work/TDOR2/out_camera_data_iOS_720_3.yml";
  cv::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0433.m4v";
  cv::string video_path_test = "/Users/adda/Work/Eyes/src/video/IMG_0434.m4v";
  EyesHelper helper = EyesHelper(camera_path);
  cv::VideoCapture capture(video_path);
  cv::VideoCapture capture_test(video_path_test);
  std::vector<std::vector<Match>> total_matches = helper.slam_map(capture, params[1], params[3]);
  helper.slam_localize(capture_test, params[2], total_matches);
}