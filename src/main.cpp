#include <iostream>
#include <stdlib.h>
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
  std::vector<Match> matches = helper.slam.join_matches(total_matches);
  helper.slam.write_point_cloud("point_cloud.txt", matches);
}

void 
test_map_and_pose_system(std::vector<int> params) {
  std::string camera_path = "/Users/adda/Work/TDOR2/out_camera_data_iOS_720_3.yml";
  cv::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0436.m4v"; // 33
  cv::string video_path_test = "/Users/adda/Work/Eyes/src/video/IMG_0437.m4v"; // 34
  EyesHelper helper = EyesHelper(camera_path);
  cv::VideoCapture capture(video_path);
  cv::VideoCapture capture_test(video_path_test);
  std::vector<std::vector<Match>> total_matches = helper.slam_map(capture, params[1], params[3]);
  helper.slam_localize(capture_test, params[2], total_matches);
}