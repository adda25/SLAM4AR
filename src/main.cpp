#include <iostream>
#include <stdlib.h>
#include "asvs.hpp"

void test_map_system(std::vector<int> params);
void test_map_and_pose_system(std::vector<int> params);
void test_write_and_load_map(std::vector<int> params);
void test_find_object(std::vector<int> params);

int 
main(int argc, char** argv) 
{
  using namespace std;
  cout << "###  Test Slam  ###" << endl;
  vector<int> params;
  for (int i = 0; i < argc; ++i) { params.push_back(atoi(argv[i])); }
  //test_map_system(params);
  test_map_and_pose_system(params);
  //test_write_and_load_map(params);
  //test_find_object(params);
  return 0;
}

void 
test_map_system(std::vector<int> params) 
{
  std::string camera_path = "/Users/adda/Work/TDOR2/out_camera_data_iOS_720_3.yml";
  cv::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0446.m4v";
  SlamAPI helper = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  Map map = helper.slam_map(capture, params[1], params[2]);
  map__write("map.txt", map, false);
}

void 
test_map_and_pose_system(std::vector<int> params) 
{
  std::string camera_path = "/Users/adda/Work/TDOR2/out_camera_data_iOS_720_3.yml";
  cv::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0446.m4v"; // 33
  cv::string video_path_test = "/Users/adda/Work/Eyes/src/video/calc.m4v"; // 34
  SlamAPI helper = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  cv::VideoCapture capture_test(video_path_test);
  //Map map = helper.slam_map(capture, params[1], params[3]);
  Map map = map__load_from_file("./maps/map_calc.txt");
  //map__write("map.txt", map, true);
  helper.slam_localize(capture_test, params[1], map);
}

void 
test_write_and_load_map(std::vector<int> params)
{
  std::string camera_path = "/Users/adda/Work/TDOR2/out_camera_data_iOS_720_3.yml";
  cv::string video_path = "/Users/adda/Work/Eyes/src/video/calc.m4v"; 
  SlamAPI helper = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  Map map = helper.slam_map(capture, params[1], params[2]);
  map__save_on_file("./maps/calc2.txt", map);
  Map loaded_map = map__load_from_file("./maps/calc2.txt");
}

void 
test_find_object(std::vector<int> params)
{
  std::string camera_path = "/Users/adda/Work/TDOR2/out_camera_data_iOS_720_3.yml";
  cv::string video_path_test = "/Users/adda/Work/Eyes/src/video/test_bike_calc.m4v";
  SlamAPI helper = SlamAPI(camera_path);
  cv::VideoCapture capture_test(video_path_test);
  Map object_map_bike = map__load_from_file("./maps/map_bike2.txt");
  Map object_map_calc = map__load_from_file("./maps/map_calc.txt");
  std::vector<Map> objects;
  objects.push_back(object_map_bike);
  objects.push_back(object_map_calc);
  helper.slam_find_objects(capture_test, params[1], objects);
}

// 1
// DEVO FARE IL REFINING
// ALTRIMENTI MAPPE TROPPO GROSSE
//
// 2
// MAP-LOC

// Oggi
// -> Disegno feature attive / non attive
// -> Marker 3d
// -> Riconoscimento oggetti
