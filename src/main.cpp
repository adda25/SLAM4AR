#include <iostream>
#include <stdlib.h>
#include "asvs.hpp"

void (*test_ptr)(std::vector<int>);
void map(cv::Mat frame);
void test_map_system(std::vector<int> params);
void test_map_system_5_point(std::vector<int> params);
void test_map_and_pose_system(std::vector<int> params);
void test_write_and_load_map(std::vector<int> params);
void test_find_object(std::vector<int> params);
void video_iterator(void (*test_ptr)(cv::VideoCapture capture, int count));


std::string camera_path = "/Users/adda/Work/TDOR2/out_camera_data_iOS_720_3.yml";

int 
main(int argc, char** argv) 
{
  using namespace std;
  cout << "###  Test Slam  ###" << endl;
  vector<int> params;
  for (int i = 0; i < argc; ++i) { params.push_back(atoi(argv[i])); }
  //test_map_system(params);
  test_map_system_5_point(params);
  //test_map_and_pose_system(params);
  //test_write_and_load_map(params);
  //test_find_object(params);
  return 0;
}

void 
test_map_system(std::vector<int> params) 
{
  std::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0446.m4v";
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  cv::Mat frame;
  SlamAPI slam = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  slam.map_init();
  slam.min_dist = (double)params[2];
  for (int i = 0; i < params[1]; i++) {
    capture >> frame;
    if (frame.cols == 0) { break; }
    Marker m1 = marker_detector.detectMarker(frame);
    if (m1.marker.size() == 0) { continue; } 
    slam.map_update(frame, m1.marker[0].pose());
  }
  slam.map_write_point_cloud("map.txt", true);
}

void 
test_map_system_5_point(std::vector<int> params)
{
  std::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0446.m4v";
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  cv::Mat frame;
  SlamAPI slam = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  slam.map_init();
  slam.min_dist = (double)params[2];
  for (int i = 0; i < params[1]; i++) {
    capture >> frame;
    if (frame.cols == 0) { break; }
    Marker m1 = marker_detector.detectMarker(frame);
    if (m1.marker.size() == 0) { continue; }
    slam.map_update(frame, m1.marker[0].pose(), 0);
  }
}

void 
test_map_and_pose_system(std::vector<int> params) 
{
  std::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0446.m4v"; // 33
  std::string video_path_test = "/Users/adda/Work/Eyes/src/video/IMG_0446.m4v"; // 34
  cv::Mat estimated_pose;
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  cv::Mat frame;
  SlamAPI slam = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  cv::VideoCapture capture_test(video_path_test);
  slam.map_init();
  slam.min_dist = (double)params[3];
  for (int i = 0; i < params[1]; i++) {
    capture >> frame;
    printf("Progress map: %d / %d  map_points: %d \r", i, params[1], slam.map_point_size);
    if (frame.cols == 0) { break; }
    //cv::Mat nf;
    //cv::Size s(640, 480);
    //cv::resize(frame, nf,s);
    Marker m1 = marker_detector.detectMarker(frame);
    if (m1.marker.size() == 0) { continue; } 
    //std::cout << "ENTER BIG\n";
    slam.map_update(frame, m1.marker[0].pose());
    //std::cout << "EXIT BIG\n";
  }
  //slam.map_write_point_cloud("map.txt", true);
  for (int i = 0; i < params[2]; i++) {
    capture_test >> frame;
    printf("Progress loc: %d / %d \r", i, params[2]);
    if (frame.cols == 0) { break; }
    slam.localize(frame, estimated_pose);
    slam.visualize(frame, estimated_pose, true, false);
  }
}

/*
void 
test_write_and_load_map(std::vector<int> params)
{
  cv::string video_path = "/Users/adda/Work/Eyes/src/video/calc.m4v"; 
  SlamAPI helper = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  Map map = helper.slam_map(capture, params[1], params[2]);
  map__save_on_file("./maps/calc2.txt", map);
  Map loaded_map = map__load_from_file("./maps/calc2.txt");
}
*/
/*
void 
test_find_object(std::vector<int> params)
{
  cv::string video_path_test = "/Users/adda/Work/Eyes/src/video/test_bike_calc.m4v";
  SlamAPI slam = SlamAPI(camera_path);
  cv::Mat frame;
  cv::VideoCapture capture(video_path_test);
  Map object_map_bike = map__load_from_file("./maps/map_bike2.txt");
  Map object_map_calc = map__load_from_file("./maps/calc2.txt");
  std::vector<Map> objects;
  objects.push_back(object_map_bike);
  objects.push_back(object_map_calc);
  std::vector<cv::Rect> rects;
  
  for (int i = 0; i < params[1]; i++) {
    capture >> frame;
    slam.localize_object(frame, object_map_calc, rects);
  }

  //helper
}
*/
// 1
// DEVO FARE IL REFINING
// ALTRIMENTI MAPPE TROPPO GROSSE
//
// 2
// MAP-LOC

// Domani
// -> Togliere parti hardcoded
// -> Incrementare precisione, test senza punti del marker
// -> Fine tuning parametri ORB


