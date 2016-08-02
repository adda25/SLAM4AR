#include <iostream>
#include <stdlib.h>
#include "asvs.hpp"

void (*test_ptr)(std::vector<int>);
void map(cv::Mat frame);
void test_map_system(std::vector<int> params);
void test_map_localize_update(std::vector<int> params);
void test_map_system_5_point(std::vector<int> params);
void test_map_and_pose_system(std::vector<int> params);
void test_write_and_load_map(std::vector<int> params);
void test_find_object(std::vector<int> params);
void video_iterator(void (*test_ptr)(cv::VideoCapture capture, int count));

void save_video_frames(std::vector<int> params);

// sudo cp /usr/local/Cellar/opencv3/3.1.0_3/lib/pkgconfig/opencv.pc /usr/local/lib/pkgconfig/
std::string camera_path = "/Users/adda/Work/TDOR2/out_camera_data_iOS_720_3.yml";

int 
main(int argc, char** argv) 
{
  std::cout << "===========  Test Slam  ===========" << std::endl;
  #if CV_MAJOR_VERSION == 2
  std::cout << "Opencv 2" << std::endl;
  #elif CV_MAJOR_VERSION == 3
  std::cout << "Opencv 3" << std::endl;
  #endif


  std::vector<int> params;
  for (int i = 0; i < argc; ++i) { params.push_back(atoi(argv[i])); }
  //test_map_system(params);
  //save_video_frames(params);
  //test_map_system_5_point(params);
  //test_map_localize_update(params);
  test_map_and_pose_system(params);
  //test_write_and_load_map(params);
  //test_find_object(params);
  return 0;
}

void 
save_video_frames(std::vector<int> params)
{
  std::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0446.m4v";
  cv::Mat frame;
  cv::VideoCapture capture(video_path);
  
  int k = params[1]; 
  for (int i = params[1]; i < params[2]; i++) {
    capture >> frame;
    if (frame.cols == 0) { break; }
    if (i != k) { continue; }
    std::string base = "images/image";
    base += std::to_string(i) + ".jpg";
    cv::imwrite(base, frame);
    k += params[3];
  }
}

void 
test_map_system(std::vector<int> params) 
{
  std::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0446.m4v";
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  cv::Mat frame;
  SlamAPI slam = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  slam.min_dist = (double)params[2];
  for (int i = 0; i < params[1]; i++) {
    capture >> frame;
    if (frame.cols == 0) { break; }
    Marker m1 = marker_detector.detectMarker(frame);
    if (m1.marker.size() == 0) { continue; } 
    slam.map_update(frame, m1.marker[0].pose());
  }
  slam.map_write_point_cloud("map_new.txt", false);
}

/* Test map init and localization
   with map update. 
   -> params[1] = init map lenght
   -> params[2] = update map lenght
   -> params[3] = distance between camera frames */
void 
test_map_localize_update(std::vector<int> params)
{
  std::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0434.m4v";
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  cv::Mat frame;
  cv::Mat estimated_pose;
  SlamAPI slam = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  slam.min_dist = (double)params[3];

  // Initialize the map with the marker
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now(); 
  for (int i = 0; i < params[1]; i++) {
    capture >> frame;
    if (frame.cols == 0) { break; }
    Marker m1 = marker_detector.detectMarker(frame);
    if (m1.marker.size() == 0) { continue; } 
    slam.map_update(frame, m1.marker[0].pose());
  }
  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1);
  std::cout << "MAP TIME: " << " " << time_span.count() << " seconds." << std::endl; 
  slam.map_write_point_cloud("initial_map.txt", true);
  slam.adjust_map();
  // Localize and update the map
  std::vector<cv::Mat> estimated_poses;
  for (int i = params[1]; i < params[2]; i++) {
    capture >> frame;
    if (frame.cols == 0) { break; }
    slam.localize_and_update(frame, estimated_pose);
    estimated_poses.push_back(estimated_pose);
    slam.visualize(frame, estimated_pose, true, true);
  }
  slam.map_write_point_cloud("final_map.txt", true);
  std::ofstream ofs;
  ofs.open ("poses.csv", std::ofstream::trunc);
  ofs << "e_x, " << "e_y, " << "e_z, " << "\n";
  for (int i = 0; i < estimated_poses.size(); i++) {
    ofs << estimated_poses[i].at<double>(0,3) << "," << estimated_poses[i].at<double>(1,3)  << "," << estimated_poses[i].at<double>(2,3)  << "\n";
  }
  ofs.close();
}
/*
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
  slam.map_write_point_cloud("map.txt", true);
}*/

void 
test_map_and_pose_system(std::vector<int> params) 
{
  std::string video_path = "/Users/adda/Work/Eyes/src/video/IMG_0433.m4v"; // 33
  std::string video_path_test = "/Users/adda/Work/Eyes/src/video/IMG_0434.m4v"; // 34
  cv::Mat estimated_pose;
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  cv::Mat frame;
  SlamAPI slam = SlamAPI(camera_path);
  cv::VideoCapture capture(video_path);
  cv::VideoCapture capture_test(video_path_test);
  slam.min_dist = (double)params[3];
  for (int i = 0; i < params[1]; i++) {
    capture >> frame;
    printf("Progress map: %d / %d  map_points: %d \r", i, params[1], slam.map_point_size);
    if (frame.cols == 0) { break; }
    Marker m1 = marker_detector.detectMarker(frame);
    if (m1.marker.size() == 0) { continue; } 
    slam.map_update(frame, m1.marker[0].pose());
  }
  slam.adjust_map();
  slam.map_write_point_cloud("map.txt", false);
  std::vector<cv::Mat> poses;
  for (int i = 0; i < params[2]; i++) {
    capture_test >> frame;
    printf("Progress loc: %d / %d \r", i, params[2]);
    if (frame.cols == 0) { break; }
    slam.localize(frame, estimated_pose);
    poses.push_back(estimated_pose);
    slam.visualize(frame, estimated_pose, true, true);
  }
  std::ofstream ofs;
  ofs.open ("poses.csv", std::ofstream::trunc);
  ofs << "e_x, " << "e_y, " << "e_z, " << "\n";
  for (int i = 0; i < poses.size(); i++) {
    ofs << poses[i].at<double>(0,3) << "," << poses[i].at<double>(1,3)  << "," << poses[i].at<double>(2,3)  << "\n";
  }
  ofs.close();

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


