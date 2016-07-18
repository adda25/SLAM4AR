#include "asvs.hpp"


template <typename T> void calc_distance(cv::Mat trvec1, cv::Mat trvec2, T& distance) {
    T x = trvec2.at<T>(0,0) - trvec1.at<T>(0,0);
    T y = trvec2.at<T>(1,0) - trvec1.at<T>(1,0);
    T z = trvec2.at<T>(2,0) - trvec1.at<T>(2,0);
    distance = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}

Map
SlamAPI::slam_c_map(cv::VideoCapture capture, int count, float min_dist) 
{
  cv::Mat image;
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  uint image_ready = 0;
  Marker m1;
  Marker m2;
  cv::Mat image1;
  SlamSystem slam_sys = SlamSystem();
  Map slam_map = Map();
  uint a[3] = {5, 5, 1};
  uint b[3] = {50, 50, 50};

  slam_map = map__create(a, b);
  if(!capture.isOpened()) { std::cout << "--> Capture is not open" << std::endl; }
  slam__init(slam_sys, camera_path);
  for (int i = 0; i < count; i++) {
    capture >> image;
    if (image.cols == 0) { break; }
    switch (image_ready) {
      case 0: {
        image.copyTo(image1);
        m1 = marker_detector.detectMarker(image1);
        if (m1.marker.size() == 0) { continue; }
        image_ready = 1;
      }
      case 1: {
        m2 = marker_detector.detectMarker(image);
        if (m2.marker.size() == 0) { continue; }
        double distance = 0.0;
        calc_distance(m1.marker[0].trVec, m2.marker[0].trVec, distance);
        if (distance > min_dist) {
          std::vector<MapPoint> mps = slam__map(slam_sys, image1, image, m1.marker[0].pose(), m2.marker[0].pose());
          image_ready = 0;
          std::cout << "Map points: " << mps.size() << std::endl;
          map__update(slam_map, mps, m2.marker[0].pose(), image);
          if (mps.size() < 4) { continue; }
          std::cout << "Real pose: " << m2.marker[0].pose() << std::endl;
           
          cv::Mat estimated_pose = slam__estimated_pose(mps, slam_sys.camera);
          std::cout << "Estimated pose: " << estimated_pose << std::endl;
        }
      }
    }
  }
  map__remove_empty_sectors(slam_map, 3);
  return slam_map;
}

void 
SlamAPI::slam_c_localize(cv::VideoCapture capture_test, int count_test, Map map) 
{
  Map tot_matches;
  cv::Mat estimated_camera_pose;
  std::vector<cv::Point3d> real_poses;
  std::vector<cv::Point3d> estimated_poses;
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  Marker m1;
  SlamSystem slam_sys = SlamSystem();
  slam__init(slam_sys, camera_path);
  
  std::cout << "Start video test" << std::endl;
  cv::Mat image;
  for (int i = 0; i < count_test; i++) {
    capture_test >> image;
    if (image.cols == 0) { break; }
    m1 = marker_detector.detectMarker(image);
    if (m1.marker.size() == 0) { std::cout << "No marker detected" << std::endl; } 
    std::cout << "Real pose: " << m1.marker[0].trVec << std::endl;
    real_poses.push_back(cv::Point3d(m1.marker[0].trVec.at<double>(0,0), m1.marker[0].trVec.at<double>(1,0), m1.marker[0].trVec.at<double>(2,0)));
    if (estimated_camera_pose.rows != 0) {
      Map p_map = map__sectors_in_view(map, estimated_camera_pose);
      estimated_camera_pose = slam__localize(slam_sys, p_map, image);
    } else {
      estimated_camera_pose = slam__localize(slam_sys, map, image);
    }
    std::cout << "Estimated pose: " << estimated_camera_pose << std::endl;
    estimated_poses.push_back(cv::Point3d(estimated_camera_pose.at<double>(0,3), estimated_camera_pose.at<double>(1,3), estimated_camera_pose.at<double>(2,3)));
  }
  std::ofstream ofs;
  ofs.open ("poses.csv", std::ofstream::out | std::ofstream::app);
  ofs << "r_x, " << "r_y, " << "r_z, " << "e_x, " << "e_y, " << "e_z, " << "\n";
  for (int i = 0; i < real_poses.size(); i++) {
    if (estimated_poses[i].z <= 0 || fabs(estimated_poses[i].z - real_poses[i].z) > 20) { 
      continue; 
    }
    ofs << real_poses[i].x << "," << real_poses[i].y << "," << real_poses[i].z << "," 
      << estimated_poses[i].x << "," << estimated_poses[i].y << "," << estimated_poses[i].z << "\n";
  }
  ofs.close();
}
