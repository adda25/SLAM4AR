#include "asvs.hpp"


template <typename T> void calc_distance(cv::Mat trvec1, cv::Mat trvec2, T& distance) {
    T x = trvec2.at<T>(0,0) - trvec1.at<T>(0,0);
    T y = trvec2.at<T>(1,0) - trvec1.at<T>(1,0);
    T z = trvec2.at<T>(2,0) - trvec1.at<T>(2,0);
    distance = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}


std::vector<std::vector<Match>> 
EyesHelper::slam_map(cv::VideoCapture capture, int count, float min_dist) {
  std::cout << "map_video_stream_min_dist " << min_dist << std::endl;
  cv::Mat image;
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  std::vector<std::vector<Match>> total_matches;
  uint image_ready = 0;
  Marker m1;
  Marker m2;
  cv::Mat image1;
  std::vector<Match> new_joined_matches;
  
  if(!capture.isOpened()) {
    std::cout << "--> Capture is not open" << std::endl;
  }
  for (int i = 0; i < count; i++) {
    capture >> image;
    if (image.cols == 0) { break; }
    switch (image_ready) {
      case 0: {
        image.copyTo(image1);
        m1 = marker_detector.detectMarker(image1);
        if (m1.marker.size() == 0) {
          continue;
        }
        image_ready = 1;
      }
      case 1: {
        m2 = marker_detector.detectMarker(image);
        if (m2.marker.size() == 0) {
          continue;
        }
        double distance = 0.0;
        calc_distance(m1.marker[0].trVec, m2.marker[0].trVec, distance);
        if (distance > min_dist) {
          std::vector<Match> tm = slam.map_2_images(image1, m1.marker[0].pose(), image, m2.marker[0].pose(), new_joined_matches);
          image_ready = 0;
          std::cout << "Matched points: " << tm.size() << std::endl;
          ////////
          std::vector<MapPoint> mps;
          for (auto &m : tm) {
            MapPoint new_mp = MapPoint{m.match_point_in_marker_frame[0], m.coords3d_2, m.keypoint2, m.descriptor2};
            mps.push_back(new_mp);
          }
          map__update(slam.map, mps, m2.marker[0].pose(), image);

          ////////
          total_matches.push_back(tm);
          if (tm.size() < 4) { continue; }
          std::cout << "Real pose: " << m2.marker[0].pose() << std::endl;
          cv::Mat estimated_pose = slam.estimated_pose(tm);
          std::cout << "Estimated pose: " << estimated_pose << std::endl;
          new_joined_matches = slam.join_matches(total_matches);
          //slam.draw_estimated_map(tm, image, m1.marker[0].trVec, m1.marker[0].rotVec);
          //cv::imshow("n", image);
          //cv::waitKey(5000);
        }
      }
    }
  }
  map__remove_empty_sectors(slam.map);
  //map__write("map.txt", slam.map, true);
  return total_matches;
}

void 
EyesHelper::slam_localize(cv::VideoCapture capture_test, int count_test, std::vector<std::vector<Match>> matches) {
  std::vector<Match> tot_matches;
  std::vector<cv::Point3d> real_poses;
  std::vector<cv::Point3d> estimated_poses;
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  Marker m1;
  
  tot_matches = slam.join_matches(matches);
  std::cout << "Start video test" << std::endl;
  cv::Mat image;
  for (int i = 0; i < count_test; i++) {
    capture_test >> image;
    if (image.cols == 0) { break; }
    m1 = marker_detector.detectMarker(image);
    if (m1.marker.size() == 0) {
      std::cout << "No marker detected" << std::endl;
    } 
    std::cout << "Real pose: " << m1.marker[0].trVec << std::endl;
    real_poses.push_back(cv::Point3d(m1.marker[0].trVec.at<double>(0,0), m1.marker[0].trVec.at<double>(1,0), m1.marker[0].trVec.at<double>(2,0)));
    cv::Mat estimated_pose = slam.calc_pose_with_matches(image, tot_matches);
    ///
    map__sectors_in_view(slam.map, estimated_pose);
    ///
    std::cout << "Estimated pose: " << estimated_pose << std::endl;
    estimated_poses.push_back(cv::Point3d(estimated_pose.at<double>(0,3), estimated_pose.at<double>(1,3), estimated_pose.at<double>(2,3)));
  }
  std::ofstream ofs;
  ofs.open ("poses.csv", std::ofstream::out | std::ofstream::app);
  ofs << "r_x, " << "r_y, " << "r_z, " << "e_x, " << "e_y, " << "e_z, " << "\n";
  for (int i = 0; i < real_poses.size(); i++) {
    if (estimated_poses[i].z <= 0) { 
      continue; 
    }
    ofs << real_poses[i].x << "," << real_poses[i].y << "," << real_poses[i].z << "," 
      << estimated_poses[i].x << "," << estimated_poses[i].y << "," << estimated_poses[i].z << "\n";
  }
  ofs.close();


  
}