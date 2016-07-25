#include "asvs.hpp"

template <typename T> 
void calc_distance(cv::Mat trvec1, cv::Mat trvec2, T& distance) {
    T x = trvec2.at<T>(0,0) - trvec1.at<T>(0,0);
    T y = trvec2.at<T>(1,0) - trvec1.at<T>(1,0);
    T z = trvec2.at<T>(2,0) - trvec1.at<T>(2,0);
    distance = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}

void 
SlamAPI::slam_sys_init(std::string camera_path)
{
  slam__init(slam_sys, camera_path);
}

void 
SlamAPI::map_init()
{
  map = Map();
  uint a[3] = {5, 5, 1};
  uint b[3] = {200, 200, 50};
  map = map__create(a, b);
  map_point_size = 0;
}
/*
void 
SlamAPI::map_update(cv::Mat &frame, cv::Mat pose, int kk)
{
  static int state = 0;
  static uint frame_counter = 0;  
  static cv::Mat image_1, pose_1;
  if (state == 0) {
    pose.copyTo(pose_1);
    frame.copyTo(image_1);
    state = 1;
    return;
  } else if (frame_counter == 100) {
    frame_counter = 0;
    std::vector<MapPoint> kf_map = slam__map(slam_sys, image_1, frame);
    map__update(map, kf_map, pose, frame);
    //std::cout << "Real diff: " << pose_1 * pose.inv() << std::endl;
    state = 0;
  }
  frame_counter++;
}*/

void 
SlamAPI::map_update(cv::Mat &frame, cv::Mat pose)
{
  static cv::Mat image_1;
  static cv::Mat pose_1;
  static int state = 0;
  if (state == 0) {
    frame.copyTo(image_1);
    pose.copyTo(pose_1);
    state = 1;
    return;
  } 
  double distance = 0.0;
  calc_distance(tr_vec_from_pose(pose_1), tr_vec_from_pose(pose), distance);
  if (distance < min_dist) { return; }
  std::vector<MapPoint> kf_map = slam__map(slam_sys, 
                                           image_1, 
                                           frame, 
                                           pose_1, 
                                           pose);
  map_point_size += kf_map.size();
  map__update(map, kf_map, pose, frame);
  state = 0;
}

void 
SlamAPI::map_update_stereo(cv::Mat &frame_1, cv::Mat &frame_2)
{
// TODO
}

void 
SlamAPI::map_remove_old_sectors()
{
// TODO
}

void 
SlamAPI::map_reset()
{
  map_init();
}

void 
SlamAPI::map_save(std::string filename)
{
  map__save_on_file(filename, map);
}

void 
SlamAPI::map_save(std::string filename, const Map &map_to_save)
{
  map__save_on_file(filename, map_to_save);
}

void 
SlamAPI::map_load(std::string filename)
{
  map = map__load_from_file(filename);
}

void
SlamAPI::map_load(std::string filename, Map &map_to_load)
{
  map_to_load = map__load_from_file(filename);
}

void 
SlamAPI::localize(cv::Mat &frame, cv::Mat &estimated_pose)
{
  estimated_pose = slam__localize(slam_sys, map, frame);
}

void 
SlamAPI::localize_and_update(cv::Mat &frame, cv::Mat &estimated_pose)
{
  static cv::Mat last_pose;
  static cv::Mat last_frame;
  static int counter = 0;
  localize(frame, estimated_pose);
  //estimated_pose = slam__localize(slam_sys, map, frame); 
  if (counter == 0) {
    frame.copyTo(last_frame);
    estimated_pose.copyTo(last_pose);
  } 
  if (counter == 30) {
    std::vector<MapPoint> kf_map = slam__map(slam_sys, 
                                             last_frame, 
                                             frame, 
                                             last_pose, 
                                             estimated_pose);
    map_point_size += kf_map.size();
    map__update(map, kf_map, estimated_pose, frame);
    counter = 0;
    return;
  }
  counter++;
}

void 
SlamAPI::localize_object(cv::Mat &frame, const Map &object_map, std::vector<cv::Rect> &objects_rects)
{
  std::vector<Map> objects_maps;
  objects_maps.push_back(map);
  objects_rects = slam__find_objects(slam_sys, objects_maps, frame);
  for (auto &r : objects_rects) {
    cv::rectangle(frame, r, cv::Scalar(0,0,255,255));
  }
  cv::imshow("find_objects", frame);
  cv::waitKey(10);
}

void 
SlamAPI::map_write_point_cloud(std::string filename, bool grid)
{
  map__write(filename, map, grid);
}

void 
SlamAPI::visualize(cv::Mat &frame, cv::Mat &pose, bool draw_map, bool draw_sr)
{
  if (draw_map) {
    map__draw(frame, map, tr_vec_from_pose(pose), rot_mat_from_pose(pose), slam_sys.camera.cameraMatrix, slam_sys.camera.distorsion);
  }
  if (draw_sr) {
    //draw_cube_on_ref_sys(frame, slam_sys.camera.getCameraMatrix(), slam_sys.camera.getDistorsion(), estimated_camera_pose, 120, cv::Scalar(255, 0, 0, 255));    
  }
  cv::imshow("n", frame);
  cv::waitKey(5);
}







/*
Map
SlamAPI::slam_map(cv::VideoCapture capture, int count, float min_dist) 
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
  uint b[3] = {200, 200, 50};
  int total_map_points_counter = 0;
  printf("-> Start mapping...\n");

  slam_map = map__create(a, b);
  if(!capture.isOpened()) { std::cout << "--> Capture is not open" << std::endl; }
  slam__init(slam_sys, camera_path);
  for (int i = 0; i < count; i++) {
    printf("-> Progress: %d / %d  sectors_founds: %lu total_points: %d \r", i, count, slam_map.size(), total_map_points_counter);
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
          map__update(slam_map, mps, m2.marker[0].pose(), image);
          total_map_points_counter += mps.size();
          if (mps.size() < 4) { continue; } 
          //cv::Mat estimated_pose = slam__estimated_pose(mps, slam_sys.camera);
        }
      }
    }
  }
  map__remove_empty_sectors(slam_map, 3);
  return slam_map;
}

void 
SlamAPI::slam_localize(cv::VideoCapture capture_test, int count_test, Map map) 
{
  Map tot_matches;
  cv::Mat estimated_camera_pose;
  std::vector<cv::Point3d> real_poses;
  std::vector<cv::Point3d> estimated_poses;
  MyMarkerDetector marker_detector = MyMarkerDetector(120.0, camera_path);
  Marker m1;
  SlamSystem slam_sys = SlamSystem();
  slam__init(slam_sys, camera_path);
  printf("\n-> Start localize...\n");
  cv::Mat image;
  for (int i = 0; i < count_test; i++) {
    capture_test >> image;
    if (image.cols == 0) { break; }
    m1 = marker_detector.detectMarker(image);
    if (m1.marker.size() == 0) { std::cout << "No marker detected" << std::endl; } 
    //real_poses.push_back(cv::Point3d(m1.marker[0].trVec.at<double>(0,0), m1.marker[0].trVec.at<double>(1,0), m1.marker[0].trVec.at<double>(2,0)));
    if (estimated_camera_pose.rows != 0) {
      Map p_map = map__sectors_in_view(map, estimated_camera_pose);
      estimated_camera_pose = slam__localize(slam_sys, p_map, image);
    } else {
      estimated_camera_pose = slam__localize(slam_sys, map, image);
    }
    std::cout << "--> Diff pose: " << m1.marker[0].trVec.at<double>(0,0) - estimated_camera_pose.at<double>(0,3) 
              << " " << m1.marker[0].trVec.at<double>(1,0) - estimated_camera_pose.at<double>(1,3) 
              << " " << m1.marker[0].trVec.at<double>(2,0) - estimated_camera_pose.at<double>(2,3) << " \n" << std::flush;
    estimated_poses.push_back(cv::Point3d(estimated_camera_pose.at<double>(0,3), estimated_camera_pose.at<double>(1,3), estimated_camera_pose.at<double>(2,3)));
    // Draw cube on ref sys
    #ifdef IMG_DEBUG
    draw_cube_on_ref_sys(image, slam_sys.camera.getCameraMatrix(), slam_sys.camera.getDistorsion(), estimated_camera_pose, 120, cv::Scalar(255, 0, 0, 255));
    //draw_cube_on_ref_sys(image, slam_sys.camera.getCameraMatrix(), slam_sys.camera.getDistorsion(), m1.marker[0].pose(), 120, cv::Scalar(0, 0, 255, 255));
    cv::imshow("n", image);
    cv::waitKey(5);
    #endif
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

void 
SlamAPI::slam_find_objects(cv::VideoCapture capture, int count_test, std::vector<Map> objects_maps)
{
  cv::Mat image;
  SlamSystem slam_sys = SlamSystem();
  slam__init(slam_sys, camera_path);
  
  for (int i = 0; i < count_test; i++) {
    capture >> image;
    if (image.cols == 0) { break; }
    std::vector<cv::Rect> rects;
    rects = slam__find_objects(slam_sys, objects_maps, image);
    for (auto &r : rects) {
      cv::rectangle(image, r, cv::Scalar(0,0,255,255));
    }
    cv::imshow("find_objects", image);
    cv::waitKey(10);
  } 
}
*/
/*
void 
SlamAPI::map_update(cv::Mat &frame, cv::Mat pose)
{
  static cv::Mat image_1;
  static cv::Mat pose_1;
  static std::future<void> future_thread;
  static bool future_launched = false;
  static int state = 0;
  if (state == 0) {
    frame.copyTo(image_1);
    pose.copyTo(pose_1);
    state = 1;
    return;
  } 
  double distance = 0.0;
  calc_distance(tr_vec_from_pose(pose_1), tr_vec_from_pose(pose), distance);
  if (distance < min_dist) { return; }
  if (future_launched) {
    future_thread.get();
    future_launched = false;
  }
  //std::cout << "LAUNCH THREAD\n";
  future_thread = std::async(std::launch::async, slam_map_update_w_thread, slam_sys, image_1, frame, pose_1, pose, std::ref(map), std::ref(map_point_size));
  future_launched = true;
  //std::cout << "AFTER THREAD\n";
  
  std::vector<MapPoint> kf_map = slam__map(slam_sys, 
                                           image_1, 
                                           frame, 
                                           pose_1, 
                                           pose);
  map_point_size += kf_map.size();
  map__update(map, kf_map, pose, frame);
  //r.get();
  state = 0;
}
*/








