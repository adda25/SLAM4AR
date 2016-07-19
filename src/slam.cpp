#include "slam.hpp"

struct SlamSystemPerformanceParams
{
  double triangulation_relative_error = 0.002;
  int brief_patch_size = 32;
} slam_sys_pfr_pr;

std::chrono::high_resolution_clock::time_point
get_chrono() { return std::chrono::high_resolution_clock::now(); }


void 
measure_chrono(std::chrono::high_resolution_clock::time_point start_time, 
               std::string mex) 
{
  std::chrono::high_resolution_clock::time_point stop_chrono = get_chrono();
  std::chrono::duration<double> time_span = std::chrono:: duration_cast<std::chrono::duration<double> >(stop_chrono - start_time);
  std::cout << mex << time_span.count() << " seconds." << std::endl;
}

// 1080p
/*
cv::Mat 
cam_point_from_pixel(cv::Point2f point, double z) {
  cv::Mat result(4, 1, CV_64FC1);
  result.at<double>(0,0) = (point.x - 960) * z / 1755;
  result.at<double>(1,0) = (point.y - 540) * z / 1755;
  result.at<double>(2,0) = z;
  result.at<double>(3,0) = 1;
  return result;
}*/
// 720p

cv::Mat rot_tr_mat(cv::Mat rot, cv::Mat tr) ;
cv::Mat cam_point_from_pixel(cv::Point2f point, double z);
std::vector<cv::KeyPoint> _slam__search_features(const SlamSystem &slam_sys, cv::Mat image);
cv::Mat _slam__extract_descriptors(const SlamSystem &slam_sys, cv::Mat image, std::vector<cv::KeyPoint> keypoint);
std::vector<cv::DMatch> _slam__match_features(cv::Mat descriptors_1, cv::Mat descriptors_2);
bool _slam__are_oblique_keypoints(cv::KeyPoint kp_1, cv::KeyPoint kp_2);
void _slam__split_matches(std::vector<cv::DMatch> matches, 
                          std::vector<cv::KeyPoint> keypoints1, 
                          cv::Mat descriptors1, 
                          std::vector<cv::KeyPoint> keypoints2, 
                          cv::Mat descriptors2, 
                          std::vector<MapPoint> &matched_points1, 
                          std::vector<MapPoint> &matched_points2,
                          bool remove_oblique = false);
std::vector<MapPoint> _slam__triangulate(cv::Mat image_1, 
                                         std::vector<MapPoint> matched_points1, 
                                         cv::Mat pose1, 
                                         std::vector<MapPoint> matched_points2, 
                                         cv::Mat pose2);
std::vector<double> _slam__solve_linear_system(cv::Point3f p11, cv::Point3f p12, cv::Point3f p21, cv::Point3f p22); 
cv::Rect _slam__detect_screen_region_for_map(const SlamSystem &slam_sys, const Map &obj_map, cv::Mat &image);
cv::Point3f match_position_in_marker_frame(cv::Mat camera_pose, cv::Point3d p_solution);
cv::Mat _slam__estimated_pose(std::vector<MapPoint> matches); 
cv::Mat _slam__estimated_pose(cv::vector<cv::Point2f> img_points_vector, cv::vector<cv::Point3f> obj_points_vector); 
std::vector<MapMatchFt_C> _slam__find_p2p_correspondence(const Map &map, const std::vector<MapPoint> &mp1, const std::vector<MapPoint> &mp2);
void  _slam__good_data_for_solvePnp(std::vector<MapMatchFt_C> &mapMatchFt,
                                    cv::vector<cv::Point2f> &img_points_vector,
                                    cv::vector<cv::Point3f> &obj_points_vector);
void debug_loc(cv::Mat image, const Map &map, std::vector<cv::Point2f> current_im_points);
void debug_pair(cv::Mat im1, cv::Mat im2, std::vector<MapPoint> matched_points1, std::vector<MapPoint> matched_points2);

//////
cv::Mat 
cam_point_from_pixel(cv::Point2f point, double z) 
{
  cv::Mat result(4, 1, CV_64FC1);
  result.at<double>(0,0) = (point.x - 640) * z / 1152;
  result.at<double>(1,0) = (point.y - 360) * z / 1152;
  result.at<double>(2,0) = z;
  result.at<double>(3,0) = 1;
  return result;
}

cv::Mat 
rot_tr_mat(cv::Mat rot, cv::Mat tr) 
{
    cv::Mat result(4, 4, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) { 
          result.at<double>(i,k) = rot.at<double>(i,k); 
        }
    }
    result.at<double>(0,3) = tr.at<double>(0,0);
    result.at<double>(1,3) = tr.at<double>(1,0);
    result.at<double>(2,3) = tr.at<double>(2,0);
    result.at<double>(3,0) = 0;
    result.at<double>(3,1) = 0;
    result.at<double>(3,2) = 0;
    result.at<double>(3,3) = 1;
    return result;
}
////


void 
slam__init(SlamSystem &slam_sys, const std::string camera_path) 
{
  slam_sys.features_detector = new cv::OrbFeatureDetector(4000, 
                                                          1.12f, 
                                                          8, 
                                                          31, 
                                                          0, 
                                                          2, 
                                                          cv::ORB::HARRIS_SCORE, 
                                                          31);
  slam_sys.descriptions_extractor = new cv::OrbDescriptorExtractor();
  slam_sys.camera = MyCamereParamReader();
  slam_sys.camera.readFromXMLFile(camera_path);
}

std::vector<MapPoint> 
slam__map(const SlamSystem &slam_sys, 
          cv::Mat &image_1, 
          cv::Mat &image_2, 
          const cv::Mat &pose_1, 
          const cv::Mat &pose_2)
{
  std::vector<MapPoint> matched_points1;
  std::vector<MapPoint> matched_points2;
  std::vector<cv::KeyPoint> keypoints1 = _slam__search_features(slam_sys, image_1);
  std::vector<cv::KeyPoint> keypoints2 = _slam__search_features(slam_sys, image_2);
  cv::Mat descriptors1 = _slam__extract_descriptors(slam_sys, image_1, keypoints1);
  cv::Mat descriptors2 = _slam__extract_descriptors(slam_sys, image_2, keypoints2);
  std::vector<cv::DMatch> matches = _slam__match_features(descriptors1, descriptors2);
  _slam__split_matches(matches, keypoints1, descriptors1, keypoints2, descriptors2, matched_points1, matched_points2, true);
  //debug_pair(image_1, image_2, matched_points1, matched_points2);
  std::vector<MapPoint> total_matches = _slam__triangulate(image_1, matched_points1, pose_1, matched_points2, pose_2);  
  return total_matches;
}

cv::Mat 
slam__localize(const SlamSystem &slam_sys, const Map &map, cv::Mat &image)
{
  std::vector<cv::KeyPoint> old_keypoints;
  std::vector<MapPoint> matched_points1;
  std::vector<MapPoint> matched_points2;
  cv::vector<cv::Point2f> img_points_vector;
  cv::vector<cv::Point3f> obj_points_vector;
  cv::Mat pose;
  cv::Mat old_descriptors = cv::Mat(0, slam_sys_pfr_pr.brief_patch_size, CV_8U);
  std::vector<cv::DMatch> new_matches;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<MapMatchFt_C> mapMatchFt;
  keypoints = _slam__search_features(slam_sys, image);
  descriptors = _slam__extract_descriptors(slam_sys, image, keypoints);
  map__merge_keypoints_and_descriptors(map, old_keypoints, old_descriptors);
  new_matches = _slam__match_features(old_descriptors, descriptors);
  _slam__split_matches(new_matches, old_keypoints, old_descriptors, keypoints, descriptors, matched_points1, matched_points2);
  mapMatchFt = _slam__find_p2p_correspondence(map, matched_points1, matched_points2);
  _slam__good_data_for_solvePnp(mapMatchFt, img_points_vector, obj_points_vector);
  if (img_points_vector.size() < 7) { return cv::Mat(4, 4, CV_64F, double(0)); }
  pose = slam__estimated_pose(img_points_vector, obj_points_vector, slam_sys.camera);
  debug_loc(image, map, img_points_vector);
  return pose;
}

std::vector<cv::Rect> 
slam__find_objects(const SlamSystem &slam_sys, 
                   std::vector<Map> objects_maps, 
                   cv::Mat &image)
{
  std::vector<cv::Rect> obj_windows;
  for (auto &m : objects_maps) {
    obj_windows.push_back(_slam__detect_screen_region_for_map(slam_sys, m, image));
  }
  return obj_windows;
}

cv::Mat 
slam_localize_and_update(const SlamSystem &slam_sys, Map &map, cv::Mat &image) 
{
  cv::Mat pose;
  return pose; 
}

void 
_slam__good_data_for_solvePnp(std::vector<MapMatchFt_C> &mapMatchFt,
                              cv::vector<cv::Point2f>   &img_points_vector,
                              cv::vector<cv::Point3f>   &obj_points_vector)
{
  int i = 0; 
  std::sort(mapMatchFt.begin(), mapMatchFt.end());
  while (true) {
    if (i == mapMatchFt.size()) { break; }
    if ((mapMatchFt[i].accuracy_index > 20)) { break; }
    img_points_vector.push_back(mapMatchFt[i].new_found_ft_in_new_image.keypoint.pt);
    obj_points_vector.push_back(mapMatchFt[i].ref_match.coords_3D); 
    i++;
  }
}

std::vector<cv::KeyPoint> 
_slam__search_features(const SlamSystem &slam_sys, cv::Mat image)
{
  std::vector<cv::KeyPoint> keypoints;
  slam_sys.features_detector->detect(image, keypoints);
  return keypoints;
}

cv::Mat 
_slam__extract_descriptors(const SlamSystem &slam_sys, 
                           cv::Mat image, 
                           std::vector<cv::KeyPoint> keypoint)
{
  cv::Mat descriptors;
  slam_sys.descriptions_extractor->compute(image, keypoint, descriptors);
  return descriptors;
}

std::vector<cv::DMatch> 
_slam__match_features(cv::Mat descriptors_1, cv::Mat descriptors_2) 
{
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> tot_matches;
  size_t t_size = 0;
  cv::BFMatcher matcher = cv::BFMatcher(cv::NORM_HAMMING2, true);
  matcher.knnMatch(descriptors_1, descriptors_2, matches, 1);
  for (auto &m : matches) { t_size += m.size(); }
  tot_matches.reserve(t_size);
  for (auto &m : matches) { tot_matches.insert(tot_matches.end(), m.begin(), m.end()); }
  return tot_matches;  
}

bool
_slam__are_oblique_keypoints(cv::KeyPoint kp_1, cv::KeyPoint kp_2)
{
  return !(fabs(kp_2.pt.y - kp_1.pt.y) < 50) || !(fabs(kp_2.pt.x - kp_1.pt.x) < 50);
}

void 
_slam__split_matches(std::vector<cv::DMatch> matches, 
                     std::vector<cv::KeyPoint> keypoints1, 
                     cv::Mat descriptors1, 
                     std::vector<cv::KeyPoint> keypoints2, 
                     cv::Mat descriptors2, 
                     std::vector<MapPoint> &matched_points1, 
                     std::vector<MapPoint> &matched_points2,
                     bool remove_oblique)
{
  for (auto &gm : matches) {
    MapPoint p1, p2;
    int idx1 = gm.queryIdx;
    int idx2 = gm.trainIdx;
    if (remove_oblique) {
      if (_slam__are_oblique_keypoints(keypoints1[idx1], keypoints2[idx2])) { continue; }
    }
    p1.keypoint = keypoints1[idx1];
    p2.keypoint = keypoints2[idx2];
    p1.descriptor = descriptors1.row(idx1);
    p2.descriptor = descriptors2.row(idx2);
    matched_points1.push_back(p1);
    matched_points2.push_back(p2);
  }
}

std::vector<MapPoint>     
_slam__triangulate(cv::Mat image_1, 
                   std::vector<MapPoint> matched_points1, 
                   cv::Mat pose1, 
                   std::vector<MapPoint> matched_points2, 
                   cv::Mat pose2)
{
  std::vector<MapPoint> total_matches;
  cv::Mat from1to2 = pose1 * pose2.inv();
  for (int gm = 0; gm < matched_points1.size(); gm++) {
    cv::Mat cam_points_11 = cam_point_from_pixel(matched_points1[gm].keypoint.pt, 100);
    cv::Mat cam_points_12 = cam_point_from_pixel(matched_points1[gm].keypoint.pt, 20000);
    cv::Mat cam_points_21 = cam_point_from_pixel(matched_points2[gm].keypoint.pt, 100);
    cv::Mat cam_points_22 = cam_point_from_pixel(matched_points2[gm].keypoint.pt, 20000);
    cam_points_21   = from1to2 * cam_points_21;
    cam_points_22   = from1to2 * cam_points_22;
    cv::Point3f p11 = cv::Point3f(cam_points_11.at<double>(0,0), cam_points_11.at<double>(1,0), cam_points_11.at<double>(2,0));
    cv::Point3f p12 = cv::Point3f(cam_points_12.at<double>(0,0), cam_points_12.at<double>(1,0), cam_points_12.at<double>(2,0));
    cv::Point3f p21 = cv::Point3f(cam_points_21.at<double>(0,0), cam_points_21.at<double>(1,0), cam_points_21.at<double>(2,0));
    cv::Point3f p22 = cv::Point3f(cam_points_22.at<double>(0,0), cam_points_22.at<double>(1,0), cam_points_22.at<double>(2,0));        
    std::vector<double> solution = _slam__solve_linear_system(p11, p12, p21, p22);
    if (solution[0] < -9999998) { continue; } 
    matched_points2[gm].coords_3D = match_position_in_marker_frame(pose1.inv(), cv::Point3d(solution[3], solution[4], solution[5]));
    matched_points2[gm].hits++;
    //matched_points2[gm].pixel_colors()
    total_matches.push_back(matched_points2[gm]);
  }
  return total_matches;
}

/*  A x = B  */
std::vector<double> 
_slam__solve_linear_system(cv::Point3f p11, cv::Point3f p12, cv::Point3f p21, cv::Point3f p22) 
{
    cv::Mat A = cv::Mat(3, 2, CV_64F);
    cv::Mat B = cv::Mat(3, 1, CV_64F);
    cv::Mat X;
    A.at<double>(0,0) = p12.x - p11.x; A.at<double>(0,1) = -p22.x + p21.x;
    A.at<double>(1,0) = p12.y - p11.y; A.at<double>(1,1) = -p22.y + p21.y;
    A.at<double>(2,0) = p12.z - p11.z; A.at<double>(2,1) = -p22.z + p21.z;
    B.at<double>(0,0) = p21.x - p11.x;
    B.at<double>(1,0) = p21.y - p11.y;
    B.at<double>(2,0) = p21.z - p11.z;
    Eigen::MatrixXd MA(3, 2);
    Eigen::MatrixXd MB(3, 1);
    for (int i = 0; i < 3; i++) {
      for (int k = 0; k < 2; k++) { MA(i,k) = A.at<double>(i,k); }
      MB(i,0) = B.at<double>(i,0);
    }
    Eigen::VectorXd x = MA.fullPivHouseholderQr().solve(MB);
    double x1 = p11.x + x(0,0) * (p12.x - p11.x);
    double y1 = p11.y + x(0,0) * (p12.y - p11.y);
    double z1 = p11.z + x(0,0) * (p12.z - p11.z);
    double x2 = p21.x + x(1,0) * (p22.x - p21.x);
    double y2 = p21.y + x(1,0) * (p22.y - p21.y);
    double z2 = p21.z + x(1,0) * (p22.z - p21.z);
    std::vector<double> sol = {x1, y1, z1, x2, y2, z2};
    double relative_error = (MA * x - MB).norm() / MB.norm(); 
    if (relative_error > slam_sys_pfr_pr.triangulation_relative_error) { // 0.0002
      sol[0] = -9999999;
      return sol;
    }
    return sol;
}

std::vector<MapMatchFt_C>
_slam__find_p2p_correspondence(const Map &map, const std::vector<MapPoint> &mp1, const std::vector<MapPoint> &mp2)
{
  std::vector<MapMatchFt_C> mapMatchFt;
  for (int i = 0; i < mp1.size(); i++) {
    bool found = false;
    std::vector<cv::DMatch> m = _slam__match_features(mp1[i].descriptor, mp2[i].descriptor);
    cv::Point2f new_old_pt = mp1[i].keypoint.pt;
    for (auto &mm : map) { 
      if (found) { break; }
      for (auto &s : mm.sector_points) {
        if (s.keypoint.pt == new_old_pt) {
          mapMatchFt.push_back(MapMatchFt_C{mp1[i], mp2[i], s, (int)m[0].distance});
          found = true;
          break;
        }
      }
    }
  }
  return mapMatchFt;
}

cv::Rect 
_slam__detect_screen_region_for_map(const SlamSystem &slam_sys, const Map &obj_map, cv::Mat &image)
{
  std::vector<cv::KeyPoint> old_keypoints;
  std::vector<MapPoint> matched_points1;
  std::vector<MapPoint> matched_points2;
  cv::vector<cv::Point2f> img_points_vector;
  cv::vector<cv::Point3f> obj_points_vector;
  cv::Mat pose;
  cv::Mat old_descriptors = cv::Mat(0, slam_sys_pfr_pr.brief_patch_size, CV_8U);
  std::vector<cv::DMatch> new_matches;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<MapMatchFt_C> mapMatchFt;
  keypoints = _slam__search_features(slam_sys, image);
  descriptors = _slam__extract_descriptors(slam_sys, image, keypoints);
  map__merge_keypoints_and_descriptors(obj_map, old_keypoints, old_descriptors);
  new_matches = _slam__match_features(old_descriptors, descriptors);
  _slam__split_matches(new_matches, old_keypoints, old_descriptors, keypoints, descriptors, matched_points1, matched_points2);
  mapMatchFt = _slam__find_p2p_correspondence(obj_map, matched_points1, matched_points2);
  cv::Point2f coord = cv::Point2f(0,0);
  int counter = 0;
  if (mapMatchFt.size() > 400) {
    for (auto &mmf : mapMatchFt) {
      if (mmf.accuracy_index > 20) { continue; }
      coord.x += mmf.new_found_ft_in_new_image.keypoint.pt.x;
      coord.y += mmf.new_found_ft_in_new_image.keypoint.pt.y;
      counter++;
    }
    coord.x /= counter;
    coord.y /= counter;
  }
  return cv::Rect(coord - cv::Point2f(50,50), coord + cv::Point2f(50,50));
}


cv::Point3f
match_position_in_marker_frame(cv::Mat camera_pose, cv::Point3d p_solution) {
  cv::Mat from_cam_2_match = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat points = cv::Mat(3, 1, CV_64F);
  cv::Mat rot = cv::Mat(3, 3, CV_64F);
  from_cam_2_match.at<double>(0,3) = p_solution.x;
  from_cam_2_match.at<double>(1,3) = p_solution.y;
  from_cam_2_match.at<double>(2,3) = p_solution.z;
  cv::Mat res = camera_pose* from_cam_2_match;
  cv::Point3f match_in_marker = cv::Point3f(res.at<double>(0,3), res.at<double>(1,3), res.at<double>(2,3));
  return match_in_marker;
}

cv::Mat 
slam__estimated_pose(std::vector<MapPoint> matches, MyCamereParamReader camera) 
{
  cv::Mat pose;
  cv::Mat rVec, tVec, rot_mat;
  cv::vector<cv::Point2f> img_points_vector;
  cv::vector<cv::Point3f> obj_points_vector;
  for (auto &m : matches) {
    obj_points_vector.push_back(m.coords_3D);
    img_points_vector.push_back(m.keypoint.pt);
  }
  cv::solvePnPRansac(obj_points_vector, 
               img_points_vector, 
               camera.getCameraMatrix(), 
               camera.getDistorsion(), 
               rVec, tVec, 
               false, 1000, 5, 100, cv::noArray(),  
               CV_EPNP);
  cv::Rodrigues(rVec, rot_mat);
  pose = rot_tr_mat(rot_mat, tVec);
  return pose;
}

cv::Mat 
slam__estimated_pose(cv::vector<cv::Point2f> img_points_vector, 
                     cv::vector<cv::Point3f> obj_points_vector, 
                     MyCamereParamReader camera) 
{
  cv::Mat pose;
  cv::Mat rVec, tVec, rot_mat;
  cv::solvePnPRansac(obj_points_vector, 
               img_points_vector, 
               camera.getCameraMatrix(), 
               camera.getDistorsion(), 
               rVec, tVec, 
               false, 1000, 5, 100, cv::noArray(),
               CV_EPNP);
  cv::Rodrigues(rVec, rot_mat);
  pose = rot_tr_mat(rot_mat, tVec);
  return pose;
}

/* DRAWING */

void 
debug_loc(cv::Mat image, const Map &map, std::vector<cv::Point2f> current_im_points)
{
  for (int i = 0; i < current_im_points.size(); i++) {
    cv::circle(image, current_im_points[i], 5, cv::Scalar(255, 0 ,0));
  }
}

void 
debug_pair(cv::Mat im1, cv::Mat im2, std::vector<MapPoint> matched_points1, std::vector<MapPoint> matched_points2) 
{
  cv::Mat fi = cv::Mat(im1.rows, im1.cols * 2, CV_8UC3);
  im1.copyTo(fi(cv::Rect(0, 0, im1.cols, im1.rows)));
  im2.copyTo(fi(cv::Rect(im1.cols, 0, im2.cols, im2.rows)));
  for (int i = 0; i < matched_points1.size(); i++) {
    cv::circle(fi, matched_points1[i].keypoint.pt, 5, cv::Scalar(255, 0 ,0));
    cv::circle(fi, matched_points2[i].keypoint.pt + cv::Point2f(1280, 0), 5, cv::Scalar(255, 0 ,0));
    cv::line(fi, matched_points1[i].keypoint.pt, matched_points2[i].keypoint.pt + cv::Point2f(1280, 0), cv::Scalar(0, 255 ,0));
  }
  cv::imshow("ni", fi);
  cv::waitKey(10);
}

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
  cv::Rodrigues(rot_mat, rot_mat);
  return rot_mat;
}

void 
draw_cube_on_ref_sys(cv::Mat &image, 
                     cv::Mat camera_matrix, 
                     cv::Mat camera_dist, 
                     cv::Mat pose, 
                     uint side_lenght, 
                     cv::Scalar color)
{
  cv::Mat tr_vec, rot_vec;
  cv::Mat objectPoints (8,3,CV_32FC1);
  cv::vector<cv::Point2f> imagePoints;
  float halfSize = side_lenght / 2;
  objectPoints.at<float>(0,0) = -halfSize;
  objectPoints.at<float>(0,1) = -halfSize;
  objectPoints.at<float>(0,2) = 0;
  objectPoints.at<float>(1,0) = halfSize;
  objectPoints.at<float>(1,1) = -halfSize;
  objectPoints.at<float>(1,2) = 0;
  objectPoints.at<float>(2,0) = halfSize;
  objectPoints.at<float>(2,1) = halfSize;
  objectPoints.at<float>(2,2) = 0;
  objectPoints.at<float>(3,0) = -halfSize;
  objectPoints.at<float>(3,1) = halfSize;
  objectPoints.at<float>(3,2) = 0;
  objectPoints.at<float>(4,0) = -halfSize;
  objectPoints.at<float>(4,1) = -halfSize;
  objectPoints.at<float>(4,2) = side_lenght;
  objectPoints.at<float>(5,0) = halfSize;
  objectPoints.at<float>(5,1) = -halfSize;
  objectPoints.at<float>(5,2) = side_lenght;
  objectPoints.at<float>(6,0) = halfSize;
  objectPoints.at<float>(6,1) = halfSize;
  objectPoints.at<float>(6,2) = side_lenght;
  objectPoints.at<float>(7,0) = -halfSize;
  objectPoints.at<float>(7,1) = halfSize;
  objectPoints.at<float>(7,2) = side_lenght;
  tr_vec = tr_vec_from_pose(pose);
  rot_vec = rot_mat_from_pose(pose);
  cv::projectPoints(objectPoints, rot_vec, tr_vec, camera_matrix, camera_dist, imagePoints);
  for (int i=0;i<4;i++) { cv::line(image, imagePoints[i], imagePoints[(i+1)%4], color, 2, 8); }
  for (int i=0;i<4;i++) { cv::line(image, imagePoints[i+4], imagePoints[4+(i+1)%4], color, 2, 8); }
  for (int i=0;i<4;i++) { cv::line(image, imagePoints[i], imagePoints[i+4], color, 2, 8); }
}


/*std::cout << A << " --- " << B << std::endl;
 cv::invert(A, X, cv::DECOMP_SVD);
 cv::Mat res = X * B;
 std::cout << X * B << std::endl;
 double x1 = p11.x + res.at<double>(0,0) * p12.x - p11.x;
 double y1 = p11.y + res.at<double>(0,0) * p12.y - p11.y;
 double z1 = p11.z + res.at<double>(0,0) * p12.z - p11.z;
 std::cout << x1 << " " << y1 << " " << z1 << std::endl;*/





