#include "slam.hpp"

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


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
// Private
std::vector<cv::KeyPoint> _slam__search_features(const SlamSystem &slam_sys, cv::Mat image);
cv::Mat _slam__extract_descriptors(const SlamSystem &slam_sys, cv::Mat image, std::vector<cv::KeyPoint> keypoint);
std::vector<cv::DMatch> _slam__match_features(cv::Mat descriptors_1, cv::Mat descriptors_2);
void _slam__split_matches(std::vector<cv::DMatch> matches, 
                          std::vector<cv::KeyPoint> keypoints1, 
                          cv::Mat descriptors1, 
                          std::vector<cv::KeyPoint> keypoints2, 
                          cv::Mat descriptors2, 
                          std::vector<MapPoint> &matched_points1, 
                          std::vector<MapPoint> &matched_points2);
std::vector<MapPoint>     _slam__triangulate(cv::Mat image_1, 
                                       std::vector<MapPoint> matched_points1, 
                                       cv::Mat pose1, 
                                       std::vector<MapPoint> matched_points2, 
                                       cv::Mat pose2);
std::vector<double> _slam__solve_linear_system(cv::Point3f p11, cv::Point3f p12, cv::Point3f p21, cv::Point3f p22); 
cv::Point3f match_position_in_marker_frame(cv::Mat camera_pose, cv::Point3d p_solution);
cv::Mat _slam__estimated_pose(std::vector<MapPoint> matches); 
cv::Mat _slam__estimated_pose(cv::vector<cv::Point2f> img_points_vector, cv::vector<cv::Point3f> obj_points_vector); 
//////


void 
slam__init(SlamSystem &slam_sys, const std::string camera_path) 
{
  slam_sys.features_detector = cv::OrbFeatureDetector(4000);
  slam_sys.descriptions_extractor = cv::OrbDescriptorExtractor();
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
  _slam__split_matches(matches, keypoints1, descriptors1, keypoints2, descriptors2, matched_points1, matched_points2);
  std::vector<MapPoint> total_matches = _slam__triangulate(image_1, matched_points1, pose_1, matched_points2, pose_2);  
  return total_matches;
}

cv::Mat 
slam__localize(const SlamSystem &slam_sys, const Map &map, cv::Mat &image)
{
  std::chrono::high_resolution_clock::time_point st = get_chrono();
  std::vector<cv::KeyPoint> old_keypoints;
  std::vector<MapPoint> matched_points1;
  std::vector<MapPoint> matched_points2;
  cv::vector<cv::Point2f> img_points_vector;
  cv::vector<cv::Point3f> obj_points_vector;
  cv::Mat pose;
  cv::Mat old_descriptors = cv::Mat(0, 32, CV_8U);
  std::vector<cv::DMatch> new_matches;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<MapMatchFt_C> mapMatchFt;
  keypoints = _slam__search_features(slam_sys, image);
  descriptors = _slam__extract_descriptors(slam_sys, image, keypoints);
  for (auto &m : map) { 
    for (auto &s : m.sector_points) {
      old_descriptors.push_back(s.descriptor); 
      old_keypoints.push_back(s.keypoint); 
    }
  }
  new_matches = _slam__match_features(old_descriptors, descriptors);
  std::cout << " Match after: " << new_matches.size() << std::endl;
  _slam__split_matches(new_matches, old_keypoints, old_descriptors, keypoints, descriptors, matched_points1, matched_points2);
  for (int i = 0; i < matched_points1.size(); i++) {
    std::vector<cv::DMatch> m = _slam__match_features(matched_points1[i].descriptor, matched_points2[i].descriptor);
    cv::Point2f new_old_pt = matched_points1[i].keypoint.pt;
    for (auto &mm : map) { 
      for (auto &s : mm.sector_points) {
        if (s.keypoint.pt == new_old_pt) {
          mapMatchFt.push_back(MapMatchFt_C{matched_points1[i], matched_points2[i], s, (int)m[0].distance});
          break;
        }
      }
    }
    if (mapMatchFt.back().accuracy_index < 20) {
      img_points_vector.push_back(mapMatchFt.back().new_found_ft_in_new_image.keypoint.pt);
      obj_points_vector.push_back(mapMatchFt.back().ref_match.coords_3D); 
    }
  }
  std::cout << "Pose size: " << img_points_vector.size() << std::endl;
  if (img_points_vector.size() < 6) { return cv::Mat(4, 4, CV_64F, double(0)); }
  pose = slam__estimated_pose(img_points_vector, obj_points_vector, slam_sys.camera);
  measure_chrono(st, "-> calc_pose_with_matches time: ");
  return pose;
}

std::vector<cv::KeyPoint> 
_slam__search_features(const SlamSystem &slam_sys, cv::Mat image)
{
  std::vector<cv::KeyPoint> keypoints;
  slam_sys.features_detector.detect(image, keypoints);
  return keypoints;
}

cv::Mat 
_slam__extract_descriptors(const SlamSystem &slam_sys, 
                           cv::Mat image, 
                           std::vector<cv::KeyPoint> keypoint)
{
  cv::Mat descriptors;
  slam_sys.descriptions_extractor.compute(image, keypoint, descriptors);
  return descriptors;
}

std::vector<cv::DMatch> 
_slam__match_features(cv::Mat descriptors_1, cv::Mat descriptors_2) 
{
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> tot_matches;
  size_t t_size = 0;
  cv::BFMatcher matcher = cv::BFMatcher(cv::NORM_HAMMING, true);
  matcher.knnMatch(descriptors_1, descriptors_2, matches, 1);
  for (auto &m : matches) { t_size += m.size(); }
  tot_matches.reserve(t_size);
  for (auto &m : matches) { tot_matches.insert(tot_matches.end(), m.begin(), m.end()); }
  return tot_matches;  
}

void 
_slam__split_matches(std::vector<cv::DMatch> matches, 
                     std::vector<cv::KeyPoint> keypoints1, 
                     cv::Mat descriptors1, 
                     std::vector<cv::KeyPoint> keypoints2, 
                     cv::Mat descriptors2, 
                     std::vector<MapPoint> &matched_points1, 
                     std::vector<MapPoint> &matched_points2)
{
  for (auto &gm : matches) {
    MapPoint p1, p2;
    int idx1 = gm.queryIdx;
    int idx2 = gm.trainIdx;
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
    cv::Mat cam_points_11 = cam_point_from_pixel(matched_points1[gm].keypoint.pt, 1);
    cv::Mat cam_points_12 = cam_point_from_pixel(matched_points1[gm].keypoint.pt, 10000);
    cv::Mat cam_points_21 = cam_point_from_pixel(matched_points2[gm].keypoint.pt, 1);
    cv::Mat cam_points_22 = cam_point_from_pixel(matched_points2[gm].keypoint.pt, 10000);
    cam_points_21   = from1to2 * cam_points_21;
    cam_points_22   = from1to2 * cam_points_22;
    cv::Point3f p11 = cv::Point3f(cam_points_11.at<double>(0,0), cam_points_11.at<double>(1,0), cam_points_11.at<double>(2,0));
    cv::Point3f p12 = cv::Point3f(cam_points_12.at<double>(0,0), cam_points_12.at<double>(1,0), cam_points_12.at<double>(2,0));
    cv::Point3f p21 = cv::Point3f(cam_points_21.at<double>(0,0), cam_points_21.at<double>(1,0), cam_points_21.at<double>(2,0));
    cv::Point3f p22 = cv::Point3f(cam_points_22.at<double>(0,0), cam_points_22.at<double>(1,0), cam_points_22.at<double>(2,0));        
    std::vector<double> solution = _slam__solve_linear_system(p11, p12, p21, p22);
    if (solution[0] < -9999998) { continue; } 
    matched_points2[gm].coords_3D = match_position_in_marker_frame(pose1.inv(), cv::Point3d(solution[3], solution[4], solution[5]));
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
    if (relative_error > 0.0005) { // 0.0002
      sol[0] = -9999999;
      return sol;
    }
    return sol;
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
  cv::solvePnP(obj_points_vector, 
               img_points_vector, 
               camera.getCameraMatrix(), 
               camera.getDistorsion(), 
               rVec, tVec, false, 
               CV_ITERATIVE);
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
  cv::solvePnP(obj_points_vector, 
               img_points_vector, 
               camera.getCameraMatrix(), 
               camera.getDistorsion(), 
               rVec, tVec, false, 
               CV_ITERATIVE);
  cv::Rodrigues(rVec, rot_mat);
  pose = rot_tr_mat(rot_mat, tVec);
  return pose;
}



/*for (int i = 0; i < matched_points2.size(); i++) {
  Match ref_match;
  bool already_match = find_closest_match(matched_points2[i], old_matches, ref_match);
  if (already_match) {
    matched_points2.erase(matched_points2.begin() + i);
    matched_points1.erase(matched_points1.begin() + i);
  }
}*/

    ///////////////////////////////////////////
  ///////////////////////////////////////////
  /*
  std::chrono::high_resolution_clock::time_point stc = get_chrono();
  for (int k = 0; k < matched_points2.size(); k++) {
    Match closest_match;
    bool close_enough = find_closest_match(matched_points2[k], matches, closest_match);
    if (!close_enough) { continue; }
    img_points_vector.push_back(matched_points2[k].keypoint.pt);
    obj_points_vector.push_back(closest_match.match_point_in_marker_frame[0]);
  }*
  measure_chrono(stc, "-> calc_pose_with_matches::close_match time: ");*/
  /*std::cout << "obj_points: " << obj_points_vector.size() << std::endl;
  if (img_points_vector.size() != obj_points_vector.size() || obj_points_vector.size() < 6) {
    return cv::Mat(4, 4, CV_64F, double(0));
  }*/
  // pose = estimated_pose(final_mathes);