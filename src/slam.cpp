/*
 ____  _                 
/ ___|| | __ _ _ __ ___  
\___ \| |/ _` | '_ ` _ \ 
 ___) | | (_| | | | | | |
|____/|_|\__,_|_| |_| |_|
                         
*/

#include "slam.hpp"

KeyframesMap keyframes_map;

struct SlamSystemPerformanceParams
{
  int    accuracy_index               = 50;
  double triangulation_relative_error = 0.02;
  int    orb_n_of_features_to_search  = 8000;
  float  orb_scale_factor             = 1.02f;
  int    orb_n_pyramid                = 32;                                             
  int    orb_brief_patch_size         = 128;
  int    bfm_norm                     = cv::NORM_HAMMING2;
} slam_sys_pfr_pr;

typedef std::chrono::high_resolution_clock::time_point mtime;

mtime
get_chrono() { return std::chrono::high_resolution_clock::now(); }

void 
measure_chrono(mtime start_time, 
               std::string mex) 
{
  std::chrono::high_resolution_clock::time_point stop_chrono = get_chrono();
  std::chrono::duration<double> time_span = std::chrono:: duration_cast<std::chrono::duration<double> >(stop_chrono - start_time);
  std::cout << mex << " " << time_span.count() << " seconds." << std::endl;
}

cv::Mat 
rot_tr_mat(cv::Mat rot, cv::Mat tr) ;

std::vector<cv::KeyPoint> 
_slam__search_features(const SlamSystem &slam_sys, cv::Mat image);

cv::Mat 
_slam__extract_descriptors(const SlamSystem &slam_sys, cv::Mat image, std::vector<cv::KeyPoint> keypoint);

std::vector<cv::DMatch> 
_slam__match_features(const SlamSystem &slam_sys, cv::Mat descriptors_1, cv::Mat descriptors_2);

bool 
_slam__are_oblique_keypoints(cv::KeyPoint kp_1, cv::KeyPoint kp_2);

void 
_slam__split_matches(std::vector<cv::DMatch> matches, 
                     std::vector<cv::KeyPoint> keypoints1, 
                     cv::Mat descriptors1, 
                     std::vector<cv::KeyPoint> keypoints2, 
                     cv::Mat descriptors2, 
                     std::vector<MapPoint> &matched_points1, 
                     std::vector<MapPoint> &matched_points2,
                     bool remove_oblique = false);

std::vector<MapPoint> 
_slam__triangulate(CameraSystem camera,
                   cv::Mat image_1, 
                   std::vector<MapPoint> matched_points1, 
                   cv::Mat pose1, 
                   std::vector<MapPoint> matched_points2, 
                   cv::Mat pose2,
                   cv::Mat image_2);

cv::Mat_<double>
_slam__linear_triangulation(cv::Point3d u,
                            cv::Mat P,
                            cv::Point3d u1,
                            cv::Mat P1);

std::vector<MapPoint> 
_slam__triangulate(CameraSystem camera,
                   cv::Mat image_1, 
                   std::vector<MapPoint> matched_points1, 
                   std::vector<MapPoint> matched_points2, 
                   cv::Mat from1to2);

cv::Mat_<double>  
_slam__linear_triangulation_iterative(cv::Point3d u,
                                      cv::Mat P,
                                      cv::Point3d u1,
                                      cv::Mat P1);

std::vector<double> 
_slam__solve_linear_system(cv::Point3f p11, cv::Point3f p12, cv::Point3f p21, cv::Point3f p22); 

cv::Rect 
_slam__detect_screen_region_for_map(const SlamSystem &slam_sys, const Map &obj_map, cv::Mat &image);

cv::Point3f 
match_position_in_marker_frame(cv::Mat camera_pose, cv::Point3d p_solution);

cv::Mat 
_slam__estimated_pose(std::vector<MapPoint> matches); 

cv::Mat 
_slam__estimated_pose(std::vector<cv::Point2f> img_points_vector, std::vector<cv::Point3f> obj_points_vector); 

std::vector<MapMatchFt_C> 
_slam__find_p2p_correspondence(const SlamSystem &slam_sys, 
                               const Map &map,  
                               const std::vector<MapPoint> &mp1, 
                               const std::vector<MapPoint> &mp2);

void  
_slam__good_data_for_solvePnp(std::vector<MapMatchFt_C> &mapMatchFt,
                              std::vector<cv::Point2f> &img_points_vector,
                              std::vector<cv::Point3f> &obj_points_vector);

void 
_slam__correct_matches(std::vector<MapPoint> &matched_points1, std::vector<MapPoint> &matched_points2);

void 
debug_loc(cv::Mat image, const Map &map, std::vector<cv::Point2f> current_im_points);

void 
debug_pair(cv::Mat im1, cv::Mat im2, std::vector<MapPoint> matched_points1, std::vector<MapPoint> matched_points2);

double 
matrix_norm(cv::Mat matrix);

cv::Mat 
pose_mat_from_tr_and_rot(cv::Mat tr_mat, cv::Mat rot_mat);

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

void 
slam__init(SlamSystem &slam_sys, const std::string camera_path) 
{
  #if CV_MAJOR_VERSION == 2
  slam_sys.features_detector = new cv::OrbFeatureDetector(slam_sys_pfr_pr.orb_n_of_features_to_search, 
                                               slam_sys_pfr_pr.orb_scale_factor, 
                                               slam_sys_pfr_pr.orb_n_pyramid, 
                                               slam_sys_pfr_pr.orb_brief_patch_size - 1, 
                                               0, 
                                               2, 
                                               cv::ORB::HARRIS_SCORE, 
                                               slam_sys_pfr_pr.orb_brief_patch_size - 1);
  slam_sys.descriptions_extractor = new cv::OrbDescriptorExtractor(slam_sys_pfr_pr.orb_n_of_features_to_search, 
                                               slam_sys_pfr_pr.orb_scale_factor, 
                                               slam_sys_pfr_pr.orb_n_pyramid, 
                                               slam_sys_pfr_pr.orb_brief_patch_size - 1, 
                                               0, 
                                               2, 
                                               cv::ORB::HARRIS_SCORE, 
                                               slam_sys_pfr_pr.orb_brief_patch_size - 1);
  #elif CV_MAJOR_VERSION == 3
  slam_sys.features_detector = cv::ORB::create(slam_sys_pfr_pr.orb_n_of_features_to_search, 
                                               slam_sys_pfr_pr.orb_scale_factor, 
                                               slam_sys_pfr_pr.orb_n_pyramid, 
                                               slam_sys_pfr_pr.orb_brief_patch_size - 1, 
                                               0, 
                                               2, 
                                               cv::ORB::HARRIS_SCORE, 
                                               slam_sys_pfr_pr.orb_brief_patch_size - 1); 
  slam_sys.descriptions_extractor = cv::ORB::create(slam_sys_pfr_pr.orb_n_of_features_to_search, 
                                               slam_sys_pfr_pr.orb_scale_factor, 
                                               slam_sys_pfr_pr.orb_n_pyramid, 
                                               slam_sys_pfr_pr.orb_brief_patch_size - 1, 
                                               0, 
                                               2, 
                                               cv::ORB::HARRIS_SCORE, 
                                               slam_sys_pfr_pr.orb_brief_patch_size - 1);
  #endif
  slam_sys.matcher = cv::BFMatcher(slam_sys_pfr_pr.bfm_norm, true);
  slam_sys.camera = CameraSystem();
  slam_sys.camera.set_from_yaml(camera_path);
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
  std::future<std::vector<cv::KeyPoint>> sf1 = std::async(std::launch::async, _slam__search_features, slam_sys, image_1);
  std::future<std::vector<cv::KeyPoint>> sf2 = std::async(std::launch::async, _slam__search_features, slam_sys, image_2);
  std::vector<cv::KeyPoint> keypoints1 = sf1.get();
  std::vector<cv::KeyPoint> keypoints2 = sf2.get();
  std::future<cv::Mat> df1 = std::async(std::launch::async, _slam__extract_descriptors, slam_sys, image_1, keypoints1);
  std::future<cv::Mat> df2 = std::async(std::launch::async, _slam__extract_descriptors, slam_sys, image_2, keypoints2);
  cv::Mat descriptors1 = df1.get();
  cv::Mat descriptors2 = df2.get();
  std::vector<cv::DMatch> matches = _slam__match_features(slam_sys, descriptors1, descriptors2); 
  _slam__split_matches(matches, keypoints1, descriptors1, keypoints2, descriptors2, matched_points1, matched_points2, true);
  //_slam__correct_matches(matched_points1, matched_points2);
  //debug_pair(image_1, image_2, matched_points1, matched_points2);
  std::vector<MapPoint> total_matches = _slam__triangulate(slam_sys.camera, image_1, matched_points1, pose_1, matched_points2, pose_2, image_2);
  
  ////////////////////////////////////////////
  ////////////////////////////////////////////
  Keyframe kf = Keyframe(pose_2, image_2, slam_sys_pfr_pr.orb_brief_patch_size);
  for (auto &mp : total_matches) {
    kf.keypoints.push_back(mp.keypoint);
    kf.coords_3D.push_back(mp.coords_3D);
    kf.descriptors.push_back(mp.descriptor);
  }
  keyframes_map << kf;
  //if (keyframes_map.keyframes.size() > 2) {
  //  keyframes_map.adjust(slam_sys.camera.camera_matrix);
  //}
  ////////////////////////////////////////////
  ////////////////////////////////////////////

  return total_matches;
}

void 
slam__adjust_map(const SlamSystem &slam_sys)
{
  keyframes_map.adjust(slam_sys.camera.camera_matrix);
}

cv::Mat 
slam__localize(const SlamSystem &slam_sys, const Map &map, cv::Mat &image)
{
  std::vector<cv::KeyPoint> old_keypoints;
  std::vector<MapPoint> matched_points1;
  std::vector<MapPoint> matched_points2;
  std::vector<cv::Point2f> img_points_vector;
  std::vector<cv::Point3f> obj_points_vector;
  cv::Mat pose;
  cv::Mat old_descriptors = cv::Mat(0, slam_sys_pfr_pr.orb_brief_patch_size, CV_8U);
  std::vector<cv::DMatch> new_matches;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<MapMatchFt_C> mapMatchFt;
  keypoints = _slam__search_features(slam_sys, image);
  descriptors = _slam__extract_descriptors(slam_sys, image, keypoints);
  //Keyframe kf = keyframes_map.end();
  //old_descriptors = kf.descriptors;
  //old_keypoints = kf.keypoints;
  map__merge_keypoints_and_descriptors(map, old_keypoints, old_descriptors);
  new_matches = _slam__match_features(slam_sys, old_descriptors, descriptors); 
  _slam__split_matches(new_matches, old_keypoints, old_descriptors, keypoints, descriptors, matched_points1, matched_points2);
  mapMatchFt = _slam__find_p2p_correspondence(slam_sys, map, matched_points1, matched_points2);
  _slam__good_data_for_solvePnp(mapMatchFt, img_points_vector, obj_points_vector);
  if (img_points_vector.size() < 7) { return cv::Mat(4, 4, CV_64F, double(0)); }
  std::cout << "---> old:  " << old_keypoints.size() << " new: " << new_matches.size() << " afterp2p: " << mapMatchFt.size() << " afterGoodPnp: "<< img_points_vector.size() << std::endl; 
  pose = slam__estimated_pose(img_points_vector, obj_points_vector, slam_sys.camera);
  //debug_loc(image, map, img_points_vector);
  //map__draw(image, map, tr_vec_from_pose(pose), rot_mat_from_pose(pose), slam_sys.camera.cameraMatrix, slam_sys.camera.distorsion);
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

void 
_slam__good_data_for_solvePnp(std::vector<MapMatchFt_C> &mapMatchFt,
                              std::vector<cv::Point2f>  &img_points_vector,
                              std::vector<cv::Point3f>  &obj_points_vector)
{
  int i = 0; 
  std::sort(mapMatchFt.begin(), mapMatchFt.end());
  while (true) {
    if (i == mapMatchFt.size()) { break; }
    if ((mapMatchFt[i].accuracy_index > slam_sys_pfr_pr.accuracy_index)) { break; }
    img_points_vector.push_back(mapMatchFt[i].new_found_ft_in_new_image.keypoint.pt);
    obj_points_vector.push_back(mapMatchFt[i].ref_match.coords_3D); 
    i++;
  }
}

void compute_params_for_image(const SlamSystem &slam_sys, ImageForMapping &ifm)
{
  ifm.keypoint = _slam__search_features(slam_sys, ifm.frame);
  ifm.descriptor = _slam__extract_descriptors(slam_sys, ifm.frame, ifm.keypoint);
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
_slam__match_features(const SlamSystem &slam_sys, 
                      cv::Mat descriptors_1, 
                      cv::Mat descriptors_2)
{
  std::vector<std::vector<cv::DMatch>> matches;
  std::vector<cv::DMatch> tot_matches;
  size_t t_size = 0;
  slam_sys.matcher.knnMatch(descriptors_1, descriptors_2, matches, 1);
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

cv::Point3d 
_normalized_point(const cv::Mat camera_matrix_e, cv::Point2f point)
{
  cv::Mat p(3, 1, CV_32FC1);
  p.at<float>(0,0) = point.x;
  p.at<float>(1,0) = point.y;
  p.at<float>(2,0) = 1;
  cv::Mat r = camera_matrix_e.inv() * p;
  return cv::Point3d(r.at<float>(0,0), r.at<float>(1,0), r.at<float>(2,0));
}

std::vector<MapPoint>     
_slam__triangulate(CameraSystem camera,
                   cv::Mat image_1, 
                   std::vector<MapPoint> matched_points1, 
                   cv::Mat pose1, 
                   std::vector<MapPoint> matched_points2, 
                   cv::Mat pose2,
                   cv::Mat image_2)
{
  std::vector<MapPoint> total_matches;
  cv::Mat gray_1;
  cv::cvtColor(image_1, gray_1, CV_BGRA2GRAY);
  cv::Mat gray_2;
  cv::cvtColor(image_2, gray_2, CV_BGRA2GRAY);
  for (int gm = 0; gm < matched_points1.size(); gm++) {
    std::vector<cv::Point2f> corners1;
    std::vector<cv::Point2f> corners2;
    corners1.push_back(matched_points1[gm].keypoint.pt);
    corners2.push_back(matched_points2[gm].keypoint.pt);
    cv::cornerSubPix(gray_1, corners1, cvSize(5, 5), cvSize(-1,-1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 40, 0.01));
    cv::cornerSubPix(gray_2, corners2, cvSize(5, 5), cvSize(-1,-1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 40, 0.01));
    matched_points1[gm].keypoint.pt = corners1[0];
    matched_points2[gm].keypoint.pt = corners2[0];
    //std::cout << matched_points1[gm].keypoint.pt << " " << corners[0] << std::endl;
    cv::Point3d norm_p1 = _normalized_point(camera.camera_matrix, matched_points1[gm].keypoint.pt);
    cv::Point3d norm_p2 = _normalized_point(camera.camera_matrix, matched_points2[gm].keypoint.pt);
    cv::Mat X = _slam__linear_triangulation_iterative(norm_p1, pose1, norm_p2, pose2);
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    object_points.push_back(cv::Point3f(X.at<double>(0,0), X.at<double>(1,0), X.at<double>(2,0)));
    cv::projectPoints(object_points, rot_mat_from_pose(pose2), tr_vec_from_pose(pose2), camera.camera_matrix, camera.distorsion, image_points);
    float err = sqrt(pow(image_points[0].x - matched_points2[gm].keypoint.pt.x, 2) + 
                     pow(image_points[0].y - matched_points2[gm].keypoint.pt.y, 2));
    if (err > slam_sys_pfr_pr.triangulation_relative_error) { continue; } 
    matched_points2[gm].coords_3D = cv::Point3d(X.at<double>(0,0), X.at<double>(1,0), X.at<double>(2,0)); 
    matched_points2[gm].pose = pose2;
    matched_points2[gm].hits++;
    matched_points2[gm].pixel_colors = image_1.at<cv::Vec3b>(matched_points1[gm].keypoint.pt.y, matched_points1[gm].keypoint.pt.x);
    total_matches.push_back(matched_points2[gm]);
  }
  return total_matches;
}

std::vector<MapPoint>     
_slam__triangulate(CameraSystem camera,
                   cv::Mat image_1, 
                   std::vector<MapPoint> matched_points1, 
                   std::vector<MapPoint> matched_points2, 
                   cv::Mat from1to2)
{
  std::vector<MapPoint> total_matches;
  for (int gm = 0; gm < matched_points1.size(); gm++) {
    cv::Mat cam_points_11 = camera.cam_point_from_pixel(matched_points1[gm].keypoint.pt, 100);
    cv::Mat cam_points_12 = camera.cam_point_from_pixel(matched_points1[gm].keypoint.pt, 20000);
    cv::Mat cam_points_21 = camera.cam_point_from_pixel(matched_points2[gm].keypoint.pt, 100);
    cv::Mat cam_points_22 = camera.cam_point_from_pixel(matched_points2[gm].keypoint.pt, 20000);
    cam_points_21   = from1to2 * cam_points_21;
    cam_points_22   = from1to2 * cam_points_22;
    cv::Point3f p11 = cv::Point3f(cam_points_11.at<double>(0,0), cam_points_11.at<double>(1,0), cam_points_11.at<double>(2,0));
    cv::Point3f p12 = cv::Point3f(cam_points_12.at<double>(0,0), cam_points_12.at<double>(1,0), cam_points_12.at<double>(2,0));
    cv::Point3f p21 = cv::Point3f(cam_points_21.at<double>(0,0), cam_points_21.at<double>(1,0), cam_points_21.at<double>(2,0));
    cv::Point3f p22 = cv::Point3f(cam_points_22.at<double>(0,0), cam_points_22.at<double>(1,0), cam_points_22.at<double>(2,0));        
    std::vector<double> solution = _slam__solve_linear_system(p11, p12, p21, p22);
    if (solution[0] < -9999998) { continue; } 
    matched_points2[gm].coords_3D = match_position_in_marker_frame(from1to2.inv(), cv::Point3d(solution[3], solution[4], solution[5]));
    matched_points2[gm].hits++;
    matched_points2[gm].pixel_colors = image_1.at<cv::Vec3b>(matched_points1[gm].keypoint.pt.y, matched_points1[gm].keypoint.pt.x);
    total_matches.push_back(matched_points2[gm]);
  }
  return total_matches;
}

void
_slam__correct_matches(std::vector<MapPoint> &matched_points1, std::vector<MapPoint> &matched_points2) 
{
  std::vector<cv::Point2f> im_points_1, im_points_2;
  std::vector<cv::Point2f> im_points_1n, im_points_2n;
  im_points_1.reserve(matched_points1.size());
  im_points_2.reserve(matched_points2.size());
  for (int i = 0; i < matched_points1.size(); i++) { 
    im_points_1.push_back(matched_points1[i].keypoint.pt); 
    im_points_2.push_back(matched_points2[i].keypoint.pt); 
  }
  cv::Mat foundamental = cv::findFundamentalMat(im_points_1, im_points_2, CV_FM_RANSAC);
  if (foundamental.empty()) {
    return;
  }
  cv::correctMatches(foundamental, im_points_1, im_points_2, im_points_1n, im_points_2n);
  for (int i = 0; i < matched_points1.size(); i++) { 
    matched_points1[i].keypoint.pt = im_points_1n[i];
    matched_points2[i].keypoint.pt = im_points_2n[i];
  }
}

double 
matrix_norm(cv::Mat matrix)
{
  double max = 0.0; /* tutte le somme sono >= 0 */ 
  for (int j = 0; j < matrix.cols; ++j) {
    double sum = 0.0;
    for (int i = 0; i < matrix.rows; ++i) {
        sum += fabs(matrix.at<double>(i,j));
    }
    if (sum > max) {
        max = sum;
    }
  }
  return max;
}

cv::Mat_<double>  
_slam__linear_triangulation_iterative(cv::Point3d u,
                                      cv::Mat P,
                                      cv::Point3d u1,
                                      cv::Mat P1)
{
  double THRESH = 0.0000001f;
  double wi = 1, wi1 = 1;
  cv::Mat_<double> X(4,1);
  cv::Mat_<double> X_ = _slam__linear_triangulation(u, P, u1, P1);
  X(0) = X_(0);
  X(1) = X_(1);
  X(2) = X_(2);
  X(3) = 1;
  for (int i = 0; i < 10; i++) {
    double p2x  = cv::Mat_<double>(cv::Mat(P).row(2) *  X)(0);
    double p2x1 = cv::Mat_<double>(cv::Mat(P1).row(2) * X)(0);
    if (std::abs(wi - p2x) <= THRESH && std::abs(wi1 - p2x1) <= THRESH) { break; }
    wi  = p2x;
    wi1 = p2x1;
    cv::Matx43d A((u.x  * P.at<double>(2,0)  - P.at<double>(0,0))  / wi,  (u.x  * P.at<double>(2,1)  - P.at<double>(0,1))  / wi,    (u.x *  P.at<double>(2,2)  - P.at<double>(0,2))  / wi,   
                  (u.y  * P.at<double>(2,0)  - P.at<double>(1,0))  / wi,  (u.y  * P.at<double>(2,1)  - P.at<double>(1,1))  / wi,    (u.y *  P.at<double>(2,2)  - P.at<double>(1,2))  / wi,   
                  (u1.x * P1.at<double>(2,0) - P1.at<double>(0,0)) / wi1, (u1.x * P1.at<double>(2,1) - P1.at<double>(0,1)) / wi1,   (u1.x * P1.at<double>(2,2) - P1.at<double>(0,2)) / wi1, 
                  (u1.y * P1.at<double>(2,0) - P1.at<double>(1,0)) / wi1, (u1.y * P1.at<double>(2,1) - P1.at<double>(1,1)) / wi1,   (u1.y * P1.at<double>(2,2) - P1.at<double>(1,2)) / wi1);
    cv::Mat_<double> B = (cv::Mat_<double>(4,1) << -(u.x  * P.at<double>(2,3)  - P.at<double>(0,3))  / wi,
                                                   -(u.y  * P.at<double>(2,3)  - P.at<double>(1,3))  / wi, 
                                                   -(u1.x * P1.at<double>(2,3) - P1.at<double>(0,3)) / wi1, 
                                                   -(u1.y * P1.at<double>(2,3) - P1.at<double>(1,3)) / wi1);
    cv::solve(A, B, X_, cv::DECOMP_SVD);
    X(0) = X_(0); 
    X(1) = X_(1); 
    X(2) = X_(2); 
    X(3) = 1.0;
  }
  return X; 
}

cv::Mat_<double>
_slam__linear_triangulation(cv::Point3d u,
                            cv::Mat P,
                            cv::Point3d u1,
                            cv::Mat P1)
{
  cv::Matx43d A(u.x  * P.at<double>(2,0)  -P.at<double>(0,0),   u.x  * P.at<double>(2,1)  - P.at<double>(0,1),   u.x*P.at<double>(2,2)   - P.at<double>(0,2),
                u.y  * P.at<double>(2,0)  -P.at<double>(1,0),   u.y  * P.at<double>(2,1)  - P.at<double>(1,1),   u.y*P.at<double>(2,2)   - P.at<double>(1,2),
                u1.x * P1.at<double>(2,0) -P1.at<double>(0,0),  u1.x * P1.at<double>(2,1) - P1.at<double>(0,1),  u1.x*P1.at<double>(2,2) - P1.at<double>(0,2),
                u1.y * P1.at<double>(2,0) -P1.at<double>(1,0),  u1.y * P1.at<double>(2,1) - P1.at<double>(1,1),  u1.y*P1.at<double>(2,2) - P1.at<double>(1,2));
  cv::Matx41d B(-(u.x  * P.at<double>(2,3)  - P.at<double>(0,3)), 
                -(u.y  * P.at<double>(2,3)  - P.at<double>(1,3)),
                -(u1.x * P1.at<double>(2,3) - P1.at<double>(0,3)),
                -(u1.y * P1.at<double>(2,3) - P1.at<double>(1,3)));
  cv::Mat_<double> X;
  cv::solve(A, B, X, cv::DECOMP_SVD);
  return X; 
}

// A = 3x2  X = 2x1 B = 3x1
// A * X = 3 * 1
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
  cv::invert(A, X, cv::DECOMP_SVD);
  cv::Mat res = X * B;
  double x1 = p11.x + res.at<double>(0,0) * (p12.x - p11.x);
  double y1 = p11.y + res.at<double>(0,0) * (p12.y - p11.y);
  double z1 = p11.z + res.at<double>(0,0) * (p12.z - p11.z);
  double x2 = p21.x + res.at<double>(0,1) * (p22.x - p21.x);
  double y2 = p21.y + res.at<double>(0,1) * (p22.y - p21.y);
  double z2 = p21.z + res.at<double>(0,1) * (p22.z - p21.z);
  cv::Mat xx = cv::Mat(2, 1, CV_64F);
  xx.at<double>(0,0) = res.at<double>(0,0);
  xx.at<double>(1,0) = res.at<double>(0,1);
  std::vector<double> sol = {x1, y1, z1, x2, y2, z2};
  double relative_error = matrix_norm(A * xx - B) / matrix_norm(B);
  if (relative_error > slam_sys_pfr_pr.triangulation_relative_error) { // 0.0002
    sol[0] = -9999999;
    return sol;
  }
  return sol;
}

std::vector<MapMatchFt_C>
_slam__find_p2p_correspondence(const SlamSystem &slam_sys, const Map &map, const std::vector<MapPoint> &mp1, const std::vector<MapPoint> &mp2)
{
  std::vector<MapMatchFt_C> mapMatchFt;
  for (int i = 0; i < mp1.size(); i++) {
    bool found = false;
    std::vector<cv::DMatch> m = _slam__match_features(slam_sys, mp1[i].descriptor, mp2[i].descriptor);
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
  std::vector<cv::Point2f> img_points_vector;
  std::vector<cv::Point3f> obj_points_vector;
  cv::Mat pose;
  cv::Mat old_descriptors = cv::Mat(0, slam_sys_pfr_pr.orb_brief_patch_size, CV_8U);
  std::vector<cv::DMatch> new_matches;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  std::vector<MapMatchFt_C> mapMatchFt;
  keypoints = _slam__search_features(slam_sys, image);
  descriptors = _slam__extract_descriptors(slam_sys, image, keypoints);
  map__merge_keypoints_and_descriptors(obj_map, old_keypoints, old_descriptors);
  new_matches = _slam__match_features(slam_sys, old_descriptors, descriptors);
  _slam__split_matches(new_matches, old_keypoints, old_descriptors, keypoints, descriptors, matched_points1, matched_points2);
  mapMatchFt = _slam__find_p2p_correspondence(slam_sys, obj_map, matched_points1, matched_points2);
  cv::Point2f coord = cv::Point2f(0,0);
  std::cout << mapMatchFt.size() << std::endl;
  int counter = 0;
  if (mapMatchFt.size() > 50) {
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
  cv::Mat res = camera_pose * from_cam_2_match;
  cv::Point3f match_in_marker = cv::Point3f(res.at<double>(0,3), res.at<double>(1,3), res.at<double>(2,3));
  return match_in_marker;
}

cv::Mat 
slam__estimated_pose(std::vector<MapPoint> matches, CameraSystem camera) 
{
  cv::Mat pose;
  cv::Mat rVec, tVec, rot_mat;
  std::vector<cv::Point2f> img_points_vector;
  std::vector<cv::Point3f> obj_points_vector;
  for (auto &m : matches) {
    obj_points_vector.push_back(m.coords_3D);
    img_points_vector.push_back(m.keypoint.pt);
  }
  #if CV_MAJOR_VERSION == 3
  cv::solvePnPRansac(obj_points_vector, 
               img_points_vector, 
               camera.camera_matrix, 
               camera.distorsion, 
               rVec, tVec, 
               false, 1000, 2, 0.999, cv::noArray(),
               cv::SOLVEPNP_ITERATIVE);
  #elif CV_MAJOR_VERSION == 2
  cv::solvePnPRansac(obj_points_vector, 
               img_points_vector, 
               camera.camera_matrix, 
               camera.distorsion, 
               rVec, tVec, 
               false, 1000, 5, 100, cv::noArray(),  
               CV_EPNP);
  #endif
  cv::Rodrigues(rVec, rot_mat);
  pose = rot_tr_mat(rot_mat, tVec);
  return pose;
}

cv::Mat 
slam__estimated_pose(std::vector<cv::Point2f> img_points_vector, 
                     std::vector<cv::Point3f> obj_points_vector, 
                     CameraSystem camera) 
{
  cv::Mat pose;
  cv::Mat rVec, tVec, rot_mat;
  #if CV_MAJOR_VERSION == 3
  cv::solvePnPRansac(obj_points_vector, 
               img_points_vector, 
               camera.camera_matrix, 
               camera.distorsion, 
               rVec, tVec, 
               false, 1000, 2, 0.999, cv::noArray(),
               cv::SOLVEPNP_ITERATIVE);
  #elif CV_MAJOR_VERSION == 2
  cv::solvePnPRansac(obj_points_vector, 
               img_points_vector, 
               camera.camera_matrix, 
               camera.distorsion, 
               rVec, tVec, 
               false, 1000, 5, 100, cv::noArray(),  
               CV_EPNP);
  #endif
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
  cv::waitKey(5000);
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

cv::Mat 
pose_mat_from_tr_and_rot(cv::Mat tr_mat, cv::Mat rot_mat)
{
  cv::Mat camera_mat(4, 4, CV_64F);
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      camera_mat.at<double>(row, col) = rot_mat.at<double>(row, col);
    }
  } 
  camera_mat.at<double>(0,3) = tr_mat.at<double>(0,0);
  camera_mat.at<double>(1,3) = tr_mat.at<double>(1,0);
  camera_mat.at<double>(2,3) = tr_mat.at<double>(2,0);
  camera_mat.at<double>(3,3) = 1;
  camera_mat.at<double>(3,0) = 0;
  camera_mat.at<double>(3,1) = 0;
  camera_mat.at<double>(3,2) = 0;
  return camera_mat;
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
  std::vector<cv::Point2f> imagePoints;
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


/*
  Under develop
*/
#if CV_MAJOR_VERSION == 3
std::vector<MapPoint> 
slam__map(const SlamSystem &slam_sys, cv::Mat image_1, cv::Mat image_2)
{
  std::vector<MapPoint> matched_points1, matched_points2;
  std::vector<cv::KeyPoint> ok1, ok2;
  std::vector<cv::Point2f> im_points_1, im_points_2;
  std::future<std::vector<cv::KeyPoint>> sf1 = std::async(std::launch::async, _slam__search_features, slam_sys, image_1);
  std::future<std::vector<cv::KeyPoint>> sf2 = std::async(std::launch::async, _slam__search_features, slam_sys, image_2);
  std::vector<cv::KeyPoint> keypoints1 = sf1.get();
  std::vector<cv::KeyPoint> keypoints2 = sf2.get();
  std::future<cv::Mat> df1 = std::async(std::launch::async, _slam__extract_descriptors, slam_sys, image_1, keypoints1);
  std::future<cv::Mat> df2 = std::async(std::launch::async, _slam__extract_descriptors, slam_sys, image_2, keypoints2);
  cv::Mat descriptors1 = df1.get();
  cv::Mat descriptors2 = df2.get();
  std::vector<cv::DMatch> matches = _slam__match_features(slam_sys, descriptors1, descriptors2);
  _slam__split_matches(matches, keypoints1, descriptors1, keypoints2, descriptors2, matched_points1, matched_points2, true);
  im_points_1.reserve(matched_points1.size());
  im_points_2.reserve(matched_points2.size());
  for (auto m1 : matched_points1) { 
    im_points_1.push_back(m1.keypoint.pt); 
    ok1.push_back(m1.keypoint);
  }
  for (auto m2 : matched_points2) { 
    im_points_2.push_back(m2.keypoint.pt); 
    ok2.push_back(m2.keypoint);
  } 
  cv::Mat rot_mat;
  cv::Mat tr_mat;
  cv::Mat essential_matrix = findEssentialMat(im_points_1, im_points_2, 1152, cv::Point2d(0, 0));
  int inliner = cv::recoverPose(essential_matrix, im_points_1, im_points_2, rot_mat, tr_mat);
  cv::Mat pose_2_to_1 = pose_mat_from_tr_and_rot(tr_mat, rot_mat);
  cv::Mat mtxR, mtxQ;
  cv::Mat Qx, Qy, Qz;
  cv::Vec3d angles = RQDecomp3x3(rot_mat, mtxR, mtxQ, Qx, Qy, Qz);
  std::cout << "Translation: " << tr_mat.t() << std::endl;
  std::cout << "Euler angles [x y z] in degrees: " << angles.t() << std::endl;
  std::vector<MapPoint> total_matches = _slam__triangulate(slam_sys.camera, image_1, matched_points1, matched_points2, pose_2_to_1); 
  return total_matches;
}
#endif


// OLD Eigen Triangulation
/*  A x = B   //#include "/usr/local/include/eigen/Eigen/Dense" */
/*std::vector<double> 
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
    std::cout << "-------------------" << std::endl;
    std::cout << MA << "#\n" << x << "#\n" << MB << std::endl;
    double relative_error = (MA * x - MB).norm() / MB.norm(); 
    if (relative_error > slam_sys_pfr_pr.triangulation_relative_error) { // 0.0002
      sol[0] = -9999999;
      return sol;
    }
    return sol;
}*/


  /*
  cv::SVD decomp = cv::SVD(essential_matrix);
  //U
  cv::Mat U = decomp.u;
  //S
  cv::Mat S(3, 3, CV_32F, cv::Scalar(0));
  S.at<float>(0, 0) = decomp.w.at<float>(0, 0);
  S.at<float>(1, 1) = decomp.w.at<float>(0, 1);
  S.at<float>(2, 2) = decomp.w.at<float>(0, 2);
  //V
  cv::Mat V = decomp.vt; //Needs to be decomp.vt.t(); (transpose once more)
  //W
  cv::Mat W(3, 3, CV_32F, cv::Scalar(0));
  W.at<float>(0, 1) = -1;
  W.at<float>(1, 0) = 1;
  W.at<float>(2, 2) = 1;
  std::cout << "computed rotation: " << std::endl;
  std::cout << U * W * V.t() << std::endl; // U * W.t() * V.t()
  std::cout << "TR: " << U << std::endl;*/

  // Essential matrix
  /*cv::Mat ess_mask_out;
  cv::Mat foundamental_matrix_32;
  cv::Mat foundamental_matrix_64 = cv::findFundamentalMat(im_points_1, im_points_2, CV_RANSAC, 3, 0.99, ess_mask_out);
  foundamental_matrix_64.convertTo( foundamental_matrix_32, CV_32FC1 );
  cv::Mat essential_matrix = slam_sys.camera.cameraMatrix.t() * foundamental_matrix_32 * slam_sys.camera.cameraMatrix;
  
  cv::SVD svd = cv::SVD(essential_matrix);
  cv::Matx33f W(0,-1,0,1,0,0,0,0,1);
  cv::Mat_<float> R = svd.u * cv::Mat(W) * svd.vt; //HZ 9.19
  cv::Mat_<float> t = svd.u.col(2); //u3

  if (!CheckCoherentRotation(R)) { return; };
  
  cv::Matx34f P(1,0,0,0, 0,1,0,0, 0,0,1,0);
  cv::Matx34f P1(R(0,0),R(0,1),R(0,2),t(0),
                 R(1,0),R(1,1),R(1,2),t(1),
                 R(2,0),R(2,1),R(2,2),t(2));

  cv::Mat cam_matrix;
  cam_matrix = slam_sys.camera.cameraMatrix.inv();
  cam_matrix.convertTo(cam_matrix,CV_32F); 
  std::vector<cv::Point3d> point_cloud;
  double rel_err = TriangulatePoints(ok1,
                  ok2,
                  cam_matrix,
                  P,
                  P1,
                  point_cloud);
  
  std::ofstream ofs;
  ofs.open ("test_map_tr.txt", std::ofstream::app);
  for (auto &p : point_cloud) {
    ofs << p.x << " " << p.y << " " << p.z << " " << 0 << " " << 255 << " " << 0 << "\n";
  }
  ofs.close();*/
  /*
   bool 
 CheckCoherentRotation(cv::Mat_<float>& R) {
  if(fabsf(determinant(R))-1.0 > 1e-07) {
      std::cerr << "det(R) != +-1.0, this is not a rotation matrix" << std::endl;
      return false;
  }
  return true;
}


cv::Mat_<float> 
LinearLSTriangulation(cv::Point3d u,  //homogenous image point (u,v,1)
                      cv::Matx34d P,  //camera 1 matrix
                      cv::Point3d u1, //homogenous image point in 2nd camera
                      cv::Matx34d P1) //camera 2 matrix
{
  //build A matrix
  cv::Matx43f A(u.x*P(2,0)-P(0,0),u.x*P(2,1)-P(0,1),u.x*P(2,2)-P(0,2),
  u.y*P(2,0)-P(1,0),u.y*P(2,1)-P(1,1),u.y*P(2,2)-P(1,2),
  u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),u1.x*P1(2,2)-P1(0,2),
  u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),u1.y*P1(2,2)-P1(1,2)
    );
  //build B vector
  cv::Matx41f B(-(u.x*P(2,3)-P(0,3)),
    -(u.y*P(2,3)-P(1,3)),
    -(u1.x*P1(2,3)-P1(0,3)),
    -(u1.y*P1(2,3)-P1(1,3)));
   //solve for X
  cv::Mat_<float> X;
  cv::solve(A, B, X, cv::DECOMP_SVD);
return X; 
}

double 
TriangulatePoints(const std::vector<cv::KeyPoint> &pt_set1,
                  const std::vector<cv::KeyPoint> &pt_set2,
                  const cv::Mat &Kinv,
                  const cv::Matx34f &P,
                  const cv::Matx34f &P1,
                  std::vector<cv::Point3d> &pointcloud)
{
  std::vector<double> reproj_error;
  for (unsigned int i=0; i<pt_set1.size(); i++) {
    //convert to normalized homogeneous coordinates
    cv::Point2f kp = pt_set1[i].pt;
    cv::Point3d u(kp.x,kp.y,1.0);
    cv::Mat_<float> um = Kinv * cv::Mat_<float>(u);
    u = um.at<cv::Point3d>(0);
    cv::Point2f kp1 = pt_set2[i].pt;
    cv::Point3d u1(kp1.x,kp1.y,1.0);
    cv::Mat_<float> um1 = Kinv *cv:: Mat_<float>(u1);
    u1 = um1.at<cv::Point3d>(0);
    //triangulate
    cv::Mat_<float> X = LinearLSTriangulation(u, P, u1, P1);
    //calculate reprojection error
    std::cout << Kinv.rows << " " << Kinv.cols << " " << cv::Mat(P1).rows << " " << cv::Mat(P1).cols << " " << X.rows << X.cols << std::endl;
    std::cout << X << std::endl;
    //cv::Mat_<float> xPt_img = Kinv * cv::Mat(P1) * X; //K * cv::Mat(P1) * X;
    //cv::Point2f xPt_img_(xPt_img(0) / xPt_img(2), xPt_img(1) / xPt_img(2));
    //reproj_error.push_back(cv::norm(xPt_img_-kp1));
    //store 3D point
    pointcloud.push_back(cv::Point3d(X(0),X(1),X(2)));
  }
  //return mean reprojection error
  cv::Scalar me = cv::mean(reproj_error);
  return me[0];
}
*/
