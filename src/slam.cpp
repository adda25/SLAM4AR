#include "slam.hpp"

std::chrono::high_resolution_clock::time_point
get_chrono() {
  return std::chrono::high_resolution_clock::now();
}

void measure_chrono(std::chrono::high_resolution_clock::time_point start_time, std::string mex) {
  std::chrono::high_resolution_clock::time_point stop_chrono = get_chrono();
  std::chrono::duration<double> time_span =std::chrono:: duration_cast<std::chrono::duration<double> >(stop_chrono - start_time);
  std::cout << mex << time_span.count() << " seconds." << std::endl;
}

cv::Mat rot_tr_mat(cv::Mat rot, cv::Mat tr) {
    cv::Mat result(4, 4, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) { result.at<double>(i,k) = rot.at<double>(i,k); }
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
cam_point_from_pixel(cv::Point2f point, double z) {
  cv::Mat result(4, 1, CV_64FC1);
  result.at<double>(0,0) = (point.x - 640) * z / 1152;
  result.at<double>(1,0) = (point.y - 360) * z / 1152;
  result.at<double>(2,0) = z;
  result.at<double>(3,0) = 1;
  return result;
}

std::vector<Match> 
Slam::map_2_images(cv::Mat image1, cv::Mat image2, cv::Mat relative_pose) {
  std::vector<Match> total_matches;
  return total_matches;
}

std::vector<Match> 
Slam::map_2_images(cv::Mat image1, 
                   cv::Mat pose1, 
                   cv::Mat image2, 
                   cv::Mat pose2, 
                   std::vector<Match> old_matches) 
{
  std::vector<SingleMatchPoint> matched_points1;
  std::vector<SingleMatchPoint> matched_points2;
  std::vector<cv::KeyPoint> keypoints1    = search_features(image1);
  std::vector<cv::KeyPoint> keypoints2    = search_features(image2);
  cv::Mat descriptors1                    = extract_descriptors(image1, keypoints1);
  cv::Mat descriptors2                    = extract_descriptors(image2, keypoints2);
  std::vector<cv::DMatch> matches         = match_features(keypoints1, descriptors1, keypoints2, descriptors2);
  split_matches(matches, keypoints1, descriptors1, keypoints2, descriptors2, matched_points1, matched_points2);
  std::vector<Match> total_matches        = triangulate(image1, matched_points1, pose1, matched_points2, pose2);  
  return total_matches;
}

cv::Mat
Slam::calc_pose_with_matches(cv::Mat image, std::vector<Match> matches) {
  std::chrono::high_resolution_clock::time_point st = get_chrono();
  std::vector<cv::KeyPoint> old_keypoints;
  std::vector<SingleMatchPoint> matched_points1;
  std::vector<SingleMatchPoint> matched_points2;
  cv::vector<cv::Point2f> img_points_vector;
  cv::vector<cv::Point3f> obj_points_vector;
  cv::Mat pose;
  cv::Mat old_descriptors = cv::Mat(0, 64, CV_32F);
  std::vector<cv::DMatch> new_matches;
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;
  
  std::chrono::high_resolution_clock::time_point st1 = get_chrono();
  keypoints = search_features(image);
  measure_chrono(st1, "-> features  time: ");
  std::chrono::high_resolution_clock::time_point st2 = get_chrono();
  descriptors = extract_descriptors(image, keypoints);
  measure_chrono(st2, "-> descriptors  time: ");
  
  for(auto &m : matches) {
    old_descriptors.push_back(m.descriptor1);
    old_keypoints.push_back(m.keypoint1);
  }
  //std::cout << "New data: " << keypoints.size() << " " << descriptors.size() << " " << old_keypoints.size() << " " << old_descriptors.size() << std::endl; 
  new_matches = match_features(old_keypoints, old_descriptors, keypoints, descriptors);
  //std::cout << "Total pose matches " << new_matches.size() << std::endl;
  split_matches(new_matches, old_keypoints, old_descriptors, keypoints, descriptors, matched_points1, matched_points2);
  std::cout << "Matchs size: " << matched_points1.size() << " " << matched_points2.size() << std::endl;
  
  for (int k = 0; k < matched_points2.size(); k++) {
    Match closest_match;
    bool close_enough = find_closest_match(matched_points2[k], matches, closest_match);
    if (!close_enough) { continue; }
    img_points_vector.push_back(matched_points2[k].keypoint.pt);
    obj_points_vector.push_back(closest_match.match_point_in_marker_frame[0]);
  }
  std::cout << "obj_points: " << obj_points_vector.size() << std::endl;
  if (img_points_vector.size() != obj_points_vector.size() || obj_points_vector.size() < 6) {
    std::cout << "Return empty - " << obj_points_vector.size() << " - " << img_points_vector.size() << std::endl;
    return cv::Mat(4, 4, CV_64F, double(0));
  }
  // pose = estimated_pose(final_mathes);
  pose = estimated_pose(img_points_vector, obj_points_vector);
  measure_chrono(st, "-> calc_pose_with_matches time: ");
  return pose;
}

/* find_closest_match */
bool 
Slam::find_closest_match(SingleMatchPoint point_to_search, 
                         std::vector<Match> reference_matches, 
                         Match& closest_match) 
{
  std::vector<uint> point_to_search_desc;
  std::vector<int> eq_val_vec; 
  int min_index = 0;
  int max_eq_val = 0;

  for (int i = 0; i < point_to_search.descriptor.cols; i++) {
    point_to_search_desc.push_back((uint)point_to_search.descriptor.at<uchar>(0,i));
  } 
  for (auto &ref_m : reference_matches) {
    std::vector<uint> ref_m_desc;
    for (int i = 0; i < ref_m.descriptor1.cols; i++) {
      ref_m_desc.push_back((uint)ref_m.descriptor1.at<uchar>(0,i));
    }
    int eq_val = 0;
    for (int k = 0; k < ref_m_desc.size(); k++) {
      if (ref_m_desc[k] <= point_to_search_desc[k] + 2 && ref_m_desc[k] >= point_to_search_desc[k] - 2) {
        eq_val++;
      }
    }
    eq_val_vec.push_back(eq_val);
  }
  for (int i = 0; i < eq_val_vec.size(); i++) {
    if (eq_val_vec[i] > max_eq_val) {
      min_index = i;
      max_eq_val = eq_val_vec[i];
    }
  }
  closest_match = reference_matches[min_index];
  if (max_eq_val < 20) {
    return false;
  }
  return true;  
}

std::vector<cv::KeyPoint> 
Slam::search_features(cv::Mat image) {
  std::vector<cv::KeyPoint> keypoints;
  features_detector->detect(image, keypoints);
  return keypoints;
} 

cv::Mat 
Slam::extract_descriptors(cv::Mat image, std::vector<cv::KeyPoint> keypoints) {
  cv::Mat descriptors;
  descriptions_extractor->compute(image, keypoints, descriptors);
  return descriptors;
}

std::vector<cv::DMatch> 
Slam::match_features(std::vector<cv::KeyPoint> keypoints1, 
                     cv::Mat descriptors1, 
                     std::vector<cv::KeyPoint> keypoints2, 
                     cv::Mat descriptors2) 
{
  //std::vector<cv::DMatch> tot_matches = robust_matcher.match(keypoints1, descriptors1, keypoints2, descriptors2);
  std::vector<std::vector<cv::DMatch>> matches;
  cv::BFMatcher matcher = cv::BFMatcher(cv::NORM_HAMMING, true);
  matcher.knnMatch(descriptors1, descriptors2, matches, 1);
  
  std::vector<cv::DMatch> tot_matches;
  size_t t_size = 0;
  for (int i = 0; i < matches.size(); i++) { t_size += matches[i].size(); }
  tot_matches.reserve(t_size);
  for (int i = 0; i < matches.size(); i++) {
    tot_matches.insert(tot_matches.end(), matches[i].begin(), matches[i].end());
  }
  return tot_matches;  
}

void 
Slam::split_matches(std::vector<cv::DMatch> matches, 
                    std::vector<cv::KeyPoint> keypoints1, 
                    cv::Mat descriptors1, 
                    std::vector<cv::KeyPoint> keypoints2, 
                    cv::Mat descriptors2, 
                    std::vector<SingleMatchPoint>& match1, 
                    std::vector<SingleMatchPoint>& match2) 
{
  for (auto &gm : matches) {
    int idx1 = gm.queryIdx;
    int idx2 = gm.trainIdx;
    SingleMatchPoint p1, p2;
    p1.keypoint = keypoints1[idx1];
    p2.keypoint = keypoints2[idx2];
    p1.descriptor = descriptors1.row(idx1);
    p2.descriptor = descriptors2.row(idx2);
    match1.push_back(p1);
    match2.push_back(p2);
  }
}

std::vector<Match> 
Slam::triangulate(cv::Mat image1, 
                  std::vector<SingleMatchPoint> matched_points1, 
                  cv::Mat pose1, 
                  std::vector<SingleMatchPoint> matched_points2, 
                  cv::Mat pose2) 
{
  std::vector<Match> total_matches;
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
    std::vector<double> solution = solve_linear_system(p11, p12, p21, p22);
    if (solution[0] < -9999998) { continue; } 
    // Compund match
    Match match            = Match();
    match.coords3d         = cv::Point3d(solution[0], solution[1], solution[2]);
    match.coords3d_2       = cv::Point3d(solution[3], solution[4], solution[5]);
    match.coords2d         = matched_points1[gm].keypoint.pt;
    match.coords2d_2       = matched_points2[gm].keypoint.pt;
    match.keypoint1        = matched_points1[gm].keypoint;
    match.keypoint2        = matched_points2[gm].keypoint;
    match.descriptor1      = matched_points1[gm].descriptor;
    match.descriptor2      = matched_points2[gm].descriptor;
    match.reference_pose   = pose1;
    match.reference_pose_2 = pose2;
    match.match_position_in_marker_frame(pose1.inv());
    match.colour = image1.at<cv::Vec3b>(matched_points1[gm].keypoint.pt);
    total_matches.push_back(match);
  }
  return total_matches;
}

cv::Mat 
Slam::estimated_pose(std::vector<Match> matches) {
  cv::Mat pose;
  cv::Mat rVec, tVec, rot_mat;
  cv::vector<cv::Point2f> img_points_vector;
  cv::vector<cv::Point3f> obj_points_vector;
  for (auto &m : matches) {
    obj_points_vector.push_back(m.match_point_in_marker_frame[0]);
    img_points_vector.push_back(m.coords2d);
  }
  cv::solvePnP(obj_points_vector, img_points_vector, camera.getCameraMatrix(), camera.getDistorsion(), rVec, tVec, false, CV_ITERATIVE);
  cv::Rodrigues(rVec, rot_mat);
  pose = rot_tr_mat(rot_mat, tVec);
  return pose;
}

cv::Mat 
Slam::estimated_pose(cv::vector<cv::Point2f> img_points_vector, cv::vector<cv::Point3f> obj_points_vector) {
  cv::Mat pose;
  cv::Mat rVec, tVec, rot_mat;
  cv::solvePnP(obj_points_vector, img_points_vector, camera.getCameraMatrix(), camera.getDistorsion(), rVec, tVec, false, CV_ITERATIVE);
  cv::Rodrigues(rVec, rot_mat);
  pose = rot_tr_mat(rot_mat, tVec);
  return pose;
}

void 
Slam::draw_estimated_map(std::vector<Match> matches, cv::Mat& image, cv::Mat tr_vec, cv::Mat rot_vec) {
  for (auto &m : matches) {
    std::vector<cv::Point2f> p = m.calc_image_point_for_frame(camera, rot_vec, tr_vec);
    cv::circle(image, p[0], 5, cv::Scalar(0, 255, 255), 2);
    cv::circle(image, m.coords2d, 8, cv::Scalar(255, 0, 0), 2);
    std::ostringstream ss;
    ss << m.match_point_in_marker_frame[0].x;
    ss << " ";
    ss << m.match_point_in_marker_frame[0].y;
    ss << " ";
    ss << m.match_point_in_marker_frame[0].z;
    std::string s(ss.str());
    cv::putText(image, s, p[0], cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 255));
  }
}

void
Slam::refine_old_matches() {
  
}

/*  A x = B  */
std::vector<double> 
Slam::solve_linear_system(cv::Point3f p11, cv::Point3f p12, cv::Point3f p21, cv::Point3f p22) {
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
    Eigen::VectorXd x = MA.colPivHouseholderQr().solve(MB);
    double x1 = p11.x + x(0,0) * (p12.x - p11.x);
    double y1 = p11.y + x(0,0) * (p12.y - p11.y);
    double z1 = p11.z + x(0,0) * (p12.z - p11.z);
    double x2 = p21.x + x(1,0) * (p22.x - p21.x);
    double y2 = p21.y + x(1,0) * (p22.y - p21.y);
    double z2 = p21.z + x(1,0) * (p22.z - p21.z);
    std::vector<double> sol = {x1,y1,z1,x2,y2,z2};
    double relative_error = (MA * x - MB).norm() / MB.norm(); // norm() is L2 norm
    if (relative_error > 0.0005) {
      sol[0] = -9999999;
      return sol;
    }
    //std::cout << "The relative error is:\n" << relative_error << std::endl;
    return sol;
}

std::vector<cv::DMatch> 
Slam::match_with_features(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptor1, std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptor2) {
  /*-- cv::FlannBasedMatcher matcher; --*/
  cv::BFMatcher matcher(cv::NORM_L2);  
  std::vector<cv::DMatch> matches;
  matcher.match(descriptor1, descriptor2, matches);
  
  double max_dist = 0; double min_dist = 1;

  //-- Quick calculation of max and min distances between keypoints
  for(int i = 0; i < descriptor1.rows; i++) { 
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  std::vector<cv::DMatch> good_matches;
  if (matches.size() != descriptor1.rows) {
    return matches;
  }
  for (int i = 0; i < descriptor1.rows; i++) {
    if (matches[i].distance <= cv::max(2 * min_dist, 0.02)) { 
      good_matches.push_back(matches[i]); 
    }
  }
  return good_matches;
}

/*for (int i = 0; i < matched_points2.size(); i++) {
  Match ref_match;
  bool already_match = find_closest_match(matched_points2[i], old_matches, ref_match);
  if (already_match) {
    matched_points2.erase(matched_points2.begin() + i);
    matched_points1.erase(matched_points1.begin() + i);
  }
}*/