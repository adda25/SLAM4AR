#ifndef __ROBUST_MATCHER_HPP__
#define __ROBUST_MATCHER_HPP__

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

class RobustMatcher 
{
public:
  RobustMatcher(double confidence = 0.99, double distance = 1.0, float ratio = 0.65) {
    this->confidence = confidence;
    this->distance = distance;
    this->ratio = ratio;
    refine_foundamental = false;
  }
  
  std::vector<cv::DMatch> match(std::vector<cv::KeyPoint> keypoints1, cv::Mat descriptors1, std::vector<cv::KeyPoint> keypoints2, cv::Mat descriptors2);
  
  double confidence;
  double distance;
  float  ratio;
  bool   refine_foundamental;
  
private:
  int ratio_test(std::vector<std::vector<cv::DMatch>> &matches);
  void symmetry_test(const std::vector<std::vector<cv::DMatch>>& matches1,
                     const std::vector<std::vector<cv::DMatch>>& matches2,
                     std::vector<cv::DMatch>& symMatches);
  cv::Mat ransac_test(const std::vector<cv::DMatch>& matches,
                      const std::vector<cv::KeyPoint>& keypoints1,
                      const std::vector<cv::KeyPoint>& keypoints2,
                      std::vector<cv::DMatch>& outMatches);         
};

#endif // End __ROBUST_MATCHER_HPP__