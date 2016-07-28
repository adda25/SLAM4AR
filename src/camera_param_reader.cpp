//
//  Created by Amedeo on 13/07/15.
//  Copyright (c) 2015 Amedeo. All rights reserved.
//

#include "camera_param_reader.hpp"

void
CameraSystem::set_from_yaml(std::string filePath) {
  cv::FileStorage fs (filePath, cv::FileStorage::READ);
  cv::Mat MCamera,MDist;
  int w = -1, h = -1;
  fs["image_width"] >> w;
  fs["image_height"] >> h;
  fs["distortion_coefficients"] >> MDist;
  fs["camera_matrix"] >> MCamera;
  if (MCamera.cols==0 || MCamera.rows==0) throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera matrix","CameraParameters::readFromXML",__FILE__,__LINE__);
  if (w == -1 || h == 0) throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera dimensions","CameraParameters::readFromXML",__FILE__,__LINE__); 
  if (MCamera.type()!=CV_32FC1) MCamera.convertTo(camera_matrix, CV_32FC1);
  else camera_matrix = MCamera;
  if (MDist.total() < 4) throw cv::Exception(9007,"File :"+filePath+" does not contains valid distortion_coefficients","CameraParameters::readFromXML",__FILE__,__LINE__);  
  cv::Mat mdist32;
  MDist.convertTo(mdist32,CV_32FC1);
  distorsion.create(1,5,CV_32FC1);
  for (int i=0;i<5;i++)
      distorsion.ptr<float>(0)[i] = mdist32.ptr<float>(0)[i];
  cam_size.width = w;
  cam_size.height = h;
  x_center = (int)cam_size.width  * 0.5;
  y_center = (int)cam_size.height * 0.5;
  xy_focal = camera_matrix.template at <float>(0,0);
}

cv::Mat 
CameraSystem::cam_point_from_pixel(cv::Point pixel_xy, double z)
{
  cv::Mat result(4, 1, CV_64FC1);
  result.at<double>(0,0) = (pixel_xy.x - x_center) * z / xy_focal;
  result.at<double>(1,0) = (pixel_xy.y - y_center) * z / xy_focal;
  result.at<double>(2,0) = z;
  result.at<double>(3,0) = 1;
  return result;
}




void MyCamereParamReader::readFromXMLFile (std::string filePath) {
    cv::FileStorage fs (filePath, cv::FileStorage::READ);
    cv::Mat MCamera,MDist;
    int w = -1, h = -1;
    
    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["distortion_coefficients"] >> MDist;
    fs["camera_matrix"] >> MCamera;
    
    if (MCamera.cols==0 || MCamera.rows==0) throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera matrix","CameraParameters::readFromXML",__FILE__,__LINE__);
    if (w == -1 || h == 0) throw cv::Exception(9007,"File :"+filePath+" does not contains valid camera dimensions","CameraParameters::readFromXML",__FILE__,__LINE__);
    
    if (MCamera.type()!=CV_32FC1) MCamera.convertTo( cameraMatrix, CV_32FC1 );
    else cameraMatrix = MCamera;
    
    if (MDist.total() < 4) throw cv::Exception(9007,"File :"+filePath+" does not contains valid distortion_coefficients","CameraParameters::readFromXML",__FILE__,__LINE__);

    cv::Mat mdist32;
    MDist.convertTo(mdist32,CV_32FC1);
    distorsion.create(1,5,CV_32FC1);
    for (int i=0;i<5;i++)
        distorsion.ptr<float>(0)[i] = mdist32.ptr<float>(0)[i];
    camSize.width = w;
    camSize.height = h;
}