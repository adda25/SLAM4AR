//
//  Created by Amedeo on 13/07/15.
//  Copyright (c) 2015 Amedeo. All rights reserved.
//

#ifndef __CAMERA_PARAM_READER_HPP__
#define __CAMERA_PARAM_READER_HPP__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>

class CameraSystem
{
public:
    CameraSystem() {};
    ~CameraSystem() {};

    void set_from_yaml(std::string filePath);

    cv::Mat cam_point_from_pixel(cv::Point pixel_xy, double z);

    cv::Mat  camera_matrix;
    cv::Mat  distorsion;
    cv::Size cam_size;
    int x_center = 0;
    int y_center = 0;
    int xy_focal = 0;
private:
    
};


class MyCamereParamReader
{
public:
    MyCamereParamReader() {}
    ~MyCamereParamReader() {};
    
    void readFromXMLFile(std::string filePath);
    
    cv::Mat getCameraMatrix() {
        return cameraMatrix;
    }
    
    cv::Mat getDistorsion() {
        return distorsion;
    }
    
    cv::Size getCamSize() {
        return camSize;
    }
    
    cv::Mat  cameraMatrix;
    cv::Mat  distorsion;
    cv::Size camSize;
private:
    
};



#endif /* End __CAMERA_PARAM_READER_HPP__ */
