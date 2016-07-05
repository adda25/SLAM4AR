//     _    ____ _____           _
//    / \  |  _ \_   _|__   ___ | |
//   / _ \ | |_) || |/ _ \ / _ \| |
//  / ___ \|  _ < | | (_) | (_) | |
// /_/   \_\_| \_\|_|\___/ \___/|_|
// Augmented Reality for Manufacturing
//
//  cameraParamReader.h
//  testMarker
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
    
private:
    cv::Mat  cameraMatrix;
    cv::Mat  distorsion;
    cv::Size camSize;
};



#endif /* End __CAMERA_PARAM_READER_HPP__ */
