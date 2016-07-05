//     _    ____ _____           _
//    / \  |  _ \_   _|__   ___ | |
//   / _ \ | |_) || |/ _ \ / _ \| |
//  / ___ \|  _ < | | (_) | (_) | |
// /_/   \_\_| \_\|_|\___/ \___/|_|
// Augmented Reality for Manufacturing
//
//  cameraParamReader.cpp
//  testMarker
//
//  Created by Amedeo on 13/07/15.
//  Copyright (c) 2015 Amedeo. All rights reserved.
//

#include "camera_param_reader.hpp"

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