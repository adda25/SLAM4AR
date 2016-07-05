//
//  marker.cpp
//  testMarker
//
//  Created by Amedeo on 16/12/15.
//  Copyright (c) 2015 Amedeo. All rights reserved.
//

#include "marker.hpp"
/*
    Aruco Original Functions
*/
int hammDistMarker(cv::Mat  bits);
int analyzeMarkerImage(cv::Mat &grey,int &nRotations);
cv::Mat rotate(const cv::Mat  &in);
int hammDistMarker(cv::Mat  bits);

Marker
MyMarkerDetector::detectMarker(cv::Mat& image)
{
    frame = image;
    preProcessing();
    findContours(30);
    findCandidates();
    detectAndDecode();
	  image = frame; 
    return markersToReturn;
}

void
MyMarkerDetector::findContours(int minContourPointsAllowed)
{
    static std::vector< std::vector < cv::Point > > allContours;
    unsigned long contourSize;
    
    allContours.reserve(1000);
    cv::Mat thr_frame_copy;
    thr_frame.copyTo(thr_frame_copy);
    cv::findContours(thr_frame_copy, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    contours.clear();
    for (size_t i = 0; i < allContours.size(); i++) {
        contourSize = allContours[i].size();
        if (contourSize > minContourPointsAllowed) {
            contours.push_back( allContours[i] );
        }
    }
    allContours.clear();
}

void
MyMarkerDetector::findCandidates()
{
    static std::vector < cv::Point > approxCurve;
    cv::Point side;
    
    _marker.clear(); // delete
    approxCurve.reserve(12);
    _marker.reserve(12);
    for (size_t i = 0; i < contours.size(); i++) {
        cv::approxPolyDP(
                         contours[i],
                         approxCurve,
                         cv::arcLength(cv::Mat(contours[i]), true) * 0.02,
                         true); // 0.02 DEF
        if (approxCurve.size() == 4 && cv::isContourConvex(approxCurve)) {
            if (verifyDistance(approxCurve[0], approxCurve[2])) {
                sortVertices( approxCurve );
                _marker.push_back(approxCurve);
            }
        }
        approxCurve.clear();
    }
}

inline bool
MyMarkerDetector::verifyDistance(cv::Point p_o, cv::Point p_t)
{
    const int min_dist = 10; // DEF
    ty_f1 dx = p_o.x - p_t.x;
    ty_f1 dy = p_o.y - p_t.y;
    ty_f1 dist = sqrtf(powf(dx, 2) + powf(dy, 2));
    if (dist > min_dist)
        return true;
    return false;
}

inline void
MyMarkerDetector::sortVertices(std::vector < cv::Point >& approxCurve)
{
	std::vector < cv::Point > approxCurveBuffer;
    cv::Point v1 = approxCurve[1] - approxCurve[0];
    cv::Point v2 = approxCurve[2] - approxCurve[0];
    double o = (v1.x * v2.y) - (v1.y * v2.x);
    if (o < 0.0) {
        std::swap(approxCurve[1], approxCurve[3]);
    }
}

void
MyMarkerDetector::detectAndDecode()
{
	markersToReturn.marker.clear();
    for (int i = 0; i < _marker.size(); i++){
        getWarpPerspective(i);
        thresholdAfterWarp();
        int r = 0;
        int idM = analyzeMarkerImage(marker_roi, r);
        if (idM != -1) {
            std::vector <cv::Point2f> corners;
            convertIntegerPointToFloatPoint(i, corners);
            rotateCorners(corners, r);
            refineSubPix(corners);
            
		   /* for (int i = 0; i < 3; i++) {
		    	cv::line(frame, corners[i], corners[i + 1], cv::Scalar(0, 0, 255, 1), 3);
		    }*/
			//cv::line(frame, corners[3], corners[0], cv::Scalar(0, 0, 255, 1), 3);
			float side = sqrt(pow(corners[0].x - corners[1].x, 2) + pow(corners[0].y - corners[1].y, 2));
		    // Push back marker 
		    MarkerData marker;
			marker.markerId = idM;
			marker.pixelMMratio = realMarkerSize / side;
      marker.markerSize = realMarkerSize;
			marker.corners = corners;
      cv::Mat rotVec;
      cv::Mat trVec;
      getMarkerPose(marker, trVec, rotVec);
      marker.trVec = trVec;
      marker.rotVec = rotVec;
			markersToReturn.marker.push_back(marker);
      }
    }
}

inline void 
MyMarkerDetector::getMarkerPose(MarkerData& m, cv::Mat& trVec, cv::Mat& rotVec) {
    cv::Mat raux, taux;
    cv::Mat rVecTest, tVecTest;
    cv::vector<cv::Point2f> imgPointsVector;
    cv::vector<cv::Point3f> objPointsVector;
    ty_d1 halfSize = m.markerSize * 0.5;
    bool find = false;
    
    objPointsVector.push_back(cv::Point3f(-halfSize, -halfSize,  0));
    objPointsVector.push_back(cv::Point3f(-halfSize,  halfSize,  0));
    objPointsVector.push_back(cv::Point3f( halfSize,  halfSize,  0));
    objPointsVector.push_back(cv::Point3f( halfSize, -halfSize,  0));
  
    for (int c = 0; c < 4; c++)
        imgPointsVector.push_back(m.corners[c]);
    
    cv::solvePnPRansac(objPointsVector,
                 imgPointsVector,
                 camera.getCameraMatrix(),
                 camera.getDistorsion(),
                 raux,
                 taux,
                 false,
                 CV_P3P);
                 
    raux.convertTo(rVecTest, CV_64FC1);
    taux.convertTo(tVecTest, CV_64FC1);

    raux.convertTo(rotVec, CV_64FC1);
    taux.convertTo(trVec, CV_64FC1);        
}


inline void
MyMarkerDetector::rotateCorners(std::vector < cv::Point2f >& corners, int r)
{
    std::vector < cv::Point2f > corners_buffer = corners;
    switch (r) {
        case 0:
            corners[0] = corners_buffer[3];
            corners[1] = corners_buffer[0];
            corners[2] = corners_buffer[1];
            corners[3] = corners_buffer[2];
            break;
        case 1:
            corners[0] = corners_buffer[2];
            corners[1] = corners_buffer[3];
            corners[2] = corners_buffer[0];
            corners[3] = corners_buffer[1];
            break;
        case 2:
            corners[0] = corners_buffer[1];
            corners[1] = corners_buffer[2];
            corners[2] = corners_buffer[3];
            corners[3] = corners_buffer[0];
            break;
        case 3:
            break;
    }
}

inline void
MyMarkerDetector::getWarpPerspective(int i)
{
    cv::Point2f pointsRes[4], pointsIn[4];
    for ( int j = 0; j < 4; j++ ){
        pointsIn[j]=_marker[i][j];
    }
    pointsRes[0] = cv::Point2f(0, 0) ;
    pointsRes[1] = cv::Point2f(markerSize-1, 0);
    pointsRes[2] = cv::Point2f(markerSize-1, markerSize-1);
    pointsRes[3] = cv::Point2f(0, markerSize-1);
    cv::Mat prsx_trnsf = cv::getPerspectiveTransform(pointsIn, pointsRes);
    cv::warpPerspective (gray_frame, marker_roi, prsx_trnsf, cv::Size(markerSize, markerSize));
}

/* Aruco Original Functions
 */
using namespace cv;
int analyzeMarkerImage(Mat &grey,int &nRotations)
{
    //Markers  are divided in 7x7 regions, of which the inner 5x5 belongs to marker info
    //the external border shoould be entirely black
    int swidth=grey.rows/7;
    for (int y=0;y<7;y++)
    {
        int inc=6;
        if (y==0 || y==6) inc=1;//for first and last row, check the whole border
        for (int x=0;x<7;x+=inc)
        {
            int Xstart=(x)*(swidth);
            int Ystart=(y)*(swidth);
            Mat square=grey(cv::Rect(Xstart,Ystart,swidth,swidth));
            int nZ=countNonZero(square);
            if ( nZ > (swidth*swidth) /2) {
                // 		cout<<"neb"<<endl;
                return -1;//can not be a marker because the border element is not black!
            }
        }
    }
    
    //now,
    vector<int> markerInfo(5);
    Mat _bits=Mat::zeros(5,5,CV_8UC1);
    //get information(for each inner square, determine if it is  black or white)
    
    for (int y=0;y<5;y++)
    {
        
        for (int x=0;x<5;x++)
        {
            int Xstart=(x+1)*(swidth);
            int Ystart=(y+1)*(swidth);
            Mat square=grey(cv::Rect(Xstart,Ystart,swidth,swidth));
            int nZ=countNonZero(square);
            if (nZ> (swidth*swidth) /2)  _bits.at<uchar>( y,x)=1;
        }
    }
    // 		printMat<uchar>( _bits,"or mat");
    
    //checkl all possible rotations
    Mat _bitsFlip;
    Mat Rotations[4];
    Rotations[0]=_bits;
    int dists[4];
    dists[0]=hammDistMarker( Rotations[0]) ;
    std::pair<int,int> minDist( dists[0],0);
    for (int i=1;i<4;i++)
    {
        //rotate
        Rotations[i] = rotate(Rotations[i-1]);
        //get the hamming distance to the nearest possible word
        dists[i]=hammDistMarker( Rotations[i]) ;
        if (dists[i]<minDist.first)
        {
            minDist.first=  dists[i];
            minDist.second=i;
        }
    }
    // 		        printMat<uchar>( Rotations [ minDist.second]);
    // 		 	cout<<"MinDist="<<minDist.first<<" "<<minDist.second<<endl;
    
    nRotations=minDist.second;
    if (minDist.first!=0)	 //FUTURE WORK: correct if any error
        return -1;
    else {//Get id of the marker
        int MatID=0;
        cv::Mat bits=Rotations [ minDist.second];
        for (int y=0;y<5;y++)
        {
            MatID<<=1;
            if ( bits.at<uchar>(y,1)) MatID|=1;
            MatID<<=1;
            if ( bits.at<uchar>(y,3)) MatID|=1;
        }
        return MatID;
    }
}

Mat rotate(const Mat  &in)
{
    Mat out;
    in.copyTo(out);
    for (int i=0;i<in.rows;i++)
    {
        for (int j=0;j<in.cols;j++)
        {
            out.at<uchar>(i,j)=in.at<uchar>(in.cols-j-1,i);
        }
    }
    return out;
}

int hammDistMarker(Mat  bits)
{
    int ids[4][5]=
    {
        {
            1,0,0,0,0
        }
        ,
        {
            1,0,1,1,1
        }
        ,
        {
            0,1,0,0,1
        }
        ,
        {
            0, 1, 1, 1, 0
        }
    };
    int dist=0;
    
    for (int y=0;y<5;y++)
    {
        int minSum=1e5;
        //hamming distance to each possible word
        for (int p=0;p<4;p++)
        {
            int sum=0;
            //now, count
            for (int x=0;x<5;x++)
                sum+=  bits.at<uchar>(y,x) == ids[p][x]?0:1;
            if (minSum>sum) minSum=sum;
        }
        //do the and
        dist+=minSum;
    }
    
    return dist;
}

bool correctHammMarker(Mat &bits)
{
    //detect this lines with errors
    bool errors[4];
    int ids[4][5]=
    {
        {
            0,0,0,0,0
        }
        ,
        {
            0,0,1,1,1
        }
        ,
        {
            1,1,0,0,1
        }
        ,
        {
            1, 1, 1, 1, 0
        }
    };
    
    for (int y=0;y<5;y++)
    {
        int minSum=1e5;
        //hamming distance to each possible word
        for (int p=0;p<4;p++)
        {
            int sum=0;
            //now, count
            for (int x=0;x<5;x++)
                sum+=  bits.at<uchar>(y,x) == ids[p][x]?0:1;
            if (minSum>sum) minSum=sum;
        }
        if (minSum!=0) errors[y]=true;
        else errors[y]=false;
    }
    
    return true;
}
