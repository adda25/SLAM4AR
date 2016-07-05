//
//  marker.hpp
//  testMarker
//
//  Created by Amedeo on 16/12/15.
//  Copyright (c) 2015 Amedeo. All rights reserved.
//

#ifndef __testMarker__markerDetector__
#define __testMarker__markerDetector__

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include "camera_param_reader.hpp"

#ifndef PRE_PROC_ADAPTIVE
#define PRE_PROC_ADAPTIVE  0x1
#define OTSU_THRESH 130
#define ADAPT_SIZE 201
#define ADAPT_C -3
#endif
#ifndef PRE_PROC_BLUR
#define PRE_PROC_BLUR  0x0
#endif

typedef float ty_f1;
typedef float ty_d1;

typedef struct
{
	int markerId;
	float pixelMMratio;
	std::vector<cv::Point2f> corners;
  cv::Mat trVec;
  cv::Mat rotVec;
  float markerSize;
  
  cv::Mat pose() {
    cv::Mat rot_mat(3, 3, CV_64FC1);
    cv::Mat result(4, 4, CV_64FC1);
    cv::Rodrigues(rotVec, rot_mat);
    for (int i = 0; i < 3; i++) {
      for (int k = 0; k < 3; k++) {
        result.at<double>(i,k) = rot_mat.at<double>(i,k);
      }
    }
    result.at<double>(0,3) = trVec.at<double>(0,0);
    result.at<double>(1,3) = trVec.at<double>(1,0);
    result.at<double>(2,3) = trVec.at<double>(2,0);
    result.at<double>(3,0) = 0;
    result.at<double>(3,1) = 0;
    result.at<double>(3,2) = 0;
    result.at<double>(3,3) = 1;
    return result;
  }
  
} MarkerData;

typedef struct
{
	std::vector <MarkerData> marker;
} Marker;

class MyMarkerDetector 
{
public:
    MyMarkerDetector(float realMarkerSize, std::string camera_calib) {
      camera = MyCamereParamReader();
      camera.readFromXMLFile(camera_calib);
    	setMarkerSize(realMarkerSize);
	    markerSize = 56;
	    this->realMarkerSize = realMarkerSize;
	    _TEMP_N_MARKER = 1;
    }
    ~MyMarkerDetector() {};
    
    // Funzione che dirige le operazioni necessarie per la
    // ricerca dei marker
    Marker detectMarker(cv::Mat& image);
            
    static int compare (const void * a, const void * b)
    {
        return ( *(int*)a - *(int*)b );
    }

    void setMarkerSize(float markerSize)
    {
        realMarkerSize = markerSize;
    }
    
    MyCamereParamReader camera;
private:
    std::vector <std::vector<cv::Point> > contours;
    std::vector <std::vector<cv::Point> > _marker; // Marker candidate
    cv::Mat gray_frame;
    cv::Mat frame;
    cv::Mat thr_frame;
    cv::Mat marker_roi;
    ty_f1 markerSize; // Da incrementare a 80? 8xN N->0,1,2..
    ty_f1 realMarkerSize; // Viene cambiato da server
	  Marker markersToReturn;
    int _TEMP_N_MARKER;
    
    // Segmento l'immagine usando l'algoritmo di Otsu,
    // più veloce del threshold adattivo
    void preProcessing()
    {
        cv::cvtColor(frame, gray_frame, CV_RGBA2GRAY);
        if (PRE_PROC_BLUR)
            preProcessingBlur();
        PRE_PROC_ADAPTIVE ? preProcessingAdaptiveThreshold() : preProcessingThresholdOtsu();
    }
    
    void preProcessingBlur()
    {
        cv::Mat gray_filtered_frame;
        cv::blur (gray_frame, gray_filtered_frame, cv::Size(3,3));
        gray_frame = gray_filtered_frame;
    }
    
    void preProcessingThresholdOtsu()
    {
        cv::threshold(gray_frame, thr_frame, OTSU_THRESH, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
    }
    
    void preProcessingAdaptiveThreshold()
    {
        cv::adaptiveThreshold (gray_frame , thr_frame, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, ADAPT_SIZE, ADAPT_C);
    }
    
    // Questa funzione trova i contorni presenti nell'immagine
    // e salva solo i contorni formati da almeno un certo
    // numero di punti
    void findContours(int minContourPointsAllowed);
    
    // Tra tutti i contorni salvati cerco quelli che
    // formano un quadrato e sono convessi
    void findCandidates();
    
    // Verifico che i due vertici opposti del quadrato
    // non siano troppo vicini
    inline bool verifyDistance(cv::Point p_o, cv::Point p_t);
    
    // Ordina i vertici del quadrato in modo da avere sempre
    // lo stesso ordine, necessario in getWarpPerspective
    inline void sortVertices(std::vector<cv::Point>& approxCurve);
    
    // Una volta che ho trovato dei candidati
    // eseguo la trasformazione prospettica
    // e decodifico il codeice presente nel
    // marker candidato. Se è un marker,
    // procedo a calcolarne tutte le informazioni
    // possibili, utilizzando per salvare i dati
    // la classe MyMarker
    void detectAndDecode();
        
    // Ordina i vertici in base alle rotazioni necessarie ad identificare
    // il marker, ricevute da analyzeMarkerImage
    inline void rotateCorners(std::vector<cv::Point2f>& corners, int r);
    
    // Ottengo la trasformazione prospettica per il marker "i"
    // questa funzione sovrascrive la marker_roi
    inline void getWarpPerspective(int i);
    
    // Esegue il threshold della marker_roi
    inline void thresholdAfterWarp()
    {
        cv::adaptiveThreshold (
                               marker_roi, marker_roi,
                               255,
                               CV_ADAPTIVE_THRESH_MEAN_C,
                               CV_THRESH_BINARY_INV,
                               ADAPT_SIZE,
                               ADAPT_C
                               );
        // cv::threshold(marker_roi, marker_roi, 125, 255, CV_THRESH_BINARY || CV_THRESH_OTSU);
        cv::bitwise_not(marker_roi, marker_roi);
    }
    
    // Migliora la precisione dei corner trovati
    inline void refineSubPix(std::vector<cv::Point2f>& corners)
    {
        cv::cornerSubPix(gray_frame, corners, cvSize(9, 9), cvSize(-1, -1),
                         cvTermCriteria (CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 50, 0.001));
    }
    
    // Necessaria per passare i corner alla funzione refineSubPix
    inline void convertIntegerPointToFloatPoint(int i, std::vector<cv::Point2f> &corners)
    {
        for (int k = 0; k < 4; k++) {
            corners.push_back(_marker[i][k]);
        }
    }
    
    inline void getMarkerPose(MarkerData& m, cv::Mat& trVec, cv::Mat& rotVec);
};
#endif /* defined(__testMarker__markerDetector__) */
