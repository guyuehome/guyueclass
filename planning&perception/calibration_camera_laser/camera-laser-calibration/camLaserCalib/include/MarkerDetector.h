/************************************************************************
* Copyright(c) 2021  Z
* All rights reserved.
*
* File:	MarkerDetector.h
* Brief: 标签检测类
* Version: 1.0
* Author: Z
* Email: 
* Date: 2021/8/1 11:45
* History:
************************************************************************/
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Marker.h"

using namespace cv;
using std::pair;

class MarkerDetector
{
public:
	MarkerDetector(void);
	~MarkerDetector(void);
	void processFrame(const Mat& _frame);
	void getTransformations(void);
private:
	bool findMarkers(const Mat& _frame, vector<Marker>& _detectedMarkers);
// 	void performThreshold(const Mat& _imgGray, Mat& _thresholdImg);
    void findMarkerContours(const Mat& _imgThreshold, vector<vector<Point> >& _contours, int _minContourPointsAllowed);
    void findMarkerCandidates(const vector<vector<Point> >& _contours, vector<Marker>& _detectedMarkers);
	void detectMarkers(const Mat& _imgGray, vector<Marker>& _detectedMarkers);
	void estimatePosition(vector<Marker>& _detectedMarkers);
private:
	float m_minContourLengthAllowed;	// contour最小边长的阈值 = 100
	Size m_markerSize;
	vector<Point2f> m_markerCorners2d;	// marker 4个角点的正交投影标准值
	vector<Point3f> m_markerCorners3d;	
	Mat m_camMat;
	Mat m_distCoeff;
public:
	Mat m_imgGray;
	Mat m_imgThreshold;
    vector<vector<Point> > m_contours;
public:
	vector<Marker> m_markers;
};
