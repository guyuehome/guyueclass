#pragma once
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp> 
#include "Ellipse.h"
// #include <opencv2/highgui/highgui.hpp>

using namespace cv;
using std::pair;

class EllipseDetector
{
public:
	EllipseDetector(void);
	~EllipseDetector(void);
	void processFrame(const Mat& _frame);

private:
	bool findEllipses(const Mat& _frame, vector<EllipseN>& _detectedEllipses);
	void findEllipseContours(const Mat& _imgThreshold, vector<vector<Point>>& _contours, int _minContourPointsAllowed);
	void findEllipseCandidates(const vector<vector<Point>>& _contours, vector<EllipseN>& _detectedEllipses);
	void sortEllipses( vector<EllipseN>& _detectedEllipses);
	void rankX(vector<EllipseN>& _detectedE);
	void rankY(vector<EllipseN>& _detectedE);
	void estimatePosition(vector<EllipseN>& _detectedEllipses);
	void color_filter(Mat& _imgHsv);
private:
	float m_minContourLengthAllowed;	
	vector<Point2f> m_ellipseCorners2d;	
	vector<Point3f> m_ellipseCorners3d;	
	Mat m_camMat;
	Mat m_distCoeff;
public:
	//Mat m_imgGray;
	//Mat m_imgThreshold;
	Mat m_imgHsv;
	Mat m_imgCanny;
	vector<vector<Point>> m_contours;
	vector< Point > hull;
	double t_fx,t_fy,t_cx,t_cy;
	size_t pairNum;
	//FILE *fp;
public:
	vector<EllipseN> m_ellipses;
	vector<EllipseN> m_ellipsesRight;
	vector<EllipseN> m_ellipsesLeft;
};
