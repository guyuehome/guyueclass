#pragma once
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp> 
// #include <opencv2/highgui/highgui.hpp>
//#include "Ellipse.h"
using namespace cv;

class EllipseN
{
public:
	EllipseN(void);
	~EllipseN(void);	

	CvBox2D Box;
	vector<Point2f> m_points;//椭圆图像四个点
	vector<Point2f> m_rec;//旋转矩形
	Matx33f m_rotation;
	Vec3f m_translation;
	
};