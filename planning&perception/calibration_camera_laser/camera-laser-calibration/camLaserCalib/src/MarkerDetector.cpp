#include "MarkerDetector.h"

MarkerDetector::MarkerDetector(void)
{
	m_minContourLengthAllowed = 100.0f;

	m_camMat = (Mat_<float>(3,3) << 0,0,0,
									0,0,0,
									0,0,0);
	m_distCoeff = (Mat_<float>(4,1) << 0,0,0,0);

	m_markerSize = Size(100, 100);
	// 默认marker的size为100*100,markercorner在2d空间为100*100的矩形
	m_markerCorners2d.push_back(Point2f(0, 0));
	m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, 0));
	m_markerCorners2d.push_back(Point2f(m_markerSize.width-1, m_markerSize.height-1));
	m_markerCorners2d.push_back(Point2f(0, m_markerSize.height-1));

	// 3d corner所在坐标为以marker中心为原点
	m_markerCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));
	m_markerCorners3d.push_back(cv::Point3f(+0.5f,-0.5f,0));
	m_markerCorners3d.push_back(cv::Point3f(+0.5f,+0.5f,0));
	m_markerCorners3d.push_back(cv::Point3f(-0.5f,+0.5f,0));
}

MarkerDetector::~MarkerDetector(void)
{
}

void MarkerDetector::processFrame(const Mat& _frame)
{
	m_markers.clear();
	findMarkers(_frame, m_markers);
}

void MarkerDetector::getTransformations(void)
{
}

bool MarkerDetector::findMarkers(const Mat& _frame, vector<Marker>& _detectedMarkers)
{
	// 转为灰度图
	cvtColor(_frame, m_imgGray, CV_BGR2GRAY);

	// 转为2值图,自适应的阈值,这个自适应阈值函数好像有问题啊。。。。！！！！！！
	threshold(m_imgGray, m_imgThreshold, 128, 255, cv::THRESH_BINARY_INV);
// 	imshow("threshold", m_imgThreshold);
// 	adaptiveThreshold(m_imgGray, m_imgThreshold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 7, 7);
// 	imshow("threshold", m_imgThreshold);

	// 检测边缘
	findMarkerContours(m_imgThreshold, m_contours, m_imgGray.cols/5);
// 	vector<Vec4i> hierarchy;
// 	Mat contourImg = Mat::zeros(_frame.size(), CV_8UC3);
// 	for(int i=0; i<m_contours.size(); i++)
// 	{
// 		drawContours(contourImg, m_contours, i, Scalar(255,255,255), 2, 8, hierarchy, 0, Point());
// 	}
// 	imshow("contours", contourImg);

	// 筛选contours，选择那些又4点围成的contour，得到候选marker
	findMarkerCandidates(m_contours, _detectedMarkers);
// 	Mat markerCandidateImg = Mat::zeros(_frame.size(), CV_8UC3);
// 	for(int i=0; i<_detectedMarkers.size(); i++)
// 	{
// 		int sizeNum = _detectedMarkers[i].m_points.size();
// 		for (int j=0; j<sizeNum; j++)
// 		{
// 			line(markerCandidateImg, _detectedMarkers[i].m_points[j], _detectedMarkers[i].m_points[(j+1)%sizeNum], Scalar(255,255,255), 2, 8);
// 		}
// 	}
// 	imshow("markerCandidate", markerCandidateImg);

	// 检测marker，根据marker的id信息筛选
	detectMarkers(m_imgGray, _detectedMarkers);
// 	Mat markerImg = Mat::zeros(_frame.size(), CV_8UC3);
// 	for(int i=0; i<_detectedMarkers.size(); i++)
// 	{
// 		int sizeNum = _detectedMarkers[i].m_points.size();
// 		for (int j=0; j<sizeNum; j++)
// 		{
// 			line(markerImg, _detectedMarkers[i].m_points[j], _detectedMarkers[i].m_points[(j+1)%sizeNum], Scalar(255,255,255), 2, 8);
// 		}
// 	}
// 	imshow("marker", markerImg);

	// 计算姿态
 	estimatePosition(_detectedMarkers);

	// 按照id排序,这里先不做这步@@@@@@@@@
// 	std::sort(_detectedMarkers.begin(), _detectedMarkers.end());
	return false;
}

// void MarkerDetector::performThreshold(const Mat& _imgGray, Mat& _thresholdImg)
// {
// }

void MarkerDetector::findMarkerContours(const Mat& _imgThreshold, vector<vector<Point> >& _contours, int _minContourPointsAllowed)
{
	Mat imgTemp = _imgThreshold.clone();

    vector<vector<Point> > allContours;
	findContours(imgTemp, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	// 做一个筛选,如果一个contour的点的个数比较少,不是一个好contour
	_contours.clear();
	for (size_t i=0; i<allContours.size(); i++)
	{
		int contourSize = allContours[i].size();
		if (contourSize > _minContourPointsAllowed)
		{
			_contours.push_back(allContours[i]);
		}
	}
}

void MarkerDetector::findMarkerCandidates(const vector<vector<Point> >& _contours, vector<Marker>& _detectedMarkers)
{
	vector<Point> approxCurve;
	vector<Marker> markerPossible;
	
	for (size_t i=0; i<_contours.size(); i++)
	{
		// 得到近似多边形
		approxPolyDP(_contours[i], approxCurve, double(_contours[i].size())*0.05, true);
		// 我们只关心那些4边形的轮廓，因为只有他们才可能是marker
		if (approxCurve.size() != 4)
			continue;
		// 并且是凸4边形
		if (!isContourConvex(approxCurve))
			continue;
		// 找个4个边最小的边
		float minDist = FLT_MAX;
		for (int i=0; i<4; i++)
		{
			Point vecTemp = approxCurve[i] - approxCurve[(i+1)%4];
			float distSquared = vecTemp.dot(vecTemp);
			minDist = std::min(minDist, distSquared);
		}
		// 如果这个最小边不够长，这个contour不是marker
		if (minDist > m_minContourLengthAllowed)
		{
			Marker markerTemp;
			for (int i=0; i<4; i++)
			{
				markerTemp.m_points.push_back(Point2f(approxCurve[i].x, approxCurve[i].y));
			}
			markerPossible.push_back(markerTemp);
		}
	}
	// 把得到的markerPossible按照逆时针排序
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		// 检测第三个点在一二点组成的线的左边还是右边
		Point v1 = markerPossible[i].m_points[1] - markerPossible[i].m_points[0];
		Point v2 = markerPossible[i].m_points[2] - markerPossible[i].m_points[0];
		double theta = (v1.x * v2.y) - (v1.y * v2.x);
		// 左边为逆时针，右边顺时针，变换2、4点顺序
		if (theta < 0.0)
		{
			std::swap(markerPossible[i].m_points[1], markerPossible[i].m_points[3]);
		}
	}
	// 找到corner非常接近的contour,放到tooNearCandidates
    vector<pair<int, int> > tooNearCandidates;
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		for (size_t j=i+1; j<markerPossible.size(); j++)
		{
			float distSquared = 0.0f;
			for (int k=0; k<4; k++)
			{
				Point vec = markerPossible[i].m_points[k] - markerPossible[j].m_points[k];
				distSquared += vec.dot(vec);
			}
			if (distSquared < 400)
			{
				tooNearCandidates.push_back(pair<int, int>(i,j));
			}
		}
	}
	// 选择tooNearCadidates中周长更小的作为移除的对象
	vector<bool> markerRemoveIndex(markerPossible.size(), false);
	for (size_t i=0; i<tooNearCandidates.size(); i++)
	{
		float length1 = markerPossible[tooNearCandidates[i].first].calPerimeter();
		float length2 = markerPossible[tooNearCandidates[i].second].calPerimeter();
		markerRemoveIndex[(length1>length2) ? tooNearCandidates[i].second : tooNearCandidates[i].first] = true;
	}
	// 去掉markerRemoveIndex得到最终的候选者
	_detectedMarkers.clear();
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		if (!markerRemoveIndex[i])
		{
			_detectedMarkers.push_back(markerPossible[i]);
		}
	}
}

void MarkerDetector::detectMarkers(const Mat& _imgGray, vector<Marker>& _detectedMarkers)
{
	Mat canonicalImg;	// 去除投影变换恢复的3维视图
	vector<Marker> goodMarkers;

	// 根据id，验证marker
	for (size_t i=0; i<_detectedMarkers.size(); i++)
	{
		// 得到当前marker的透视变换矩阵M
		Mat M = getPerspectiveTransform(_detectedMarkers[i].m_points, m_markerCorners2d);
		// 将当前的marker变换为正交投影
		warpPerspective(_imgGray, canonicalImg, M, m_markerSize);
// 		imshow(str, canonicalImg);
		// 解码marker
		int nRotations;
		int idMarker = Marker::decode(canonicalImg, nRotations);
		if (idMarker != -1)
		{
			_detectedMarkers[i].m_id = idMarker;
			// 将对应的corner point也进行旋转
			std::rotate(_detectedMarkers[i].m_points.begin(), _detectedMarkers[i].m_points.begin()+4-nRotations, _detectedMarkers[i].m_points.end());
			goodMarkers.push_back(_detectedMarkers[i]);
		}
	}

	// 用subpixel提高marker corner的精度
	if (goodMarkers.size() > 0)
	{
		// 把各corner整理为一个数组，因为cornerSubPix函数很慢，尽力不要多次调用
		vector<Point2f> preciseCorners(4*goodMarkers.size());

		for (size_t i=0; i<goodMarkers.size(); i++)
		{
			for (int j=0; j<4; j++)
			{
				preciseCorners[4*i+j] = goodMarkers[i].m_points[j];
			}
		}

		cornerSubPix(_imgGray, preciseCorners, Size(5,5), Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1));

		for (size_t i=0; i<goodMarkers.size(); i++)
		{
			for (int j=0; j<4; j++)
			{
				goodMarkers[i].m_points[j] = preciseCorners[4*i+j];
			}
		}
	}

	_detectedMarkers = goodMarkers;
}

void MarkerDetector::estimatePosition(vector<Marker>& _detectedMarkers)
{
	for (size_t i=0; i<_detectedMarkers.size(); i++)
	{
		Mat Rvec;
// 		Mat_<float> Tvec;
		Vec3f Tvec;
		Mat raux, taux;
		// 建立一个3d to 2d的映射
		solvePnP(m_markerCorners3d, _detectedMarkers[i].m_points, m_camMat, m_distCoeff, raux, taux);
		raux.convertTo(Rvec, CV_32F);
		taux.convertTo(Tvec, CV_32F);

// 		Mat_<float> rotMat(3,3);
		Matx33f rotMat;
		Rodrigues(Rvec, rotMat);

		_detectedMarkers[i].m_rotation = rotMat.t();
		_detectedMarkers[i].m_translation = -Tvec;
	}
}
