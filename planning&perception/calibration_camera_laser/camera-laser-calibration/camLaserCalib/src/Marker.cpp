#include "Marker.h"

const int Marker::m_idVerify[4][3] = {{0,1,0},
						{0,1,1},
						{1,0,0},
						{1,1,1}};	// 设计4个word对应的id信息分别为00、01、10、11

Marker::Marker(void)
{
	m_id = -1;
	m_rotation.eye();
	m_translation.zeros();
}

Marker::~Marker(void)
{
}

Mat Marker::rotate(const Mat& _input)
{
	// 把code旋转90°
	Mat output;
	_input.copyTo(output); 
	for (int i=0; i<_input.rows; i++)
	{
		for (int j=0; j<_input.cols; j++)
		{
			output.at<uchar>(i,j) = _input.at<uchar>(_input.cols-j-1, i);
		}
	}
	return output;
}

int Marker::hammDistMarker(const Mat& _code)
{
	int dist = 0;

	for (int i=0; i<3; i++)
	{
		int minDistWord = INT_MAX;
		for (int x=0; x<4; x++)	// 去匹配每一个word，找出distance最近的word
		{
			int sumTemp = 0;
			for (int j=0; j<3; j++)
			{
				sumTemp += (_code.at<uchar>(i,j) == m_idVerify[x][j]) ? 0 : 1;
			}
			if (minDistWord > sumTemp)
			{
				minDistWord = sumTemp;
			}
		}
		dist += minDistWord;
	}
	
	return dist;
}

int Marker::code2ID(const Mat& _code)
{
	// 因为只有4个wordID，取第2位和第4位作为id，4个word的id分别为00,01,10,11
	int val = 0;
	for (int i=0; i<3; i++)
	{
		val <<= 1;
		if (_code.at<uchar>(i,0))
			val |= 1;
		val <<= 1;
		if (_code.at<uchar>(i,2))
		{
			val |= 1;
		}
	}
	return val;
}

int Marker::decode(Mat& _input, int& _numRotation)
{
	CV_Assert(_input.rows == _input.cols);
	CV_Assert(_input.type() == CV_8UC1);

	// 处理为2值图像grey
	Mat grey;
	threshold(_input, grey, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
// 	imshow("improvedMarker", grey);
	
	// Marker是5*5的区域，周围边框是全黑，中心3*3是有信息的marker
	int patchSize = _input.rows / 5;
	// 先判断周围一圈是否全黑
	for (int i=0; i<5; i++)
	{
		int step = 4;
		if (i==0 || i==4)
			step = 1;
		for (int j=0; j<5; j+=step)
		{
			int x = j * patchSize;
			int y = i * patchSize;
			Mat patch = grey(Rect(x, y, patchSize, patchSize));
			int numNoZero = countNonZero(patch);
			if (numNoZero > (patchSize*patchSize/2) )
			{
				return -1;	// 周围一圈不是全黑,说明这个不是marker
			}
		}
	}

	// 得到中心3*3区域的code
	Mat codemap = Mat::zeros(3, 3, CV_8UC1);
	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			int x = (j+1) * patchSize;
			int y = (i+1) * patchSize;
			Mat patch = grey(Rect(x, y, patchSize, patchSize));
			int numNoZero = countNonZero(patch);
			if (numNoZero > (patchSize*patchSize/2) )
			{
				codemap.at<uchar>(i,j) = 1;
			}
		}
	}

	// 解码code,因为code的方向性，要对4个方向都计算，取最小的hammdist作为id
	int minDist = INT_MAX;
	int minDistIndex = -1;
	Mat codemapRotate[4];
	codemapRotate[0] = codemap;
	for (int i=0; i<4; i++)	// 每次旋转90°，所以旋转4次
	{
		int distTemp = hammDistMarker(codemapRotate[i]);
		if (distTemp < minDist)
		{
			minDist = distTemp;
			minDistIndex = i;
		}
		codemapRotate[(i+1)%4] = rotate(codemapRotate[i]);
	}

	_numRotation = minDistIndex;
	if (minDist == 0)	// id匹配,返回id
	{
		return code2ID(codemapRotate[minDistIndex]);
	}

	return -1;	// 和id不匹配，不是marker
}

float Marker::calPerimeter()
{
	float sum = 0.0f;
	for (size_t i=0; i<m_points.size(); i++)
	{
		size_t j = (i+1) % m_points.size();
		Point2f vec = m_points[i] - m_points[j];
		sum += sqrt(vec.dot(vec));
	}
	return sum;
}