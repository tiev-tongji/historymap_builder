#pragma once 

#include <iostream>
#include <fstream>
#include <vector>

#include "opencv2/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "Eigen/Core"
using namespace Eigen;
#include "structLASERMAP.hpp"
#include "common/nature.h"
namespace  TiEV{
	struct ScanLine{
		int id;
		int state;
		cv::Point endpoint_ori;
		cv::Point endpoint_access;
		std::vector<cv::Point> linepoints_ori;
		std::vector<cv::Point> linepoints_access;
	};

	void GenerateVirtualScans(structLASERMAP grid, std::vector<Eigen::Vector3f> &virtusalscans);
}


