#ifndef _IMAGEFUSION
#define _IMAGEFUSION

//
// Created by xlz on 17-10-21.
//
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;


namespace TiEV{
#define TOL_ANGLE_DEG 1.0
#define TOL_DIST_METER 0.1
	struct pos
	{
		double utmx;
		double utmy;
    	double mHeading; //car Deg ISO EAST 0 CounterClockwise -Pi ~ Pi
    	pos(double x, double y, double heading)
    	{
    		utmx = x;
    		utmy = y;
    		mHeading =heading;
    	}
    };


//rotate map by rad counterwise-> +
    Mat get_rotate_map( Mat &src , double &angle_rad);

    Mat get_rotate_map_add_trans(Mat &src, double &angle_rad , double dx,double dy,double mheading);
//move image by pixel +x right  +y down
    void translateTransform( cv::Mat &src, cv::Mat& dst,  int &dx,  int &dy);
    Mat get_trans_by_warpAffine(Mat &src, double dx , double dy);
//src2 -> src1_inout map
    void map_fusion(Mat &src1_inout ,  pos pos1,  Mat src2 ,  pos pos2,  float res, float d ,uint8_t mask);

/*
* Merge two mats based on bit mask (in wiil be overlapped to in_out based on mask) added by John 2017.10
* both mat should be the same size
*/
    void overlap_mat(Mat in_out,  Mat &in,  uint8_t mask);


    void debug_show(cv::Mat current_map, string name);

}

#endif
