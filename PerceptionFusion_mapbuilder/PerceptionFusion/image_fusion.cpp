//
// Created by xlz on 17-10-21.
//

#include "image_fusion.h"
#include "common/nature.h"

namespace TiEV {
/*
* Merge two mats based on bit mask (in wiil be overlapped to in_out based on mask) added by John 2017.10
* both mat should be the same size
*/

    using namespace cv;
    using namespace TiEV;

    //can not call multiple times, X system will report error
    void debug_show(cv::Mat current_map, string name) {
        cv::namedWindow(name);
        cv::Mat show = current_map;
        for (int i = 0; i < current_map.rows; ++i) {
            for (int j = 0; j < current_map.cols; ++j) {
                if (current_map.ptr<uchar>(i)[j] & 0x1) {
                    /* code */
                    show.ptr<uchar>(i)[j] = 255;
                }
                if (current_map.ptr<uchar>(i)[j] & 0x2) {
                    /* code */
                    show.ptr<uchar>(i)[j] = 240;
                }
                if (current_map.ptr<uchar>(i)[j] & 0x4) {
                    /* code */
                    show.ptr<uchar>(i)[j] = 128;
                }
                if (current_map.ptr<uchar>(i)[j] & 0x8) {
                    /* code */
                    show.ptr<uchar>(i)[j] = 0;
                }
            }
        }
        //cv::equalizeHist(current_map, current_map);
        cv::imshow(name, show);
        cv::waitKey(1);

    }

    void overlap_mat(Mat in_out, Mat &in, uint8_t mask) {
        if (in_out.rows != in.rows || in_out.cols != in.cols) {
            cout << __FILE__ << __LINE__ << "overlapping two mat with different size!!" << endl;
            return;
        }
        for (int i = 0; i < in_out.rows; ++i) {
            for (int j = 0; j < in_out.cols; ++j) {
                if (in.ptr<uchar>(i)[j] > 0) {
                    /* code */
                    in_out.ptr<uchar>(i)[j] = in_out.ptr<uchar>(i)[j] | mask;
                }
            }
        }
    }

//TODO: MultiThread
    void map_fusion(Mat &src1_inout, pos pos1, Mat src2, pos pos2, float resolution,  float distance, uint8_t bitmask) //d:sensor to imu distance
    {
//        cout << "pos1.utmx = " << pos1.utmx << ' ' ;
//        cout << "pos1.utmy = " << pos1.utmy << ' ' ;
//        cout << "pos1.mheading = " << pos1.mHeading << endl; ;
//        cout << "pos2.utmy = " << pos2.utmy << ' ' ;
//        cout << "pos2.utmx = " << pos2.utmx << ' ' ;
//        cout << "pos2.mheading = " << pos2.mHeading << endl; ;

        //fixed size
        Mat src_temp1 = src1_inout.clone();
        Mat src_temp2 = src2.clone();

        //lux calibration
        if(bitmask == 0x04)
        {
            int dcol = round(0.2 / 0.2);
            int drow = -round(0.2 / 0.2);

            Mat trans_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
            translateTransform(src2, trans_temp, dcol, drow);

            Mat rotate_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));

            double delta_yaw = 1.5 / 180.0 *M_PI;
            rotate_temp = get_rotate_map(trans_temp, delta_yaw);

            src_temp2 = rotate_temp.clone();
        }

        //sick calibration
        if(bitmask ==0x08)
        {
            int dcol = -round(0.2 / 0.2);
            int drow = -round(0.2 / 0.2);

            Mat trans_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
            translateTransform(src2, trans_temp, dcol, drow);

            Mat rotate_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));

            double delta_yaw = 0.2 / 180.0 *M_PI;
            rotate_temp = get_rotate_map(trans_temp, delta_yaw);

            src_temp2 = rotate_temp.clone();
        }

//        //velodyne64 calibration
//        if(bitmask ==0x02)
//        {
//            int dcol = 0;
//            int drow = 0;
//
//            Mat trans_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
//            translateTransform(src2, trans_temp, dcol, drow);
//
//            Mat rotate_temp(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
//
//            double delta_yaw = -2.7 / 180.0 *M_PI;
//            rotate_temp = get_rotate_map(trans_temp, delta_yaw);
//
//            src_temp2 = rotate_temp.clone();
//        }

        //
        double delta_yaw = pos2.mHeading - pos1.mHeading;

        double delta_utmx = pos2.utmx - pos1.utmx ;
        double delta_utmy = pos2.utmy - pos1.utmy;

        Mat rotate_result(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        //rotate_result = get_rotate_map_add_trans(src_temp2, delta_yaw,delta_utmx,delta_utmy , pos1.mHeading);


        // 先旋转 后平移
        //
        rotate_result = get_rotate_map(src_temp2, delta_yaw);
//        if (fabs(delta_yaw) > TiEV::deg2rad(TOL_ANGLE_DEG)) {
//            rotate_result = get_rotate_map(src_temp2, delta_yaw);
//            cout<<"++++ rotate"<<endl;
//        } else {
//            rotate_result = src_temp2;
//            cout<<"---no rotate"<<endl;
//        }
        //平移
        double local_deltax = delta_utmx;
        double local_deltay = delta_utmy;
        TiEV::rotate2d(local_deltax, local_deltay, TiEV::TiEV_PI / 2.0 -pos1.mHeading);//y become forward x become lateral

        int dcol = round (local_deltax/resolution);
        int drow = -round (local_deltay/resolution);
        Mat rotate_transform_result(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        translateTransform(rotate_result, rotate_transform_result, dcol, drow);
//        if (delta_utmx * delta_utmx + delta_utmy * delta_utmy > TOL_DIST_METER * TOL_DIST_METER) {
//            translateTransform(rotate_result, rotate_transform_result, dcol, drow);
//            cout<<"++++ trans"<<endl;
//        } else {
//            rotate_transform_result = rotate_result;
//            cout<<"---no trans"<<endl;
//        }


        //叠加
        overlap_mat(src_temp1, rotate_transform_result, bitmask);
        src1_inout = src_temp1.clone();
    }

    Mat get_rotate_map(Mat &src, double &angle_rad) {
        Mat result(GRID_ROW, GRID_COL, CV_8UC1, Scalar(0));
        double angle = TiEV::rad2deg(angle_rad);
        double scale = 1;
        Mat rot_mat(2, 3, CV_32FC1);
        //cv::Point2f center(src.cols / 2, src.rows / 2);
        cv::Point2f center(CAR_CEN_COL ,CAR_CEN_ROW );
        rot_mat = getRotationMatrix2D(center, angle, scale);

        warpAffine(src, result, rot_mat, src.size());

        return result;
    }

    void translateTransform(cv::Mat &src, cv::Mat &dst, int &dx, int &dy) {
        CV_Assert(src.depth() == CV_8U);
        int rows = src.rows;
        int cols = src.cols;
        dst.create(rows, cols, src.type());

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                //平移后坐标映射到原图像
                int x = j - dx;
                int y = i - dy;
                //保证映射后的坐标在原图像范围内
                if (x >= 0 && y >= 0 && x < cols && y < rows && src.ptr<uchar>(y)[x] != 0)
                    dst.ptr<uchar>(i)[j] = src.ptr<uchar>(y)[x];
            }
        }
    }
}