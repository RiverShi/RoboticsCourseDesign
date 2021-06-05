#ifndef RECDECT_H
#define RECDECT_H



#include <opencv/cv.h>
#include <string.h>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "fstream"
#include <librealsense2/rs.hpp>


//using namespace cv;
using namespace std;


constexpr auto W_DST = "dstImage";
constexpr auto W_RST =  "resultImage";
//#define fps 30;

const double PI = 3.1415926;


//j矩形识别头文件，请自行添加需要的函数




void Get_RGB();
double get_distance(cv::Point2f p1, cv::Point2f p2);
double get_distance(cv::Point3f p1, cv::Point3f p2);
cv::Point3f pixel_to_camera(cv::Point2f p);
cv::Point3f camera_to_world(cv::Point3f p);

class My_rec
{
public:
	cv::Point2f center;           //像素坐标中点位置
	cv::Point2f vertex[4];		  //像素坐标定点位置
	cv::Point3f c_center;         //相机坐标中点位置
	cv::Point3f c_vertex[4];
	cv::Point3f w_center;         //世界坐标中点位置
	cv::Point3f w_vertex[4];
	int id;                   //0：正方形 1：矩形
	double theta, length, width, w_length, w_width, w_theta; //偏转角度，长边长度，短边长度，相机坐标系中中点位置到（0,0,Zc)距离
	void rec(cv::Point2f * p);	  //通过四边形四个定点对类初始化
	void sort();      //定点排序
	void print();     //打印信息
	void uv_to_xyz();         //中点位置从像素坐标到相机坐标、世界坐标系
};

void ColorDect(vector<My_rec>& target, string filename);
#endif // 



