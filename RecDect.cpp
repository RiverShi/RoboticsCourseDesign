
#include "RecDect.h"
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

using namespace cv;
using namespace std;

#define W_DST "dstImage" 
#define W_RST "resultImage"
#define PI 3.1415926
#define fps  30

//变量声明
int h_min1 = 0;
int h_min2 = 0; //红色色调取值区间有两个
int s_min = 0;
int v_min = 224;

int h_max1 = 180;
int h_max2 = 180;
int s_max = 255;
int v_max = 255;

//声明存储图像的变量
Mat srcImage;
Mat dstImage;
Mat dstImage1;
Mat dstImage2;
Mat tempImage;

//这里是syz自己加的预声明
double get_distance(Point2f p1, Point2f p2);
double get_distance(Point3f p1, Point3f p2);

void Get_RGB();
//double get_distance(Point2f p1, Point2f p2);
//double get_distance(Point3f p1, Point3f p2);

Point3f pixel_to_camera(Point2f p);
Point3f camera_to_world(Point3f p);



//syz
//这里声明传参的变量
//Point3f w_center;
 
void My_rec::rec(Point2f* p)
{

	for (int i = 0; i < 4; i++)
	{
		this->vertex[i] = p[i];
	}
	this->sort();
	center.x = (vertex[0].x + vertex[1].x + vertex[2].x + vertex[3].x) / 4;
	center.y = (vertex[0].y + vertex[1].y + vertex[2].y + vertex[3].y) / 4;
	length = get_distance(vertex[0], vertex[1]);
	width = get_distance(vertex[0], vertex[2]);
	theta = atan2(vertex[1].y - vertex[0].y, vertex[1].x - vertex[0].x) * 180 / PI;
	if (length < width)
	{
		double temp;
		temp = length;
		length = width;
		width = temp;
		theta = atan2(vertex[2].y - vertex[0].y, vertex[2].x - vertex[0].x) * 180 / PI;
	}
	//w_theta = theta + 81;

	if ((length - width) > 10)
		id = 1;
	else
		id = 0;
	uv_to_xyz();

}

void My_rec::uv_to_xyz()
{
	this->c_center = pixel_to_camera(this->center);
	this->w_center = camera_to_world(this->c_center);
	for (int i = 0; i < 4; i++)
	{
		this->c_vertex[i] = pixel_to_camera(this->vertex[i]);
		this->w_vertex[i] = camera_to_world(this->c_vertex[i]);
	}
	w_length = get_distance(w_vertex[0], w_vertex[1]);
	w_width = get_distance(w_vertex[0], w_vertex[2]);
	w_theta = atan2(w_vertex[1].y - w_vertex[0].y, w_vertex[1].x - w_vertex[0].x) * 180 / PI;
	if (w_length < w_width)
	{
		double temp;
		temp = w_length;
		w_length = w_width;
		w_width = temp;
		w_theta = atan2(w_vertex[2].y - w_vertex[0].y, w_vertex[2].x - w_vertex[0].x) * 180 / PI;
	}
	//w_theta = -w_theta - 48 +3.4;

	//w_theta = -w_theta - 40 - 90 - 10 - 1.2;//学长的代码
	//w_theta = -w_theta - 40 - 90 - 10 - 1.2 + 320.526;
	w_theta = -w_theta - 40 - 90 - 10 - 1.2;
	w_theta = -w_theta - 320;//雷版本

	/*while (w_theta < -90)
	{
		w_theta += 180;
	}*/
}

void My_rec::print()
{
	cout << "像素顶点坐标： ";
	for (int i = 0; i < 4; i++)
	{
		cout << this->vertex[i] << " ";
	}
	cout << endl;
	cout << "像素中点坐标: " << center << endl;
	//cout << "长边像素长度: " << length << endl;
	//cout << "短边像素长度:" << width << endl;
	cout << "像素偏转角度 : " << theta << endl;
	cout << "ID（0：正方形，1：矩形): " << id << endl;
	cout << "中点相机坐标： " << c_center << endl;
	cout << "世界顶点坐标： ";
	for (int i = 0; i < 4; i++)
	{
		cout << this->w_vertex[i] << " ";
	}
	cout << endl;
	cout << "世界中点坐标： " << w_center << endl;
	cout << "世界偏转角度 " << w_theta << endl;
	cout << "********************************************************************************************************************" << endl << endl;
}

void My_rec::sort()
{
	for (int i = 0; i < 4; i++)
	{
		for (int j = i + 1; j < 4; j++)
		{
			if (this->vertex[i].x > this->vertex[j].x)
			{
				Point2f temp;
				temp = this->vertex[i];
				this->vertex[i] = this->vertex[j];
				this->vertex[j] = temp;
			}
		}
	}
	if (get_distance(this->vertex[0], this->vertex[2]) > get_distance(vertex[0], vertex[3]))
	{
		Point2f t;
		t = vertex[2];
		vertex[2] = vertex[3];
		vertex[3] = t;
	}
}

double get_distance(Point2f p1, Point2f p2)
{
	double distance;
	distance = powf((p1.x - p2.x), 2) + powf((p1.y - p2.y), 2);
	return sqrtf(distance);
}

double get_distance(Point3f p1, Point3f p2)
{
	double distance;
	distance = powf((p1.x - p2.x), 2) + powf((p1.y - p2.y), 2) + powf((p1.z - p2.z), 2);
	return sqrtf(distance);
}

Point3f pixel_to_camera(Point2f p)
{
	//double z = 370.9;

	//double z = 195.014;
	//double z = 390.134;
	double z = 444.23;//雷版本
	
	Mat K = Mat::zeros(3, 3, CV_32FC1);


	K.at<float>(0, 0) = 624.5663660138997;
	K.at<float>(0, 2) = 642.8147780382801;
	K.at<float>(1, 1) = 624.6166620471405;
	K.at<float>(1, 2) = 361.6785576625873;
	K.at<float>(2, 2) = 1;



	Mat uv = Mat::ones(3, 1, CV_32FC1);
	uv.at<float>(0, 0) = z * p.x;
	uv.at<float>(1, 0) = z * p.y;
	uv.at<float>(2, 0) = z * 1;

	Mat K_invert;

	invert(K, K_invert, DECOMP_LU);

	Mat cxyz = Mat::ones(3, 1, CV_32FC1);
	cxyz = K_invert * uv;

	Point3f cpoint;
	cpoint.x = cxyz.at<float>(0, 0);
	cpoint.y = cxyz.at<float>(1, 0);
	cpoint.z = cxyz.at<float>(2, 0);

	return cpoint;
}

Point3f camera_to_world(Point3f p)
{
	Mat T = Mat::zeros(4, 4, CV_32FC1);

	//T.at<float>(0, 0) = -0.000170152596286;
	//T.at<float>(0, 1) = 0.999987800356395;
	//T.at<float>(0, 2) = 0.004936616905580;
	//T.at<float>(0, 3) = 0.552867964420476;
	//T.at<float>(1, 0) = 0.999987056573284;
	//T.at<float>(1, 1) = 0.000145045378319;
	//T.at<float>(1, 2) = 0.005085828127006;
	//T.at<float>(1, 3) = 0.010815490322246;
	//T.at<float>(2, 0) = 0.005085050048249;
	//T.at<float>(2, 1) = 0.004937418375701;
	//T.at<float>(2, 2) = -0.999974881767432;
	//T.at<float>(2, 3) = 0.719473840300825;
	//T.at<float>(3, 3) = 1;

	T.at<float>(0, 0) = -0.01668797515942672;
	T.at<float>(0, 1) = 0.9998558814244927;
	T.at<float>(0, 2) = -0.003118952697545498;
	T.at<float>(0, 3) = 0.5419589559652026;
	T.at<float>(1, 0) = 0.9998349863286901;
	T.at<float>(1, 1) = 0.01666507257977812;
	T.at<float>(1, 2) = -0.007230177661656682;
	T.at<float>(1, 3) = 0.009848631896755181;
	T.at<float>(2, 0) = -0.007177158085673926;
	T.at<float>(2, 1) = -0.003239095052926203;
	T.at<float>(2, 2) = -0.9999689978519593;
	T.at<float>(2, 3) = 0.8294989356458385;
	T.at<float>(3, 3) = 1;

	//cv::Mat tmp = attitudeVectorToMatrix(ToolPosestart.row(0), false, "zyz"); //机械臂位姿为欧拉角-旋转矩阵
	//T = tmp * Hcg1;
	//cout << "T:" << T << endl;  

	Mat cxyz = Mat::ones(4, 1, CV_32FC1);
	cxyz.at<float>(0, 0) = p.x * 0.001;
	cxyz.at<float>(1, 0) = p.y * 0.001;
	cxyz.at<float>(2, 0) = p.z * 0.001;

	Mat wxyz = Mat::ones(4, 1, CV_32FC1);
	wxyz = T * cxyz;

	Point3f wpoint;
	wpoint.x = wxyz.at<float>(0, 0) +0.004;//雷版
	//wpoint.x = wxyz.at<float>(0, 0);
	wpoint.y = wxyz.at<float>(1, 0) + 0.012;//雷版
	//wpoint.y = wxyz.at<float>(1, 0);
	wpoint.z = wxyz.at<float>(2, 0);

	return wpoint;

}

//响应滑动条的回调函数
void ColorDect(vector<My_rec> &target, string filename)
//void ColorDect(vector<My_rec> &De)
{
	srcImage = imread(filename.c_str());
	//imshow("src", srcImage);
	tempImage = srcImage.clone();
	cvtColor(srcImage, srcImage, COLOR_BGR2HSV); //RGB to HSV
	Scalar hsv_min1(h_min1, s_min, v_min);
	Scalar hsv_max1(h_max1, s_max, v_max);
	Scalar hsv_min2(h_min2, s_min, v_min);
	Scalar hsv_max2(h_max2, s_max, v_max);
	//find image in range
	inRange(srcImage, hsv_min1, hsv_max1, dstImage1);
	inRange(srcImage, hsv_min2, hsv_max2, dstImage2);
	//将两个二值图像进行并操作
	dstImage = dstImage1 + dstImage2;
	//形态学操作
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(dstImage, dstImage, MORPH_ERODE, element, Point(-1, -1), 1);
	morphologyEx(dstImage, dstImage, MORPH_DILATE, element, Point(-1, -1), 4);
	//imshow(W_DST, dstImage);
	//find contuors
	vector<vector<Point>>contours;
	vector<Vec4i> hierarcy;
	findContours(dstImage, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	//draw contours on tempImage
	Scalar color(255, 0, 0);
	drawContours(tempImage, contours, -1, color, 2, 8);
	//namedWindow(W_RST, WINDOW_NORMAL);
	//imshow("W_RST", tempImage);//syz  这句把学长的注释掉了先
	imwrite("pic/result1.png", tempImage);

	Mat imageContours = Mat::zeros(dstImage.size(), CV_8UC1);	//最小外接矩形画布
	My_rec* rec = new My_rec[contours.size()];
	cout << "检测到" << contours.size() << "个矩形（正方形）" << endl;
	for (int i = 0; i < contours.size(); i++)
	{
		drawContours(tempImage, contours, i, Scalar(255), 1, 8, hierarcy);
		//绘制轮廓的最小外接矩形
		RotatedRect rect = minAreaRect(contours[i]);
		Point2f P[4];
		rect.points(P);
		rec[i].rec(P);
		for (int j = 0; j <= 3; j++)
		{
			line(tempImage, P[j], P[(j + 1) % 4], Scalar(255), 2);
		}
		//imshow("MinAreaRect", tempImage);
		waitKey(30);
		rec[i].print();
		//syz 这里加了两句传参的东西
		//w_center_Dect.push_back(rec[i].w_center);
		//w_theta_Dect.push_back(rec[i].w_theta);
		target.push_back(rec[i]);
	}
}

void Get_RGB()
{
	rs2::colorizer color_map;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
//Add desired streams to configuration
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
	pipe.start(cfg);
	Mat img_show;
	bool state = true;
	const auto window_name = "Display Image";
	string  a;
	namedWindow(window_name, WINDOW_AUTOSIZE);
	cout << "Please enter s to take photo!!!" << endl;
	while (state)
	{
		cin >> a;
		if (a == "s")
		{

			rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
			rs2::frame color = data.get_color_frame();
			rs2::depth_frame depth = data.get_depth_frame();
			// Query frame size (width and height)
			const int w = color.as<rs2::video_frame>().get_width();
			const int h = color.as<rs2::video_frame>().get_height();
			// Create OpenCV matrix of size (w,h) from the colorized depth data
			Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
			//   cvtColor(image1, image1, CV_BGR2RGB);
			   // Update the window with new data   
			imwrite("test2.png", image);
			//imshow(window_name, image);
			state = false;
			cout << "End of taking photo!!" << endl;
		}

	}
}



	