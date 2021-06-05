// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。


#include "FtpControl.h"
#include "RecDect.h"
#include "Function.h"
#include "MotionPlan/MotionPlan.h"
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


using namespace std;
using namespace cv;

PosStruct dst[100];
double moving_time[100];

void target_to_pos(const My_rec& rec, PosStruct &pos);
void generate_traj();

vector<string> plan_result;
CHLMotionPlan MyPlan;

int main()
{   

	RobotConnect();
	Robot_init();
	Robot_to_initial_pose(); 

	int t = getchar();
	if(t == '1')Get_RGB();
	
	vector<My_rec> target;

	ColorDect(target, "test2.png");
	waitKey(30);

	int num = target.size();
	int i = 0;
	for (i = 0; i < num; i++)target_to_pos(target[i], dst[i + 1]);
	cout << "目标空间坐标：" << endl;
	for (i = 1; i <= num; i++)printf("%lf %lf %lf %lf %lf %lf\n", dst[i].x, dst[i].y, dst[i].z, dst[i].yaw, dst[i].pitch, dst[i].roll);
	cout << endl;

	//设置初始坐标点位
	PosStruct initpos(410, 0, 800, 0, 180, 138);
	PosStruct endpos(500, 0, 600, 100, 180, 138);

	//设置速度规划参数
	//50，200，50 可
	MyPlan.SetProfile(40, 160, 40); 
	//10, 40, 10 偏慢
	MyPlan.SetProfileJ(15, 60, 15); 
	MyPlan.SetSampleTime(0.001);


	// MyPlan.SetPlanPoints(initpos, endpos);
	// MyPlan.GetPlanPoints_line(plan_result);

	// ofstream fout;
	// fout.open("data/data.txt");
	// for (string str : plan_result)fout << str << endl;
	// fout.close();
	// plan_result.clear();

	// PPB_run("data/data.txt");
	
	
	generate_traj();

	for (i = 0; i <= 13; i++)
	{
		cout << moving_time[i] << endl;
	}

	//string filename, name;
	//for (i = 0; i <=13; i++)
	//{
	//	filename = "data/data" + to_string(i) + ".txt";
	//	name = "data" + to_string(i) + ".txt";
	//	cout << "上传文件 : " << filename << endl;
	//	FtpControl::Upload("192.168.10.101", "data", filename.c_str(), name.c_str());
	//}

	for (i = 0; i <= 13; i++)
	{
		cout << "running_data" + to_string(i) << endl;
		PPB_run("data/data" + to_string(i) + ".txt", int(moving_time[i+1]*1000)+300);
	}

	 

	// cout << "running_data0.txt"<< endl;
	// PPB_run("data/data0.txt", int(moving_time[0]*1000)+300);

	// for (i = 1; i <=12; i += 2)
	// {
	// 	suckin();
	// 	cout << "running_data" + to_string(i) << endl;
	//  	PPB_run("data/data" + to_string(i) + ".txt",int(moving_time[i]*1000)+300);
	// 	suckout();
	// 	suckoff();
	// 	cout << "running_data" + to_string(i+1) << endl;
	//  	PPB_run("data/data" + to_string(i) + ".txt", int(moving_time[i+1]*1000)+300);
	// }

	// cout << "running_data13" << endl;
	// PPB_run("data/data13.txt", int(moving_time[13]*1000)+300);


	void PPB_stop();
	close();

	getchar();

	return 0;
}


void target_to_pos(const My_rec& rec, PosStruct& pos)
{ 
	pos.x = rec.w_center.x * 1000;
	pos.y = rec.w_center.y * 1000;
	pos.z = 463;
	pos.yaw = rec.w_theta;
	pos.pitch = 180;
	pos.roll = 138;

	while (pos.yaw > 100) pos.yaw -= 180;
	while (pos.yaw < -100)pos.yaw += 180;
}


void generate_traj()
{
	PosStruct initpos(410, 0, 800, 0, 180, 138);
	//PosStruct endpos(618.047, -73.899, 458, -137, 180, 0);

	//创建积木的吸取点位,这里只选择其中的6个
	PosStruct prepick[30];
	PosStruct pick[30];
	int i;
	for (i = 1; i <= 6; i++)
	{
		prepick[i] = dst[i]; prepick[i].z += 80;
		pick[i] = dst[i];
	}

	//设置搭建积木的目标点位，6个
	PosStruct preplace[30];
	PosStruct place[30];

	preplace[1].assign(	376, -271, 463 + 80,	0, 180, 138);
	place[1].assign(	376, -271, 463,			0,	180, 138);

	preplace[2].assign(	376, -324, 463 + 80,	 0, 180, 138);
	place[2].assign(	376, -324, 463,			 0, 180, 138);

	preplace[3].assign(	396, -300, 478 + 80,	90, 180, 138);
	place[3].assign(	396, -300, 478,			90, 180, 138);

	preplace[4].assign(	344, -300, 478 + 80,	90, 180, 138);
	place[4].assign(	344, -300, 478,			90, 180, 138);

	preplace[5].assign(	376, -271, 493 + 80,	 0, 180, 138);
	place[5].assign(	376, -271, 493,			0, 180, 138);

	preplace[6].assign(	376, -324, 493 + 80,	 0, 180, 138);
	place[6].assign(	376, -324, 493,			 0, 180, 138);


	//点位已经设置完成，接下来开始规划
	cout << "begin to plan" << endl;

	//前提：已回到初始点位
	//开始分段规划
	
	double mtime = 0;

	//从初始点到达第一个预抓取点,data0
	MyPlan.SetPlanPoints(initpos, pick[1]);
	mtime += MyPlan.GetPlanPoints_line(plan_result);

	ofstream fout;
	fout.open("data/data0.txt");
	for (string str : plan_result)fout << str << endl;
	fout.close();
	plan_result.clear();
	cout << "plan data0  finished" << endl;

	moving_time[0] = mtime;

	//data1~data12，一共2×6个门字形
	
	for (i = 1; i <= 6; i++)
	{
		mtime = 0;
		//门字形forward
		MyPlan.SetPlanPoints(pick[i], prepick[i]);
		mtime += MyPlan.GetPlanPoints_line(plan_result);

		MyPlan.SetPlanPoints(prepick[i], preplace[i]);
		mtime += MyPlan.GetPlanPoints_line(plan_result);

		MyPlan.SetPlanPoints(preplace[i], place[i]);
		mtime += MyPlan.GetPlanPoints_line(plan_result);

		moving_time[2*i-1] = mtime;

		//ofstream fout;
		fout.open("data/data" + to_string(i * 2 - 1) + ".txt");
		for (string str : plan_result)fout << str << endl;
		fout.close();
		plan_result.clear();
		cout << "plan data" + to_string(i * 2 - 1) + " finished" << endl;

		mtime = 0;
		//门字形back
		MyPlan.SetPlanPoints(place[i], preplace[i]);
		mtime += MyPlan.GetPlanPoints_line(plan_result);

		if (i < 6) {
			MyPlan.SetPlanPoints(preplace[i], prepick[i + 1]);
			mtime += MyPlan.GetPlanPoints_line(plan_result);

			MyPlan.SetPlanPoints(prepick[i + 1], pick[i + 1]);
			mtime += MyPlan.GetPlanPoints_line(plan_result);
		}

		moving_time[2*i] = mtime;

		//ofstream fout;
		fout.open("data/data" + to_string(i * 2) + ".txt");
		for (string str : plan_result)fout << str << endl;
		fout.close();
		plan_result.clear();
		cout << "plan data" + to_string(i * 2) + " finished" << endl;
	}

	mtime = 0;
	//抓取完毕，回到初始点位，data13
	MyPlan.SetPlanPoints(preplace[6], initpos);
	mtime += MyPlan.GetPlanPoints_line(plan_result);
	moving_time[13] = mtime;
	//ofstream fout;
	fout.open("data/data" + to_string(13) + ".txt");
	for (string str : plan_result)fout << str << endl;
	fout.close();
	plan_result.clear();
	cout << "plan data" + to_string(13) + " finished" << endl;

	return;
}




