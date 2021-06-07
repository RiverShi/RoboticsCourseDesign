// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。


#include "FtpControl.h"
#include "RecDect.h"
#include "Function.h"
#include "MotionPlan.hpp"
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
int num;
double moving_time[100];

void target_to_pos(const My_rec& rec, PosStruct &pos);
void generate_traj();
void run_Cubic();

vector<string> plan_result;
CHLMotionPlan MyPlan;

fstream Aout, Bout;


int main()
{   

	  RobotConnect();
	  Robot_init();
	  Robot_to_initial_pose(); 

	//  int t = getchar();
	//  if(t == '1')Get_RGB();
	
	vector<My_rec> target;

	ColorDect(target, "test2.png");
	waitKey(30);

	num = target.size();
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
	MyPlan.SetProfile(50, 200, 50); 
	//10, 40, 10 偏慢
	MyPlan.SetProfileJ(20, 80, 20); 
	MyPlan.SetSampleTime(0.001);


	// MyPlan.SetPlanPoints(initpos, endpos);
	// MyPlan.GetPlanPoints_line(plan_result);

	// ofstream fout;
	// fout.open("data/data.txt");
	// for (string str : plan_result)fout << str << endl;
	// fout.close();
	// plan_result.clear();

	// PPB_run("data/data.txt");

	
//*******尝试三次插值****************************************************************
	
	Aout.open("data/A.txt");
	Bout.open("data/B.txt");

	run_Cubic(); 

	for (i = 0; i <= 13; i++)
	{
		cout << moving_time[i] << endl;
	}

	Aout.close();  Bout.close();

	return 0;

//****************************************************************************************
	
	generate_traj();


	 for (i = 0; i <= 13; i++)
	 {
	 	cout << moving_time[i] << endl;
	 }


	 

	 cout << "running_data0.txt"<< endl;
	 PPB_run("data/data0.txt", ceil(moving_time[0])*1000 + 300);

	 for (i = 1; i <=12; i += 2)
	 {
	 	suckin();
	 	cout << "running_data" + to_string(i) << endl;
		PPB_run("data/data" + to_string(i) + ".txt",ceil(moving_time[i])*1000 + 300);
	 	suckout();
	 	suckoff();
	 	cout << "running_data" + to_string(i+1) << endl;
	  	PPB_run("data/data" + to_string(i+1) + ".txt", ceil(moving_time[i+1])*1000 + 300);
	 }

	 cout << "running_data13" << endl;
	 PPB_run("data/data13.txt", ceil(moving_time[13])*1000 + 300);


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


double sampletime = 0.001;
double v_ave = 80, w_ave = 40;

double Lift_cubic(PosStruct pos, double height,  vector<string> &result)
{
	cout << "Lift  height = " << height << endl;
//*****调试*****************************************************************************
	//Aout << "Lift  height = " << height << endl;
	//Bout << "Lift  height = " << height << endl;
//*****************************************************************************************
	Cubic plan;
	double Td = fabs(height) / v_ave + 0.1;
	plan.init(pos.z, 0, pos.z+height, 0, Td);
	Point_info point;
	point.assign(pos);

	string Joint = "", spatial = ""; 

//*****调试********************************************************************************
	//HLRobot::SetPose(point.pose);
	//HLRobot::GetAngles(point.angle);
	//for (int i = 1; i <= 6; i++)Joint = Joint + to_string(point.angle[i]) + " ";
	//result.push_back(Joint);

	//for (int i = 1; i <= 6; i++)spatial = spatial + to_string(point.pose[i]) + " ";
	//Aout <<"start pos Joint :" << Joint << endl;  
	//Bout << "start pos spatial :" << spatial << endl;
//******************************************************************************************

	double time = 0;
	while(time <= Td)
	{
		Joint = "", spatial = "";
		point.z = plan.get_pos(time);
		HLRobot::SetPose(point.pose);
		HLRobot::GetAngles(point.angle);
		for(int i = 1; i <= 6; i++)Joint = Joint + to_string(point.angle[i]) + " ";
		result.push_back(Joint);
		time += sampletime; 
//****调试*********************************************************************************
		for(int i = 1; i <= 6; i++)spatial = spatial + to_string(point.pose[i]) + " ";
		Aout << Joint << endl;  Bout << spatial << endl; 
		//cout << spatial << endl;
//*******************************************************************************************
	}
	cout << endl;

//*****调试**********************************************************************************
	//Aout <<  endl;  Bout  << endl;
	
//*******************************************************************************************
	
	return Td;
}


double Move_cubic(PosStruct startpos, PosStruct endpos,  vector<string> &result)
{
	cout << "Move" << endl;
//*******************************************************************************************
	//Aout << "Move" << endl;
	//Bout << "Move" << endl;
//*******************************************************************************************
	Cubic plan[8];
	Point_info s, t;
	s.assign(startpos); t.assign(endpos);

	string Joint = "", spatial = "";

//*******************************************************************************************
	//HLRobot::SetPose(s.pose);
	//HLRobot::GetAngles(s.angle);
	//for (int i = 1; i <= 6; i++)Joint = Joint + to_string(s.angle[i]) + " ";
	//result.push_back(Joint);

	//for (int i = 1; i <= 6; i++)spatial = spatial + to_string(s.pose[i]) + " ";
	//Aout << "start pos Joint :" << Joint << endl;
	//Bout << "start pos spatial :" << spatial << endl;


	//Joint = "", spatial = "";
	//HLRobot::SetPose(t.pose);
	//HLRobot::GetAngles(t.angle);
	//for (int i = 1; i <= 6; i++)Joint = Joint + to_string(t.angle[i]) + " ";

	//for (int i = 1; i <= 6; i++)spatial = spatial + to_string(t.pose[i]) + " ";
	//Aout << "end pos Joint :" << Joint << endl;
	//Bout << "end pos spatial :" << spatial << endl;
//*******************************************************************************************


	double Td = 0;
	int i;
	for(i = 1; i <= 3; i++) Td = max(Td, fabs(t.pose[i]-s.pose[i])/v_ave);
	Td = max(Td, fabs(t.pose[4]-s.pose[4])/w_ave);
	Td += 0.1;
	for(i = 1; i <= 6; i++)plan[i].init(s.pose[i], 0, t.pose[i], 0, Td);
	Point_info point;
	point.pose[5] = 180; point.pose[6] = 138;	//5、6轴固定不变
	double time = 0;
	while(time <= Td)
	{
		Joint = "", spatial = "";
		for(i = 1; i <= 4; i++)point.pose[i] = plan[i].get_pos(time);
		HLRobot::SetPose(point.pose);
		HLRobot::GetAngles(point.angle);
		for(int i = 1; i <= 6; i++)Joint = Joint + to_string(point.angle[i]) + " ";
		result.push_back(Joint);
		time += sampletime; 
//*******************************************************************************************
		for(int i = 1; i <= 6; i++)spatial = spatial + to_string(point.pose[i]) + " ";
		Aout << Joint << endl;  Bout << spatial << endl;
		//cout << spatial << endl;


//*******************************************************************************************
	}

	cout << endl;
//*******************************************************************************************
	//Aout <<  endl;  Bout  << endl;
	
//*******************************************************************************************
	return Td;
}

double plan_cubic(PosStruct startpos, PosStruct endpos, int index)
{
	double time = 0;
	double H = max(startpos.z, endpos.z) + 40;
	PosStruct H1 = startpos, H2 = endpos;
	H1.z = H2.z = H;

	plan_result.clear();
	time += Lift_cubic(startpos, H - startpos.z, plan_result);
	time += Move_cubic(H1, H2, plan_result);
	time += Lift_cubic(H2, endpos.z - H, plan_result);

	ofstream fout;
	fout.open("data/data_cubic"+to_string(index)+".txt");
	for(auto &str : plan_result)fout << str << endl;
	fout.close();
	plan_result.clear();

	cout << "plan " + to_string(index) + " finish" << endl;

	return time;
}

PosStruct place[30];

/*
void gen_place(PosStruct firstpos, int n)
{
	place[1] = firstpos;
	place[2] = place[1]; place[2].y += 50;
	place[3] = place[1]; place[3].z += 15; place[3].yaw = place[1].yaw + 90;
	place[4] = place[2]; place[4].z += 15; place[4].yaw = place[2].yaw + 90;
	place[5] = place[1]; place[5].z += 30; 
	place[6] = place[2]; place[6].z += 30;

	for(int i = 7; i < num; i++)
	{
		place[i] = firstpos;  place[i].y += 25; place[i].yaw = firstpos.yaw + 90;
		place[i].z = firstpos.z + (i-4)*15;
	}

	if(num = 7)

	if(num > 7)
	{
		place[num] = firstpos; 
		place[num].x += 25; place[num].y += 25; 
		place[num].z = place[num-1].z + 25;
		place[num].yaw = firstpos.yaw + 90;
		place[num].pitch = 90;
	}
}
*/

void place_init()
{
	
	place[1].assign(	376, -271, 463,			0,	180, 138);
	place[2].assign(	376, -324, 463,			 0, 180, 138);
	place[3].assign(	396, -300, 478,			90, 180, 138);
	place[4].assign(	344, -300, 478,			90, 180, 138);
	place[5].assign(	376, -271, 493,			0, 180, 138);
	place[6].assign(	376, -324, 493,			 0, 180, 138);

	place[7].assign(	376, -300, 508,			90, 180, 138);
	place[8].assign(	376, -300, 523,			90, 180, 138);
	place[9].assign(	376, -300, 538,			90, 180, 138);
	place[10].assign(	376, -300, 553,			90, 180, 138);
	place[11].assign(	376, -300, 568,			90, 180, 138);
	place[12].assign(	376, -300, 583,			90, 180, 138);
	place[13].assign(	376, -300, 598,			90, 180, 138);
	place[14].assign(	376, -300, 613,			90, 180, 138);
	place[15].assign(	376, -300, 528,			90, 180, 138);
}


void run_Cubic()
{

	//gen_place(first_place_pos, num);
	place_init();

	PosStruct initpos(410, 0, 800, 0, 180, 138);
	ofstream fout;

	plan_result.clear();
 	moving_time[0] = Move_cubic(initpos, dst[1], plan_result);
	cout << "plan 0 finish" << endl;
	fout.open("data/data_cubic0.txt");
	for(auto &str : plan_result)fout << str << endl;
	fout.close();
	plan_result.clear();

	int i;
	for(i = 1; i < num; i++)
	{
		moving_time[2*i-1] = plan_cubic(dst[i], place[i], 2*i-1);
		moving_time[2*i] = plan_cubic(place[i], dst[i+1], 2*i);
	}

	moving_time[2*num-1] = plan_cubic(dst[num], place[num], 2*num-1);


	 cout << "running_data0.txt"<< endl;
	 PPB_run("data/data_cubic0.txt", ceil(moving_time[0])*1000 + 300);

	 for (i = 1; i <num; i ++ )
	 {
	 	suckin();
	 	cout << "running_data" + to_string(2*i-1) << endl;
	 	PPB_run("data/data_cubic" + to_string(2*i-1) + ".txt",ceil(moving_time[2*i-1])*1000 + 300);
	 	suckout();
	 	suckoff();
	 	cout << "running_data" + to_string(2*i) << endl;
	 	PPB_run("data/data_cubic" + to_string(2*i) + ".txt", ceil(moving_time[2*i])*1000 + 300);
	 }

	 suckin();
	 cout << "running_data_cubic" + to_string(2*num-1) + ".txt" << endl;
	 PPB_run("data/data_cubic" + to_string(2*num-1) + ".txt", ceil(moving_time[num*2-1])*1000 + 300);
	 suckout();
	 suckoff();

	  
	 void PPB_stop();
	 close();

	return ;
}






