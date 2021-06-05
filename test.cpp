
#include "MotionPlan/MotionPlan.h"
#include <string.h>
#include <vector>
#include <iostream>
#include "fstream" 
#include "RecDect.h"

using namespace std;


PosStruct dst[100];
void target_to_pos(const My_rec& rec, PosStruct &pos);

int main()
{   

	//RobotConnect();
	//Robot_init();
	//Robot_to_initial_pose();

	//Get_RGB();
	 
	vector<My_rec> target;

	ColorDect(target, "test.jpg");
	//waitKey(30);

	int num = target.size();
	int i = 0;
	for (i = 0; i < num; i++)target_to_pos(target[i], dst[i + 1]);
	cout << "目标空间坐标：" << endl;
	for (i = 1; i <= num; i++)printf("%lf %lf %lf %lf %lf %lf\n", dst[i].x, dst[i].y, dst[i].z, dst[i].yaw, dst[i].pitch, dst[i].roll);
	cout << endl;

	//设置初始坐标点位
	PosStruct initpos(419.973, 0, 733.737, -137, 180, 0);
	//PosStruct endpos(618.047, -73.899, 458, -137, 180, 0);

	//创建积木的吸取点位,这里只选择其中的6个
	PosStruct prepick[6];
	PosStruct pick[6];
	for (i = 1; i <= 6; i++)
	{
		prepick[i] = dst[i]; prepick[i].z += 20;
		pick[i] = dst[i];
	}

	//设置搭建积木的目标点位，6个
	PosStruct preplace[6];
	PosStruct place[6];

	preplace[1].assign(376, -271, 464+20, -137, 180, 0);
	place[1].assign(376, -271, 464, -137, 180, 0);
	preplace[2].assign(376, -324, 464+20, -137, 180, 0);
	place[2].assign(376, -324, 464, -137, 180, 0);
	preplace[3].assign(396, -300, 479+20, -137, 180, -90);
	place[3].assign(396, -300, 479, -137, 180, -90);
	preplace[4].assign(344, -300, 479+20, -137, 180, -90);
	place[4].assign(344, -300, 479, -137, 180, -90);
	preplace[5].assign(376, -271, 494+20, -137, 180, 0);
	place[5].assign(376, -271, 494, -137, 180, 0);
	preplace[6].assign(376, -324, 494+20, -137, 180, 0); 
	place[6].assign(376, -324, 494, -137, 180, 0);


	//点位已经设置完成，接下来开始规划
	cout << "begin to plan" << endl;
	//设置速度规划参数
	CHLMotionPlan MyPlan;
	MyPlan.SetProfile(30, 100, 30);
	MyPlan.SetProfileJ(10, 10, 10); 
	MyPlan.SetSampleTime(0.001);

	//前提：已回到初始点位
	//开始分段规划
	vector<string> plan_result;

	//从初始点到达第一个预抓取点,data0
	MyPlan.SetPlanPoints(initpos, pick[1]);
	MyPlan.GetPlanPoints_line(plan_result);

	ofstream fout;
	fout.open("data/data0.txt");
	for(string str : plan_result)fout << str << endl;
	fout.close();
	plan_result.clear();
	cout << "plan data0  finished"<< endl;

	
	//data1~data12，一共2×6个门字形
	for (i = 1; i <= 6; i++)
	{
		//门字形forward
		MyPlan.SetPlanPoints(pick[i], prepick[i]);
		MyPlan.GetPlanPoints_line(plan_result);

		MyPlan.SetPlanPoints(prepick[i], preplace[i]);
		MyPlan.GetPlanPoints_line(plan_result);

		MyPlan.SetPlanPoints(preplace[i], place[i]);
		MyPlan.GetPlanPoints_line(plan_result);

		//ofstream fout;
		fout.open("data/data" + to_string(i*2-1) + ".txt");
		for(string str : plan_result)fout << str << endl;
		fout.close();
		plan_result.clear();
		cout << "plan data" + to_string(i*2-1) +" finished"<< endl;

		//门字形back
		MyPlan.SetPlanPoints(place[i], preplace[i]);
		MyPlan.GetPlanPoints_line(plan_result);

		if(i < 6){
			MyPlan.SetPlanPoints(preplace[i], prepick[i+1]);
			MyPlan.GetPlanPoints_line(plan_result);

			MyPlan.SetPlanPoints(prepick[i+1], pick[i+1]);
			MyPlan.GetPlanPoints_line(plan_result);
		}

		//ofstream fout;
		fout.open("data/data" + to_string(i*2) + ".txt");
		for(string str : plan_result)fout << str << endl;
		fout.close();
		plan_result.clear();
		cout << "plan data" + to_string(i*2) +" finished"<< endl;
	}

	//抓取完毕，回到初始点位，data13
	MyPlan.SetPlanPoints(preplace[6], initpos);
	MyPlan.GetPlanPoints_line(plan_result);
	//ofstream fout;
	fout.open("data/data" + to_string(13) + ".txt");
	for(string str : plan_result)fout << str << endl;
	fout.close();
	plan_result.clear();
	cout << "plan data" + to_string(13) +" finished"<< endl;


	//SendToRobot(plan_result);
	//FtpControl::Upload("192.168.10.101", "data", "data.txt", "severdata.txt");

	// for (i = 0; i <=13; i++)
	// {
	// 	PPB_run("data/data" + to_string(i) + ".txt");
	// 	cout << "running_data" + to_string(i) << endl;
	// }

	//void PPB_stop()

	//close();



	return 0;
}


void target_to_pos(const My_rec& rec, PosStruct& pos)
{ 
	pos.x = rec.w_center.x * 1000;
	pos.y = rec.w_center.y * 1000;
	pos.z = 458;
	pos.yaw = -137;
	pos.pitch = 180;
	pos.roll = rec.w_theta;

	if (pos.roll > 180) pos.roll -= 360;
	if (pos.roll < -180)pos.roll += 360;
}

