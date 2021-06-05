#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include "HLrobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include "eigen3/Eigen/Dense"
#include "commom_define.h"

using namespace std;
using namespace HLRobot;
using namespace Eigen;

struct PosStruct
{
	double x;				// x坐标，单位mm
	double y;				// y坐标，单位mm
	double z;				// z坐标，单位mm
	double yaw;				// yaw坐标，单位度
	double pitch;			// pitch坐标，单位度
	double roll;			// roll坐标，单位度

	bool config[3]{1,1,1};	// config, 表示机器人姿态

	PosStruct(){}
	PosStruct(double x, double y, double z, double yaw, double pitch, double roll)
		:x(x), y(y), z(z), yaw(yaw), pitch(pitch), roll(roll){}
	
	void assign(double xx, double yy, double zz, double yyaw, double ppitch, double rroll)
	{
		x = xx;  y = yy;  z = zz;
		yaw = yyaw;  pitch = ppitch;  roll = rroll;
	}
};

struct LFPB
{
	double acc, vmax, dec;
	double Length;
	double tacc, tmov, tdec;
	double t1, t2, t3; //t1 = tacc, t2 = t1 + tmov, t3 = t2 + tdec
	double flag;

	void init(double a, double v, double d, double L)
	{
		acc = a; vmax = v; dec = d; Length = fabs(L);
		if(L > 0) flag = 1.0;
		else flag = -1.0;

		tacc = vmax / acc;
		tdec = vmax / dec;
		double s = Length - 0.5 * vmax * tacc - 0.5 * vmax * tdec;
		if(s >= 0 ) tmov = s / vmax;
		else
		{
			tmov = 0;
			tacc = sqrt(2.0 * Length * dec / (acc * (acc + dec)));
			tdec = tacc * acc / dec; 
		}
		t1 = tacc; t2 = t1 + tmov; t3 = t2 + tdec; 
	}


	double get_pos(double time)
	{
		double s = 0, v = acc * tacc;
		if (time >= t3)return Length * flag;	//已到达预定角度并停下
		if (time <= t1)return 0.5 * acc * time * time * flag;  //加速段

		/////////////////////////////////////////////////////////////////////////////
		//	匀速段速度 v = acc * tacc;
		//	匀速段 s = 0.5*a1*t1^2 + a1*tacc*(t-t1)
		//
		//	减速段 s = 0.5*a1*t1^2 + a1*tacc*tmov + a1*t1*(t-t2)-0.5*a2*(t-t2)^2;
		//			= 0.5*a1*t1^2 + v*(t-t1) -0.5 * v * (t-t2)^2
		// 
		/////////////////////////////////////////////////////////////////////////////

		s = 0.5 * v * tacc;
		s += v * (time - t1);
		if(time >= t2)s -= 0.5 * dec * (time - t2) * (time - t2);  //减速段
		return s * flag;
	}

	double get_vel(double time)
	{
		if(time <= 0)return 0;
		if(time <= t1)return acc * time;
		if(time <= t2)return acc * t1;
		if(time <= t3)return acc * t1 - dec * (time - t2);
		return 0; 
	}
};



class CHLMotionPlan
{
private:
	double mJointAngleBegin[6];					//起始点位的关节角度,单位度
	double mJointAngleEnd[6];					//结束点位的关节角度，单位度
	double mStartMatrixData[16];				//起始点位的转换矩阵数组
	double mEndMatrixData[16];					//结束点位的转换矩阵数组
	double mSampleTime;							//采样点位，单位S
	double mVel;								//速度，单位m/s
	double mAcc;								//加速度，单位m/s/s
	double mDec;								//减速度，单位m / s / s
	bool mConfig[3];							//机器人姿态
	double JmVel, JmAcc, JmDec;
	double start_x, start_y, start_z;
	double end_x, end_y, end_z;
	double end_yaw, end_pitch, end_roll;
	double start_yaw, start_pitch, start_roll;
	PosStruct sp, ep;


public:
	CHLMotionPlan()
	{
		for (int i = 0; i < 6; i++)
		{
			mJointAngleBegin[i] = 0;
			mJointAngleEnd[i] = 0;
		}

		for (int i = 0; i < 16; i++)
		{
			mStartMatrixData[i] = 0;
			mEndMatrixData[i] = 0;
		}

		mSampleTime = 0.001;
		mVel = 0;
		mAcc = 0;
		mDec = 0;
	}

	virtual ~CHLMotionPlan(){}

	void SetSampleTime(double sampleTime)							//设置采样时间
	{
		if (sampleTime < 0.001)
		{
			mSampleTime = 0.001;
		}
		else
		{
			mSampleTime = sampleTime;
		}
	}

			
	void SetPlanPoints(PosStruct startPos, PosStruct endPos)//输入起始点位和结束点位的笛卡尔坐标
	{
		double startAngle[3], endAngle[3];

		startAngle[0] = startPos.yaw * PI / 180;
		startAngle[1] = startPos.pitch * PI / 180;
		startAngle[2] = startPos.roll * PI / 180;

		endAngle[0] = endPos.yaw * PI / 180;
		endAngle[1] = endPos.pitch * PI / 180;
		endAngle[2] = endPos.roll * PI / 180;

		mStartMatrixData[0] = cos(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) - sin(startAngle[0])*sin(startAngle[2]);
		mStartMatrixData[1] = -cos(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) - sin(startAngle[0])*cos(startAngle[2]);
		mStartMatrixData[2] = cos(startAngle[0])*sin(startAngle[1]);
		mStartMatrixData[3] = startPos.x / 1000;

		mStartMatrixData[4] = sin(startAngle[0])*cos(startAngle[1])*cos(startAngle[2]) + cos(startAngle[0])*sin(startAngle[2]);
		mStartMatrixData[5] = -sin(startAngle[0])*cos(startAngle[1])*sin(startAngle[2]) + cos(startAngle[0])*cos(startAngle[2]);
		mStartMatrixData[6] = sin(startAngle[0])*sin(startAngle[1]);
		mStartMatrixData[7] = startPos.y / 1000;

		mStartMatrixData[8] = -sin(startAngle[1])*cos(startAngle[2]);
		mStartMatrixData[9] = sin(startAngle[1])*sin(startAngle[2]);
		mStartMatrixData[10] = cos(startAngle[1]);
		mStartMatrixData[11] = startPos.z / 1000;

		mStartMatrixData[12] = 0;
		mStartMatrixData[13] = 0;
		mStartMatrixData[14] = 0;
		mStartMatrixData[15] = 1;

		mEndMatrixData[0] = cos(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) - sin(endAngle[0])*sin(endAngle[2]);
		mEndMatrixData[1] = -cos(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) - sin(endAngle[0])*cos(endAngle[2]);
		mEndMatrixData[2] = cos(endAngle[0])*sin(endAngle[1]);
		mEndMatrixData[3] = endPos.x / 1000;

		mEndMatrixData[4] = sin(endAngle[0])*cos(endAngle[1])*cos(endAngle[2]) + cos(endAngle[0])*sin(endAngle[2]);
		mEndMatrixData[5] = -sin(endAngle[0])*cos(endAngle[1])*sin(endAngle[2]) + cos(endAngle[0])*cos(endAngle[2]);
		mEndMatrixData[6] = sin(endAngle[0])*sin(endAngle[1]);
		mEndMatrixData[7] = endPos.y / 1000;

		mEndMatrixData[8] = -sin(endAngle[1])*cos(endAngle[2]);
		mEndMatrixData[9] = sin(endAngle[1])*sin(endAngle[2]);
		mEndMatrixData[10] = cos(endAngle[1]);
		mEndMatrixData[11] = endPos.z / 1000;

		mEndMatrixData[12] = 0;
		mEndMatrixData[13] = 0;
		mEndMatrixData[14] = 0;
		mEndMatrixData[15] = 1;

		double angle1, angle2, angle3, angle4, angle5, angle6;
		HLRobot::SetRobotEndPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll);
		HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);

		mJointAngleBegin[0] = angle1;
		mJointAngleBegin[1] = angle2;
		mJointAngleBegin[2] = angle3;
		mJointAngleBegin[3] = angle4;
		mJointAngleBegin[4] = angle5;
		mJointAngleBegin[5] = angle6;

		HLRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll);
		HLRobot::GetJointAngles(angle1, angle2, angle3, angle4, angle5, angle6);
		mJointAngleEnd[0] = angle1;
		mJointAngleEnd[1] = angle2;
		mJointAngleEnd[2] = angle3;
		mJointAngleEnd[3] = angle4;
		mJointAngleEnd[4] = angle5;
		mJointAngleEnd[5] = angle6;

		start_x = startPos.x; start_y = startPos.y; start_z = startPos.z;
		start_yaw = startPos.yaw; start_pitch = startPos.pitch; start_roll = startPos.roll;
		end_x = endPos.x; end_y = endPos.y; end_z = endPos.z;
		end_yaw = endPos.yaw; end_pitch = endPos.pitch; end_roll = endPos.roll;
		sp = startPos;
		ep = endPos;

	}


	void SetProfile(double acc, double vel, double dec)			//设置运动参数，速度、加速度和减速度
	{
		mVel = vel;
		mAcc = acc;
		mDec = dec;
	}


	void SetProfileJ(double acc, double vel, double dec)
	{
		JmVel = vel;
		JmAcc = acc;
		JmDec = dec;
	}


	void GetPlanPoints(vector<string>& result)										//获取轨迹规划后离散点位	
	{

		//计算每个轴旋转的角度
		double deg[10];
		int i;
		for (i = 1; i <= 6; i++)deg[i] = mJointAngleEnd[i] - mJointAngleBegin[i];

		//计算每个轴移动到终止点所需要时间
		LFPB plan[10];
		for(i = 1; i <= 6; i++)plan[i].init(mAcc, mVel, mDec, deg[i]);
		double total_time = 0;
		for(i = 1; i <= 6; i++)total_time = max(total_time, plan[i].t3);
		total_time += 0.01;

		double t = 0, pose[10], angle[10];
		while(t < total_time)
		{
			string str = "", temp = ""  ;
			for(i = 1; i <= 6; i++)angle[i] = mJointAngleBegin[i] + plan[i].get_pos(t);
			for(i = 1; i <= 5; i++)str = str + to_string(angle[i]) + " ";
			str = str + to_string(angle[6]);

			result.push_back(str);
			t += mSampleTime;
		}
		//完成代码;
		return;
	}


	double GetPlanPoints_line(vector<string>& result)										//获取轨迹规划后离散点位
	{
		double angle[10];
		double pose[10];
		//HLRobot::SetRobotEndPos(angle);
		int i;
		double dx, dy, dz, dis;
		double nx, ny, nz;
		dx = end_x - start_x; dy = end_y - start_y; dz = end_z - start_z;
		dis = sqrt (dx*dx+dy*dy+dz*dz);
		nx = dx/dis; ny = dy/dis; nz = dz/dis;

		LFPB plan_space;
		LFPB plan_zyz[4];

		plan_space.init(mAcc, mVel, mDec, dis);
		plan_zyz[1].init(JmAcc, JmVel, JmDec, end_yaw-start_yaw);
		plan_zyz[2].init(JmAcc, JmVel, JmDec, end_pitch-start_pitch);
		plan_zyz[3].init(JmAcc, JmVel, JmDec, end_roll-start_roll);

		double total_time = plan_space.t3;
		for (i = 1; i <= 3; i++)total_time = max(total_time, plan_zyz[i].t3);
		total_time += 0.01;

		double t = 0, L;
		int count = 0;
		while(t < total_time)
		{
			string str = "", temp = "";
			L = plan_space.get_pos(t);
			pose[1] = start_x + L*nx;
			pose[2] = start_y + L*ny;
			pose[3] = start_z + L*nz;
			pose[4] = start_yaw + plan_zyz[1].get_pos(t);
			pose[5] = start_pitch + plan_zyz[2].get_pos(t);
			pose[6] = start_roll + plan_zyz[3].get_pos(t);
			//规划的空间坐标点
			for (i = 1; i <= 5; i++)temp = temp + to_string(pose[i]) + " ";
			temp = temp + to_string(pose[i]);
			//if (rand() % 1024 == 0)cout << "空间坐标：" << temp << endl;	//随机打印部分结果，表示还程序运行中

			HLRobot::SetPose(pose);
			HLRobot::GetAngles(angle);
			
			//反解的关节坐标点
			for(i = 1; i <= 5; i++)str = str + to_string(angle[i]) + " ";
			str = str + to_string(angle[i]);
			//if(rand()%1024 == 0)cout << "关节坐标：" << str << endl;

			result.push_back(str);
			t += mSampleTime;  count++;

		}
		return total_time;
	} 

};

