#pragma once
#include <vector>
using namespace std;

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
	PosStruct(double x, double y, double z, double yaw, double pitch, double roll):x(x), y(y), z(z), yaw(yaw), pitch(pitch), roll(roll){}
	
	void assign(double xx, double yy, double zz, double yyaw, double ppitch, double rroll);
	
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
	CHLMotionPlan();

	virtual ~CHLMotionPlan();

	void SetSampleTime(double sampleTime);							//设置采样时间
	void SetPlanPoints(PosStruct startPos, PosStruct endPos);		//输入起始点位和结束点位的笛卡尔坐标
	void SetProfile(double vel, double acc, double dec);			//设置运动参数，速度、加速度和减速度
	void SetProfileJ(double vel, double acc, double dec);
	void GetPlanPoints(vector<string>& result);											//获取轨迹规划后离散点位	
	double GetPlanPoints_line(vector<string>& result);										//获取轨迹规划后离散点位
};

