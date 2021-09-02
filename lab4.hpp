//#pragma once
//#include "MotionPlan.hpp"
//#include <iostream>>
//#include <fstream>
//#include "commom_define.h"
//
//using namespace std;
//
//void LFPB_Joint_line()
//{
//
//}
//
//double sampletime = 0.001;
//double v_ave = 80, w_ave = 40;
//
//double Lift_cubic(PosStruct pos, double height, vector<string>& result)
//{
//	cout << "Lift  height = " << height << endl;
//	//*****调试*****************************************************************************
//		//Aout << "Lift  height = " << height << endl;
//		//Bout << "Lift  height = " << height << endl;
//	//*****************************************************************************************
//	Cubic plan;
//	double Td = fabs(height) / v_ave + 0.1;
//	plan.init(pos.z, 0, pos.z + height, 0, Td);
//	Point_info point;
//	point.assign(pos);
//
//	string Joint = "", spatial = "";
//
//	//*****调试********************************************************************************
//		//HLRobot::SetPose(point.pose);
//		//HLRobot::GetAngles(point.angle);
//		//for (int i = 1; i <= 6; i++)Joint = Joint + to_string(point.angle[i]) + " ";
//		//result.push_back(Joint);
//
//		//for (int i = 1; i <= 6; i++)spatial = spatial + to_string(point.pose[i]) + " ";
//		//Aout <<"start pos Joint :" << Joint << endl;  
//		//Bout << "start pos spatial :" << spatial << endl;
//	//******************************************************************************************
//
//	double time = 0;
//	while (time <= Td)
//	{
//		Joint = "", spatial = "";
//		point.z = plan.get_pos(time);
//		HLRobot::SetPose(point.pose);
//		HLRobot::GetAngles(point.angle);
//		for (int i = 1; i <= 6; i++)Joint = Joint + to_string(point.angle[i]) + " ";
//		result.push_back(Joint);
//		time += sampletime;
//		//****调试*********************************************************************************
//		for (int i = 1; i <= 6; i++)spatial = spatial + to_string(point.pose[i]) + " ";
//		Aout << Joint << endl;  Bout << spatial << endl;
//		//cout << spatial << endl;
////*******************************************************************************************
//	}
//	cout << endl;
//
//	//*****调试**********************************************************************************
//		//Aout <<  endl;  Bout  << endl;
//
//	//*******************************************************************************************
//
//	return Td;
//}
//
//
//double Move_cubic(PosStruct startpos, PosStruct endpos, vector<string>& result)
//{
//	cout << "Move" << endl;
//	//*******************************************************************************************
//		//Aout << "Move" << endl;
//		//Bout << "Move" << endl;
//	//*******************************************************************************************
//	Cubic plan[8];
//	Point_info s, t;
//	s.assign(startpos); t.assign(endpos);
//
//	string Joint = "", spatial = "";
//
//	//*******************************************************************************************
//		//HLRobot::SetPose(s.pose);
//		//HLRobot::GetAngles(s.angle);
//		//for (int i = 1; i <= 6; i++)Joint = Joint + to_string(s.angle[i]) + " ";
//		//result.push_back(Joint);
//
//		//for (int i = 1; i <= 6; i++)spatial = spatial + to_string(s.pose[i]) + " ";
//		//Aout << "start pos Joint :" << Joint << endl;
//		//Bout << "start pos spatial :" << spatial << endl;
//
//
//		//Joint = "", spatial = "";
//		//HLRobot::SetPose(t.pose);
//		//HLRobot::GetAngles(t.angle);
//		//for (int i = 1; i <= 6; i++)Joint = Joint + to_string(t.angle[i]) + " ";
//
//		//for (int i = 1; i <= 6; i++)spatial = spatial + to_string(t.pose[i]) + " ";
//		//Aout << "end pos Joint :" << Joint << endl;
//		//Bout << "end pos spatial :" << spatial << endl;
//	//*******************************************************************************************
//
//
//	double Td = 0;
//	int i;
//	for (i = 1; i <= 3; i++) Td = max(Td, fabs(t.pose[i] - s.pose[i]) / v_ave);
//	Td = max(Td, fabs(t.pose[4] - s.pose[4]) / w_ave);
//	Td += 0.1;
//	for (i = 1; i <= 6; i++)plan[i].init(s.pose[i], 0, t.pose[i], 0, Td);
//	Point_info point;
//	point.pose[5] = 180; point.pose[6] = 138;	//5、6轴固定不变
//	double time = 0;
//	while (time <= Td)
//	{
//		Joint = "", spatial = "";
//		for (i = 1; i <= 4; i++)point.pose[i] = plan[i].get_pos(time);
//		HLRobot::SetPose(point.pose);
//		HLRobot::GetAngles(point.angle);
//		for (int i = 1; i <= 6; i++)Joint = Joint + to_string(point.angle[i]) + " ";
//		result.push_back(Joint);
//		time += sampletime;
//		//*******************************************************************************************
//		for (int i = 1; i <= 6; i++)spatial = spatial + to_string(point.pose[i]) + " ";
//		Aout << Joint << endl;  Bout << spatial << endl;
//		//cout << spatial << endl;
//
//
////*******************************************************************************************
//	}
//
//	cout << endl;
//	//*******************************************************************************************
//		//Aout <<  endl;  Bout  << endl;
//
//	//*******************************************************************************************
//	return Td;
//}
//
//double plan_LFPB(PosStruct startpos, PosStruct endpos, int index)
//{
//	double time = 0;
//	double H = max(startpos.z, endpos.z) + 40;
//	PosStruct H1 = startpos, H2 = endpos;
//	H1.z = H2.z = H;
//
//	plan_result.clear();
//	time += Lift_cubic(startpos, H - startpos.z, plan_result);
//	time += Move_cubic(H1, H2, plan_result);
//	time += Lift_cubic(H2, endpos.z - H, plan_result);
//
//	ofstream fout;
//	fout.open("data/data_cubic" + to_string(index) + ".txt");
//	for (auto& str : plan_result)fout << str << endl;
//	fout.close();
//	plan_result.clear();
//
//	cout << "plan " + to_string(index) + " finish" << endl;
//
//	return time;
//}