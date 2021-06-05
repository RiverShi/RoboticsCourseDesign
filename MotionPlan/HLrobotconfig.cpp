
#include "HLrobotconfig.h"
#include "eigen3/Eigen/Dense"
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;

const double eps = 1e-6;

namespace HLRobot
{
	//初始化TransMatrix
	double mTransMatrix[16] {0};

	//只使用一种姿态
	bool mConfig[3] = { 0, 0, 0 };

	int L1 = 491, L2 = 450, L3 = 450, L4 = 84;
	Eigen::Vector3d w1(0, 0, 1), w2(0, 1, 0), w3(0, 1, 0), w4(0, 0, 1), w5(0, 1, 0), w6(0, 0, 1);
	Eigen::Vector3d q1(0, 0, L1), q2(0, 0, L1), q3(0, 0, L1 + L2), q4(0, 0, L1 + L2 + L3), q5(0, 0, L1 + L2 + L3), q6(0, 0, L1 + L2 + L3 + L4);
	Eigen::Vector3d v1 = q1.cross(w1), v2 = q2.cross(w2), v3 = q3.cross(w3);
	Eigen::Vector3d v4 = q4.cross(w4), v5 = q5.cross(w5), v6 = q6.cross(w6);
	

	Eigen::Vector3d wx(1, 0, 0);
	Eigen::Vector3d wy(0, 1, 0);
	Eigen::Vector3d wz(0, 0, 1);

	double st0[16] = { -1, 0, 0, 0,
						0,-1, 0, 0,
						0, 0, 1, 0,
						0, 0, L1 + L2 + L3 + L4, 1
	};	//有个按列存储的巨坑

	Eigen::Matrix4d gst0(st0);

	void Subproblem_1(const Eigen::Vector3d &w, const Eigen::Vector3d &v,
		const Eigen::Vector3d &p, const Eigen::Vector3d &q,
		double &theta);

	int Subproblem_2(const Eigen::Vector3d &w1, const Eigen::Vector3d &v1,
		const Eigen::Vector3d &w2, const Eigen::Vector3d &v2,
		const Eigen::Vector3d &r,
		const Eigen::Vector3d &p, const Eigen::Vector3d &q,
		double &theta1, double &theta2, bool mconfig);

	void Subproblem_3(const Eigen::Vector3d &w, const Eigen::Vector3d &v,
		const Eigen::Vector3d &p, const Eigen::Vector3d &q,
		double d, double &theta, bool mconfig);

	Eigen::Matrix3d GetRot(const Eigen::Vector3d &w, double theta);
	Eigen::Matrix4d GetTrans_Rot(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta);
	Eigen::Matrix4d GetTrans_Mov(const Eigen::Vector3d &v, double theta);
	Eigen::Matrix3d ZYZtoR(double a, double b, double c);

	
	
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll)
	{
		Eigen::Matrix3d R = ZYZtoR(yaw, pitch, roll);
		Eigen::Matrix4d Endpos;
		Endpos << R(0, 0), R(0, 1), R(0, 2), x,
				  R(1, 0), R(1, 1), R(1, 2), y,
				  R(2, 0), R(2, 1), R(2, 2), z,
				  0, 0, 0, 1;

		for (int i = 0; i < 16; i++)mTransMatrix[i] = Endpos(i);
	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, double &angle5, double &angle6)
	{
		double angle[6];
		robotBackward(mTransMatrix, mConfig, angle);
		angle1 = angle[0];	angle2 = angle[1];
		angle3 = angle[2];	angle4 = angle[3];
		angle5 = angle[4];  angle6 = angle[5];
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4, double angle5, double angle6)
	{
		double angle[6] = { angle1, angle2, angle3, angle4, angle5, angle6 };
		robotForward(angle, mTransMatrix, mConfig);
	}

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll )
	{
		Eigen::Matrix4d Endpos;
		for (int i = 0; i < 16; i++)Endpos(i) = mTransMatrix[i];
		Eigen::Matrix4d &g = Endpos;

		x = Endpos(0, 3);  y = Endpos(1, 3);	z = Endpos(2, 3);
		//计算zyz欧拉角
		pitch = atan2(sqrt(g(2, 0)*g(2, 0) + g(2, 1)*g(2, 1)), g(2, 2));
		yaw = atan2(g(1, 2) / sin(pitch), g(0, 2) / sin(pitch));
		roll = atan2(g(2, 1) / sin(pitch), -g(2, 0) / sin(pitch));

		pitch *= 180 / PI;
		yaw *= 180 / PI;
		roll *= 180 / PI;
	}


	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

				config[3]：姿态，六轴机器人对应有8种姿态（即对应的逆运动学8个解），为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

	OUTPUTS:    theta[6] 6个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	void robotBackward(const double* TransVector, bool* mconfig, double* theta)
	{
		Eigen::Matrix4d Endpos;
		for (int i = 0; i < 16; i++)Endpos(i) = TransVector[i];
		//cout <<"Endpos:\n" << Endpos << endl;

		Eigen::Matrix4d g = Endpos * gst0.inverse();

		Eigen::Vector3d pw1(0, 0, L1+L2+L3), pw2(0, 0, L1);
		Eigen::Vector4d ph1, ph2, ph3, ph4, temp;
		Eigen::Vector3d px, qx;

		ph1 << pw1, 1;
		ph2 << pw2, 1;
		temp = g * ph1;
		double d = (g*ph1 - ph2).norm();
		px = pw1; qx = pw2;
		Subproblem_3(w3, v3, px, qx, d, theta[2], mconfig[0]);

		ph3 = GetTrans_Rot(w3, v3, theta[2]) * ph1;
		ph4 = g * ph1;
		px << ph3(0), ph3(1), ph3(2);
		qx << ph4(0), ph4(1), ph4(2);
		Eigen::Vector3d r(0, 0, L1);
		Subproblem_2(w1, v1, w2, v2, r, px, qx, theta[0], theta[1], mconfig[1]);

		Eigen::Matrix4d g1 = GetTrans_Rot(w3, v3, -theta[2]) * GetTrans_Rot(w2, v2, -theta[1])
								* GetTrans_Rot(w1, v1, -theta[0]) * g;
		pw1 << 0, 0, L1 + L2 + L3 + L4;
		ph1 << pw1, 1;
		ph2 = g1 * ph1;
		px = pw1;
		qx << ph2(0), ph2(1), ph2(2);
		r << 0, 0, L1 + L2 + L3;
		Subproblem_2(w4, v4, w5, v5, r, px, qx, theta[3], theta[4], mconfig[2]);

		Eigen::Matrix4d g2 = GetTrans_Rot(w5, v5, -theta[4]) * GetTrans_Rot(w4, v4, -theta[3]) * g1;
		pw1 << 1, 1, 1;		//取w6转轴外一点
		ph1 << pw1, 1;
		ph2 = g2 * ph1;
		px = pw1;
		qx << ph2(0), ph2(1), ph2(2);
		Subproblem_1(w6, v6, px, qx, theta[5]);
				
	}



	/********************************************************************
	ABSTRACT:	机器人正运动学
	
	INPUTS:		q[6]: 6个关节角, 单位为弧度
	
	OUTPUTS:	config[3]：姿态，六轴机器人对应有8种姿态，为了安全，
				实验室中我们只计算一种即可。config用来作为选解的标志数。

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米
	
	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, double* TransVector, bool* mconfig)
	{
		Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
		Eigen::Matrix4d g1, g2, g3, g4, g5, g6;
		//旋转矩阵
		
		g1 = GetTrans_Rot(w1, v1, q[0]);
		g2 = GetTrans_Rot(w2, v2, q[1]);
		g3 = GetTrans_Rot(w3, v3, q[2]);
		g4 = GetTrans_Rot(w4, v4, q[3]);
		g5 = GetTrans_Rot(w5, v5, q[4]);
		g6 = GetTrans_Rot(w6, v6, q[5]);

		Eigen::Matrix4d g = g1 * g2 * g3 * g4 * g5 * g6;
		Eigen::Matrix4d Endpos = g * gst0;
		for (int i = 0; i < 16; i++)TransVector[i] = Endpos(i);
		//cout << "forward:\n" << Endpos << endl;
	}





	Eigen::Matrix3d GetRot(const Eigen::Vector3d &w, double theta)
	{
		Eigen::Matrix3d wh;
		wh <<	   0, -w(2),  w(1),
				w(2),     0, -w(0),
			   -w(1),  w(0),     0;

		Eigen::Matrix3d I3 = Eigen::MatrixXd::Identity(3, 3);

		theta = theta * PI / 180;		//角度化为弧度
		Eigen::Matrix3d res = I3 + wh * sin(theta) + wh * wh*(1 - cos(theta));

		return res;
	}

	Eigen::Matrix4d GetTrans_Rot(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta)
	{
		Eigen::Matrix3d wh;
		wh <<    0, -w(2),  w(1),
			  w(2),     0, -w(0),
			 -w(1),  w(0),     0;

		Eigen::Matrix3d I3 = Eigen::MatrixXd::Identity(3, 3);

		theta = theta * PI / 180;		//角度化为弧度
		Eigen::Matrix3d R = I3 + wh * sin(theta) + wh * wh*(1 - cos(theta));
		Eigen::MatrixXd P = (I3 - R)*(w.cross(v)) + w * w.transpose()*v*theta;

		Eigen::Matrix4d res;
		res <<	R(0, 0), R(0, 1), R(0, 2), P(0),
				R(1, 0), R(1, 1), R(1, 2), P(1),
				R(2, 0), R(2, 1), R(2, 2), P(2),
				0, 0, 0, 1;
		return res;
	}

	Eigen::Matrix4d GetTrans_Mov(const Eigen::Vector3d &v, double theta)
	{
		Eigen::Vector3d P = v * theta;
		Eigen::Matrix4d res;
		res <<	1, 0, 0, P(0),
				0, 1, 0, P(1),
				0, 0, 1, P(2),
				0, 0, 0, 1;
		return res;
	}

	Eigen::Matrix3d ZYZtoR(double a, double b, double c)
	{
		Eigen::Matrix3d Res = GetRot(wz, a) * GetRot(wy, b) * GetRot(wz, c);
		return Res;
	}


	void Subproblem_1(	const Eigen::Vector3d &w, const Eigen::Vector3d &v,
						const Eigen::Vector3d &p, const Eigen::Vector3d &q,
						double &theta)
	{
		Eigen::Vector3d r = w.cross(v);
		Eigen::Vector3d u0 = p - r;
		Eigen::Vector3d v0 = q - r;
		Eigen::Vector3d u1 = u0 - w * w.transpose()*u0;
		Eigen::Vector3d v1 = v0 - w * w.transpose()*v0;

		theta = atan2(w.transpose()*(u1.cross(v1)), u1.transpose()*v1);

		theta = theta * 180 / PI;
		if (theta > 180)theta -= 180;
		if (theta < -180)theta += 180;
	}

	int Subproblem_2(	const Eigen::Vector3d &w1, const Eigen::Vector3d &v1,
						const Eigen::Vector3d &w2, const Eigen::Vector3d &v2,
						const Eigen::Vector3d &r,
						const Eigen::Vector3d &p,  const Eigen::Vector3d &q,
						double &theta1,  double &theta2,  bool mconfig)
	{
		Eigen::Vector3d u = p - r;
		Eigen::Vector3d v = q - r;
		Eigen::Vector3d z, c;

		if (abs(w1.dot(w2) - 1) < eps) 	//两轴重合
		{
			Subproblem_1(w1, v1, p, q, theta1);
			return 2;
		}

		double a = (w1.dot(w2)*w2.dot(u) - w1.dot(v)) / (w1.dot(w2) * w1.dot(w2) - 1);
		double b = (w1.dot(w2)*w1.dot(v) - w2.dot(u)) / (w1.dot(w2) * w1.dot(w2) - 1);
		double gamma2 = (u.norm()*u.norm() - a * a - b * b - 2 * a*b*w1.dot(w2)) / pow(w1.cross(w2).norm(), 2);
		double gamma;

		if (gamma2 < 0)return 3;	//无解

		if (gamma2 < eps)
		{
			gamma = 0;
			z = a * w1 + b * w2;
			c = z + r;
			Subproblem_1(w1, v1, p, c, theta1);
			Subproblem_1(w2, v2, c, q, theta2);
			return 1;
		}

		gamma = sqrt(gamma2);
		if (mconfig == 0) gamma = -gamma;	//mconfig = 0, 取负数值；
		z = a * w1 + b * w2 + gamma * w1.cross(w2);
		c = z + r;
		Subproblem_1(w1, v1, c, q, theta1);
		Subproblem_1(w2, v2, p, c, theta2);
		return 0;
		
	}

	void Subproblem_3(const Eigen::Vector3d &w, const Eigen::Vector3d &v,
		const Eigen::Vector3d &p, const Eigen::Vector3d &q,
		double d, double &theta, bool mconfig)
	{
		Eigen::Vector3d r = w.cross(v);
		Eigen::Vector3d u0 = p - r;
		Eigen::Vector3d v0 = q - r;
		Eigen::Vector3d u1 = u0 - w * w.transpose()*u0;
		Eigen::Vector3d v1 = v0 - w * w.transpose()*v0;

		double theta0 = atan2(w.transpose()*(u1.cross(v1)), u1.transpose()*v1);
		double Lu1 = u1.norm(), Lv1 = v1.norm();
		double q_ver = w.transpose()*(p - q);
		double d1_square = d * d - q_ver * q_ver;
		double phi = acos((Lu1*Lu1 + Lv1 * Lv1 - d1_square) / (2 * Lu1*Lv1));

		if (mconfig)theta = theta0 + phi;	//mconfig == 1 时取大角度 
		else theta = theta0 - phi;

		theta = theta * 180 / PI;
		if (theta > 180)theta -= 180;
		if (theta < -180)theta += 180;
	}






















}
