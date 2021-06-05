
#include<winsock.h>
#include <conio.h>
#pragma comment(lib,"ws2_32.lib")
using namespace std;
#pragma comment(lib, "WS2_32.lib")
#include "Function.h"
#include "RecDect.h"
#include<iostream>
#include"MotionPlan\MotionPlan.h"
#include"FtpControl.h"



//定义长度变量
int send_len = 0;
int recv_len = 0;

//定义发送缓冲区和接受缓冲区
char send_buf[100] = {};
char recv_buf[200] = {};

string recvstr;
//定义服务端套接字，接受请求套接字
SOCKET s_server;
//服务端地址客户端地址
SOCKADDR_IN server_addr;


void initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确！" << endl;
	}
	//填充服务端地址信息
}

void RobotConnect()
{
	initialization();
	//填充服务端信息
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
	server_addr.sin_port = htons(2090);
	//创建套接字
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}
}

void close()
{
	//关闭套接字
	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
}

void SendCmd(string cmd, string hint, int sleeptime = 200)
{
    send_len = send(s_server, cmd.c_str(), 100, 0);
	recv_len = recv(s_server, recv_buf, 100, 0);
	cout << hint << '\t' <<recv_buf << endl;
	memset(recv_buf, '\0', sizeof(recv_buf));
	Sleep(sleeptime);
}

void Robot_init()
{
    SendCmd("[1# System.Login 0]",          "Login");
    SendCmd("[2# Robot.PowerEnable 1,1]",   "PowerEnable");
    SendCmd("[3# System.Abort 1]",          "Abort");
    SendCmd("[4# System.Start 1]",          "Start");
    SendCmd("[5# Robot.Home 1]",            "Home");
    SendCmd("[6# System.Auto 1]",           "Auto");
    SendCmd("[7# System.Speed 30]",         "Speed");
    SendCmd("[8# IO.Set DOUT(20103),0]",    "吸气关");
    SendCmd("[9# IO.Set DOUT(20104),0]",    "吹气关");  
	cout << endl;
}

void Robot_to_initial_pose()
{
    std::cout << "go to initial pose" << endl;
    SendCmd("[1# Robot.Frame 1,1]",                     "关节坐标系");
    SendCmd("[2# LocationJ Jit]",                          "定义点");
	//SendCmd("[3# P=419.973,0,733.737,-137,180,0]",      "赋值点");
	SendCmd("[3# Jit=0,-4.66,101.747,0,82.914,-42.0]",      "赋值点");
    SendCmd("[4# Move.Joint Jit]",                         "运动",    5000);
	cout << endl;
}

//void PPB_run(string filename)
//{
//	string runningfile = "[3# PPB.ReadFile 1, " + filename + "]";
//
//	SendCmd("[1# Robot.Frame 1,1]",                     "关节坐标系");
//    SendCmd("[2# PPB.Enable 1,1]",                      "PPB使能");
//    SendCmd(runningfile.c_str(),						"读取文件");
//    SendCmd("[4# PPB.J2StartPoint 1,0,1]",              "运动到起点", 	1000);
//	SendCmd("[4# PPB.Run 1]",                         	"运动",    		8000);
//	cout << endl;
//}

void PPB_run(string filename, int sleeptime)
{
	cout << "上传文件 : " << filename << endl;
	FtpControl::Upload("192.168.10.101", "data", filename.c_str(), "severdata.txt");
	SendCmd("[2# PPB.Enable 1,1]", "PPB使能");
	SendCmd("[1# Robot.Frame 1,1]", "关节坐标系");
	SendCmd("[3# PPB.ReadFile 1, /data/severdata.txt]", "读取文件");
	SendCmd("[4# PPB.J2StartPoint 1,0,1]", "运动到起点", 1000);
	SendCmd("[4# PPB.Run 1]", "运动");
	cout << endl;
	Sleep(sleeptime);
}

void PPB_stop()
{
	SendCmd("[1# PPB.Enable 1,0]", "退出PPB模式");
}




void suckin()
{
	SendCmd("[7# IO.Set DOUT(20103),0]", "");
	SendCmd("[7# IO.Set DOUT(20104),1]", "");
	Sleep(500);
}

void suckout()
{
	SendCmd("[7# IO.Set DOUT(20103),1]", "");
	SendCmd("[7# IO.Set DOUT(20104),0]", "");
	Sleep(500);
}

void suckoff()
{
	SendCmd("[7# IO.Set DOUT(20103),0]", "");
	SendCmd("[7# IO.Set DOUT(20104),0]", "");
	Sleep(500);
}



/*
// void PPB_suck(int i)
// {
// 	//气阀控制
// 	if (i == 1 || 3 || 5 || 7 || 9 || 11)
// 	{
// 		cout << "吸气开启，等待0.8s" << endl;
// 		//吸气，一直吸气
// 		send_len = send(s_server, "[7# IO.Set DOUT(20103),0]", 100, 0);
// 		recv_len = recv(s_server, recv_buf, 100, 0);
// 		cout << recv_buf << endl;
// 		memset(recv_buf, '\0', sizeof(recv_buf));

// 		send_len = send(s_server, "[8# IO.Set DOUT(20104),1]", 100, 0);
// 		recv_len = recv(s_server, recv_buf, 100, 0);
// 		cout << recv_buf << endl;
// 		memset(recv_buf, '\0', sizeof(recv_buf));

// 		Sleep(800);
// 	}
// 	else if(i == 2 || 4 || 6 || 8 || 10 || 12)
// 	{
// 		cout << "吸气关闭，放气开启0.8s，然后关闭放气" << endl;
// 		//放气0.8s然后关闭放气
// 		send_len = send(s_server, "[7# IO.Set DOUT(20103),1]", 100, 0);
// 		recv_len = recv(s_server, recv_buf, 100, 0);
// 		cout << recv_buf << endl;
// 		memset(recv_buf, '\0', sizeof(recv_buf));

// 		send_len = send(s_server, "[8# IO.Set DOUT(20104),0]", 100, 0);
// 		recv_len = recv(s_server, recv_buf, 100, 0);
// 		cout << recv_buf << endl;
// 		memset(recv_buf, '\0', sizeof(recv_buf));

// 		Sleep(800);
// 		send_len = send(s_server, "[7# IO.Set DOUT(20103),0]", 100, 0);
// 		recv_len = recv(s_server, recv_buf, 100, 0);
// 		cout << recv_buf << endl;
// 		memset(recv_buf, '\0', sizeof(recv_buf));;
// 	}
// 	else 
// 	{
// 		cout << "其他阶段，气阀不变" << endl;
// 	}
// }
*/
