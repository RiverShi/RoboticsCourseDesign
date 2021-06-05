#pragma once
#include <vector>
#include "RecDect.h"


//可在此设计相关功能函数，如机器人的、连接初始化、机器人的运动、积木的抓取顺序等

//以机器人连接为例
void initialization();

void RobotConnect();

void close();

void Robot_init();

void Robot_to_initial_pose();

void PPB_run(string filename, int sleeptime);

void PPB_stop();


void PPB_suck(int i);

void suckin();
void suckout();
void suckoff();