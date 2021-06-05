#pragma once

#define SetPose(arr)  SetRobotEndPos(arr[1], arr[2], arr[3], arr[4], arr[5], arr[6])
#define GetAngles(arr)  GetJointAngles(arr[1], arr[2], arr[3], arr[4], arr[5], arr[6])
#define	SetAngles(arr)  SetRobotJoint(arr[1], arr[2], arr[3], arr[4], arr[5], arr[6])
#define GetPose(arr)  GetJointEndPos(arr[1], arr[2], arr[3], arr[4], arr[5], arr[6])

#define PI  3.1415926