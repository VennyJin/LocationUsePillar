#ifndef _MAIN_H
#define _MAIN_H

#include<sys/time.h>
//opencv
#include <opencv2/opencv.hpp>
//c
#include <time.h> 
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 

//C++
#include <iostream>
#include <fstream>
#include <string.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <stack>
#include <algorithm>
//openni
#include "OpenNI.h"

//my
#include "Featurefit.h"
#include "ImageProcess.h"
#include "CoordinateTrans.h"

using namespace std;
using namespace cv;
using namespace openni;

#define   IMAGE_X  320
#define   IMAGE_Y  240

#define TanX 0.413  //0.425
#define TanY 0.599911  //0.553  0.57

#define pi    3.14159265358979f

//保存部分参数
#define WRITEPARAMETERS
//保存视频
//#define SAVEVIDEO 1
//#define SAVEONLYRESULT 1

extern Mat g_src_img;
extern Mat g_dep_img;
//存储实际坐标
extern Mat g_x_img;
extern Mat g_y_img;
extern Mat g_z_img;

//存储到地面的距离
extern Mat g_dist_image;

//地面和挡板方程系数
extern float g_groundEquation[4];
extern float g_guardEquation[4];
//相机姿态角
//extern double g_pitch;  //绕x轴
//extern double g_yaw;  //绕y轴 
//extern double g_roll;  //绕z轴

//相机平面到地面的旋转矩阵
extern Mat rotateMatrixCameraToGround;
extern Mat rotateMatrixCameraToWorld;
extern Mat rotationMatrixCameraToGuard_pie;


//柱子相机坐标
extern Point3f PillarCameralocation[3];

extern Point3f PilarWorldlocation[3];

#ifdef SAVEVIDEO
//保存视频
extern VideoWriter rgbWriter;
extern VideoWriter pillarWriter;
#endif

#ifdef WRITEPARAMETERS
extern ofstream PillarWorldLocationFile;
extern ofstream GuardPlaneParameterFile;
extern ofstream GroundPlaneParameterFile;
#endif
#ifdef SAVEONLYRESULT
//保存视频

extern VideoWriter pillarWriter;
#endif


//将时间转化为字符串
String nowTime2String();

#endif