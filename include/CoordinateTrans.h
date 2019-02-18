#ifndef _CoordinateTrans_H__
#define _CoordinateTrans_H__
#include "main.h"
void cameraCoorToWorld(float &camera_x,float &camera_y,float &camera_z,cv::Mat & worrldCoor,int pillarIdx);


void calPYRbyGround();
void calPYRbyGuard();
#endif