#ifndef _CoordinateTrans_H__
#define _CoordinateTrans_H__
#include "main.h"
void cameraCoorToWorld(float &camera_x,float &camera_y,float &camera_z,cv::Mat & worrldCoor,int pillarIdx);


void calRotationMatrixbyGround();
void calRotationMatrixbyGuard();

//测试用
void UpdateCameraCoordinate(cv::Mat &xImage,cv::Mat &yImage,cv::Mat &zImage,cv::Mat & r_Image);
void UpdateGuardPoints(cv::Mat &guard_points);

void UpdateGuardPoints_Final(cv::Mat &guardPoints);

void CalFullRotationMatrix();

void CalCameraWorldCoordinate(int pillarId);
#endif