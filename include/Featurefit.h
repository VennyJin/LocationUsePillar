#ifndef _FEATUREFIT_H__
#define _FEATUREFIT_H__
#include "main.h"

void calDistanceToGround(cv::Mat & distImage);
void fitPlane(const cv::Mat &points, float planeCof[4]);

void dataAdjustToMat( cv::Mat & Ximg, cv::Mat &Yimg, cv::Mat &Zimg,cv::Mat &result );
void dataAdjustToGuard( cv::Mat & Ximg, cv::Mat &Yimg,cv:: Mat &Zimg,cv::Mat & Dimg,cv::Mat &result);
void dataAdjustToPillar(cv::Mat & Ximg, cv::Mat &Yimg,cv:: Mat &Zimg,std::vector<cv::Point3f>& points);
void calSegDistMat(cv::Mat &mask,float minHeight,float maxHeight,float minDepth,float maxDepth);
void getMapData();
void fit3DPillar(cv::Mat &pillarMask);
void fit3DGuard(cv::Mat &guardMask);
#endif