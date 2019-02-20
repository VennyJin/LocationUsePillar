#include "ImageProcess.h"



/*
 * 
 * 功能：用以提取用来拟合平面的点
 * 
 * 
 * */
void getPlaneFitPoint(int roix,int roiy,cv::Rect & fitPoints,int roiwidth,int roiheight)
{

	fitPoints=Rect(roix-roiwidth/2,roiy-roiheight/2,roiwidth,roiheight);
	
}




bool groundFit()
{
	const int maxIterNum=5;  //最大移动次数
	int iterNum=0;	   //初始化移动次数
	
	int moveVStep=20;  //垂直移动像素距离 正值图片下方
	int moveHStep=20;	//水平一定距离	负值图片左右
	int c_Roix=160; //初始化扫描中心的x坐标
	int c_Roiy=200; //初始化扫描点中心yz坐标
	
	int roiwidth=25;//拟合窗口宽度
	int roiheight=25;//拟合窗口高度
	
	
	
	
	//初始化平面方程
	for(int i=0;i<4;i++)
	{
		g_groundEquation[i]=0;
	}
	float pitchGround=90.0;
	//得到进行平面拟合的区域
	Rect box;
	Mat fitGroundPoints[3];
	//getPlaneFitPoint(160,200,box,15,15);
	
	//setFitarea
	int fitAreaCoordinate[maxIterNum][2];
	fitAreaCoordinate[0][0]=c_Roix;
	fitAreaCoordinate[0][1]=c_Roiy;
	fitAreaCoordinate[1][0]=c_Roix-moveHStep;
	fitAreaCoordinate[1][1]=c_Roiy-moveVStep;
	fitAreaCoordinate[2][0]=c_Roix+moveHStep;
	fitAreaCoordinate[2][1]=c_Roiy+moveVStep;
	fitAreaCoordinate[3][0]=c_Roix-moveHStep;
	fitAreaCoordinate[3][1]=c_Roiy+moveVStep;
	fitAreaCoordinate[4][0]=c_Roix+moveHStep;
	fitAreaCoordinate[4][1]=c_Roiy-moveVStep;
	float minPitch=91;
	int minIndex=0;
	/*while(pitchGround>40&&iterNum<maxIterNum)
	{
		iterNum++;
		
		//进行平面拟合的数据
		//getPlaneFitPoint(c_Roix+moveHStep*iterNum,c_Roiy+moveVStep*iterNum,box,roiwidth,roiheight);
		getPlaneFitPoint(fitAreaCoordinate[iterNum][0],fitAreaCoordinate[iterNum][1],box,roiwidth,roiheight);
		fitGroundPoints[0]=g_x_img(box);
		fitGroundPoints[1]=g_y_img(box);
		fitGroundPoints[2]=g_z_img(box);
		
		//拟合平面
		Mat adjustedFitGroundPoints(fitGroundPoints[0].rows*fitGroundPoints[0].cols,3,CV_32FC1);
		dataAdjustToMat(fitGroundPoints[0],fitGroundPoints[1],fitGroundPoints[2],adjustedFitGroundPoints);
		fitPlane(adjustedFitGroundPoints,g_groundEquation);
		
		
		pitchGround=acos(abs(g_groundEquation[1]))/pi*180 ;
		
	}	*/
	for(int i=0;i<1;i++)
	//for(int i=0;i<maxIterNum;i++)
	{
		iterNum++;
		
		//进行平面拟合的数据
		//getPlaneFitPoint(c_Roix+moveHStep*iterNum,c_Roiy+moveVStep*iterNum,box,roiwidth,roiheight);
		getPlaneFitPoint(fitAreaCoordinate[i][0],fitAreaCoordinate[i][1],box,roiwidth,roiheight);
		
		fitGroundPoints[0]=g_x_img(box);
		fitGroundPoints[1]=g_y_img(box);
		fitGroundPoints[2]=g_z_img(box);
		
		//检查平面数据有效性
		double minZ=9999;
		double maxZ=0;
		double *minZp=&minZ;
		double *maxZp=&maxZ;
		minMaxIdx(fitGroundPoints[2],minZp,maxZp);
		
		if(minZ<800||maxZ>2000)
			continue;
		//cout << "Mat minZ = " << minZ << endl;
		//cout << "Mat maxZ = " << maxZ << endl;
		//拟合平面
		Mat adjustedFitGroundPoints(fitGroundPoints[0].rows*fitGroundPoints[0].cols,3,CV_32FC1);
		dataAdjustToMat(fitGroundPoints[0],fitGroundPoints[1],fitGroundPoints[2],adjustedFitGroundPoints);
		fitPlane(adjustedFitGroundPoints,g_groundEquation);
		
		
		pitchGround=acos(abs(g_groundEquation[1]))/pi*180 ;
		if(pitchGround<minPitch)
		{
			minPitch=pitchGround;
			minIndex=i;
		}
			
	}
	
	getPlaneFitPoint(fitAreaCoordinate[minIndex][0],fitAreaCoordinate[minIndex][1],box,roiwidth,roiheight);
	fitGroundPoints[0]=g_x_img(box);
	fitGroundPoints[1]=g_y_img(box);
	fitGroundPoints[2]=g_z_img(box);
		
		//拟合平面
	Mat adjustedFitGroundPoints(fitGroundPoints[0].rows*fitGroundPoints[0].cols,3,CV_32FC1);
	dataAdjustToMat(fitGroundPoints[0],fitGroundPoints[1],fitGroundPoints[2],adjustedFitGroundPoints);
	fitPlane(adjustedFitGroundPoints,g_groundEquation);
	
#ifdef  WRITEPARAMETERS
GroundPlaneParameterFile<<g_groundEquation[0]<<" "<<g_groundEquation[1]<<" "<<g_groundEquation[2]<<" "<<g_groundEquation[3]<<endl;
#endif
	
	//cout<<"pitch:  "<<minPitch<<endl;
		//显示
	Mat tmpSrcImage;
	g_src_img.copyTo(tmpSrcImage);
	rectangle(tmpSrcImage,Point(box.x,box.y),Point(box.x+box.width,box.y+box.height),Scalar(0,255,0),1,8,0);
	imshow("拟合平面区域",tmpSrcImage);
	
	
#ifdef SAVEVIDEO
		rgbWriter.write(tmpSrcImage);
		
#endif
	
	if(minPitch<50.0)
		return true;
	else	
		return false;
	
	
}

void imageProcess()
{
	double start = static_cast<double>(cvGetTickCount());
	//获得点云数据
	getMapData();
	
	Mat pillorMask=Mat::zeros(IMAGE_Y,IMAGE_X,CV_8UC1);
	Mat guardMask=Mat::zeros(IMAGE_Y,IMAGE_X,CV_8UC1);
	//拟合地面
	bool isFindPillar=false;
	bool isFindGuard=false;

	if(groundFit()==true)
	{
		
		
		//计算部分姿态角，未计算roll;
		
		calRotationMatrixbyGround();
	
		//计算到地面的距离
		calDistanceToGround(g_dist_image);
		//按照到地面的高度进行分割
		
		calSegDistMat(pillorMask,400,1200,3000,4000);
		isFindPillar=fit3DPillar(pillorMask);
		
	
		
		calSegDistMat(guardMask,100,500,1800,2500);
		isFindGuard=fit3DGuard(guardMask);
		if(isFindGuard==true)
		{
#ifdef WRITEPARAMETERS
		GuardPlaneParameterFile<<g_guardEquation[0]<<" "<<g_guardEquation[1]<<" "<<g_guardEquation[2]<<" "<<g_guardEquation[3]<<endl;	
#endif
			calRotationMatrixbyGuard();
			CalFullRotationMatrix();
		}
		
		if(isFindPillar==true&&isFindGuard==true)
		{
			
			CalCameraWorldCoordinate(0);
			
		}
		//更新最后的数据，为了调试用的
		//UpdateGuardPoints_Final(guard_points);
		
	}
	
	
	
	double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
	//cout << "图像处理所用时间为:" << time/1000 << "ms" << endl;	

#ifdef SAVEVIDEO
		
		pillarWriter.write(pillorMask);
	
#endif
#ifdef SAVEONLYRESULT
		
		pillarWriter.write(pillorMask);

#endif
		imshow("柱子",pillorMask);
		imshow("挡板",guardMask);
	
	
	
}