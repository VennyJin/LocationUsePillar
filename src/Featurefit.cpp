
#include "Featurefit.h"





/*
 * 功能：
 * 		计算各个点到地面的距离
 * 		AX+BY+CZ-D=0
 * 		d=|Ax0+By0+CZ0-D|/sqrt(A*A+B*B+C*C);
 * 		 =|Ax0+By0+CZ0-D|/n
 * 			
 * */
void calDistanceToGround(Mat & distImage)
{
	//double start = static_cast<double>(cvGetTickCount()); //开始计时
	float A=g_groundEquation[0];
	float B=g_groundEquation[1];
	float C=g_groundEquation[2];
	float D=g_groundEquation[3];
	
	int rows=distImage.rows;
	int cols=distImage.cols;
	for(int y=0;y<rows;y++)
	{
		float *pdsit = distImage.ptr<float>(y);
		float *xData = g_x_img.ptr<float>(y);
		float *yData = g_y_img.ptr<float>(y);
		float *zData = g_z_img.ptr<float>(y);
		for(int x=0;x<cols;x++)
		{
			pdsit[x]=abs((A*xData[x]+B*yData[x]+C*zData[x]-D));
			//(A*xData[x]+B*yData[x]+C*zData[x]-D);
		}
	}
	//double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
	//cout << "计算距离时间为:" << time/1000 << "ms" << endl;

}




/*
 * 
 * 
 *  最小二乘拟合平面，平面方程：Ax+By+Cz=D
 *  A = plane.at<float>(0,0)
 *  B = plane.at<float>(1,0)
 *  C = plane.at<float>(2,0)
 *  D = plane.at<float>(3,0)
 * 
 * */
void fitPlane(const cv::Mat &points, float planeCof[4]){
    
	//double start = static_cast<double>(cvGetTickCount());
	int rows = points.rows;
    int cols = points.cols;
	
    cv::Mat centroid = cv::Mat::zeros(1,cols,CV_32FC1);
    for(int i=0;i<cols;i++)
		{
        for(int j=0;j<rows;j++){
            centroid.at<float>(0,i) += points.at<float>(j,i);
        }
		
        centroid.at<float>(0,i)/=rows;
		//cout<<"第"<<i<<"列"<<centroid.at<float>(0,i)<<"   ";
	}
	//cout<<endl;

    cv::Mat points2 = cv::Mat::ones(rows,cols,CV_32FC1);
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){
            points2.at<float>(i,j) = points.at<float>(i,j) - centroid.at<float>(0,j) ;
        }
    }
    cv::Mat A,W,U,V;
    //cv::gemm(points2,points,1,NULL,0,A,CV_GEMM_A_T);
	//GEMM_1_T第一个矩阵转置
    cv::gemm(points2,points,1,10,0,A,GEMM_1_T);
    SVD::compute(A,W,U,V);

	
    Mat plane = cv::Mat::zeros(4,1,CV_32FC1);
	 
    for (int c = 0; c<cols; c++){
        plane.at<float>(c,0) = V.at<float>(cols-1,c);
        plane.at<float>(cols,0) += plane.at<float>(c,0)*centroid.at<float>(0,c);
		
    }
	
	for(int i=0;i<4;i++)
	{
		planeCof[i]=plane.at<float>(i,0);
	}
	
	
	
	//double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
	//cout << "拟合所用时间为:" << time/1000 << "ms" << endl;

}




/*
 * 
 * 调整数据格式
 * 
 * */
void dataAdjustToMat( Mat & Ximg, Mat &Yimg, Mat &Zimg,Mat &result )
{
	//double start = static_cast<double>(cvGetTickCount());
	int rows=Ximg.rows;
	int cols=Ximg.cols;
	for(int y=0;y<rows;y++)
	{
		float *xData=Ximg.ptr<float>(y);
		float *yData=Yimg.ptr<float>(y);
		float *zData=Zimg.ptr<float>(y);
		
		for(int x=0;x<cols;x++)
		{
			result.at<float>(y*cols+x,0)=xData[x];
			result.at<float>(y*cols+x,1)=yData[x];
			result.at<float>(y*cols+x,2)=zData[x];
		}
	}
	//double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
	//cout << "调整格式所用时间为:" << time/1000 << "ms" << endl;
}

void dataAdjustToGuard( Mat & Ximg, Mat &Yimg, Mat &Zimg,Mat & Dimg,Mat &result)
{
	//double start = static_cast<double>(cvGetTickCount());
	int rows=Ximg.rows;
	int cols=Ximg.cols;
	int pointNum=0;
	Mat iniPoints(rows*cols,3,CV_32FC1);
	for(int y=0;y<rows;y++)
	{
		float *xData=Ximg.ptr<float>(y);
		float *yData=Yimg.ptr<float>(y);
		float *zData=Zimg.ptr<float>(y);
        float *dData=Dimg.ptr<float>(y);
		
		for(int x=0;x<cols;x++)
		{
			if(dData[x]>50&&dData[x]<90)
			{	
				iniPoints.at<float>(pointNum,0)=xData[x];
				iniPoints.at<float>(pointNum,1)=yData[x];
				iniPoints.at<float>(pointNum,2)=zData[x];
				pointNum++;
			}
		}
	}
	Rect roi(0,0,3,pointNum);
	result=iniPoints(roi);
}

void dataAdjustToPillar(cv::Mat & Ximg, cv::Mat &Yimg,cv:: Mat &Zimg,std::vector<Point3f>& points)
{
	int rows=Ximg.rows;
	int cols=Ximg.cols;
	//ofstream PillarPoints("1.txt");
	for(int y=0;y<rows;y++)
	{
		float *xData=Ximg.ptr<float>(y);
		float *yData=Yimg.ptr<float>(y);
		float *zData=Zimg.ptr<float>(y);
		
		for(int x=0;x<cols;x++)
		{
			//去掉异常点
			if(zData[x]<2500||zData[x]>4300)
				continue;
			points.push_back(Point3f(xData[x],yData[x],zData[x]));
			//PillarPoints<<xData[x]<<"  "<<yData[x]<<"  "<<zData[x]<<endl; 
		}
	}
	//PillarPoints.close();
}

/*
 *依据距离进行分割 
 * */

void calSegDistMat(Mat &mask,float minHeight,float maxHeight,float minDepth,float maxDepth)
{
	
	
	
	for(int y=0;y<IMAGE_Y;y++)
	{
		uchar *pMask=mask.ptr<uchar>(y);
		float *pDist=g_dist_image.ptr<float>(y);
		float *pZdata=g_z_img.ptr<float>(y);
		for(int x=0;x<IMAGE_X;x++)
		{
			
			if(pDist[x]>minHeight&&pDist[x]<maxHeight&&pZdata[x]<maxDepth&&pZdata[x]>minDepth)
			{
				pMask[x]=255;
			}
		}
		
	}
	
}


//得到地图数据
void getMapData()
{
	
	
	//double start = static_cast<double>(cvGetTickCount());
	
	//将深度信息存储到图片里
	g_dep_img.convertTo(g_z_img,CV_32F);
	
	for(int y=0;y<IMAGE_Y;y++)
	{
		
		float *xData=g_x_img.ptr<float>(y);
		float *yData=g_y_img.ptr<float>(y);
		float *zData=g_z_img.ptr<float>(y);
		
		for(int x=0;x<IMAGE_X;x++)
		{
			xData[x]=-zData[x]*TanX/(IMAGE_X/2)*((IMAGE_X/2)-x);   //已经转成了右手系
			yData[x]=zData[x]*TanY/(IMAGE_Y/2)*((IMAGE_Y/2)-y);			
		}
		
	}

    //double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
    //cout << "所用时间为:" << time/1000 << "ms" << endl;

}

//用于对轮廓排序
static inline bool ContoursSortFunByNum(vector<cv::Point> contour1,vector<cv::Point> contour2)  
{  
    return (contour1.size() > contour2.size());  
}  


//用于对轮廓排序
static inline bool ContoursSortFunBySize(vector<cv::Point> contour1,vector<cv::Point> contour2)  
{  
    return (contourArea(contour1) > contourArea(contour2));  
}  


/*
 * AX+BY+CZ-D=0
 * (x-x0)/vx=(y-y0)/vy=(z-z0)/vz
 * 
 *令 m=Avx+Bvy+Cvz
 *
 * x=(B*vy*x0-B*vx*y0+C*vz*x0-C*vx*z0+D*vx)/m
 * y=(A*vx*y0-A*vy*x0+C*vz*y0-C*vy*z0+D*vy)/m
 * z=(A*vx*z0-A*vz*x0+B*vy*z0-B*vz*y0+D*vz)/m
 */
static void calCameraLocation(const Vec6f &lineParameters,Vec3f &cameraLocation)
{
	float A,B,C,D,x0,y0,z0,vx,vy,vz;
	A=g_groundEquation[0];
	B=g_groundEquation[1];
	C=g_groundEquation[2];
	D=g_groundEquation[3];
	vx=lineParameters[0];
	vy=lineParameters[1];
	vz=lineParameters[2];
	x0=lineParameters[3];
	y0=lineParameters[4];
	z0=lineParameters[5];
	
	float m=A*vx+B*vy+C*vz;
	
	cameraLocation[0]=(B*vy*x0-B*vx*y0+C*vz*x0-C*vx*z0+D*vx)/m;
	cameraLocation[1]=(A*vx*y0-A*vy*x0+C*vz*y0-C*vy*z0+D*vy)/m;
	cameraLocation[2]=(A*vx*z0-A*vz*x0+B*vy*z0-B*vz*y0+D*vz)/m;
	//cout<<"X  "<<cameraLocation[0]<<" Y "<<cameraLocation[1]<<" Z "<<cameraLocation[2]<<endl;
	//cout<<m<<endl;
	
	
	//cout<<"x0:"<<x0<<" y0: "<<y0<<" z0 "<<z0<<endl;
}
//拟合空间直线 直接用opencv的函数
static void fit3DPillar(vector<Point3f>& points,Vec6f &lineParameters)
{
	
	//Output line parameters. In case of 2D fitting, it should be
	//a vector of 4 elements (like Vec4f) - (vx, vy, x0, y0), 
	//where (vx, vy) is a normalized vector collinear to the line and (x0, y0) is a point on the line.
	//In case of 3D fitting, it should be a vector of 6 elements (like Vec6f) - (vx, vy, vz, x0, y0, z0),
	//where (vx, vy, vz) is a normalized vector collinear to the line and (x0, y0, z0) is a point on the line. 
	
	//vx vy vz已经归一化了
	fitLine(points,lineParameters,DIST_L2,0.0,0.01,0.01);
	//cout<<"vx*vx+vy*vy+vz*vz: "<<lineParameters[0]*lineParameters[0]+lineParameters[1]*lineParameters[1]+lineParameters[2]*lineParameters[2]<<endl;
	//cout<<"平行向量vx： "<<lineParameters[0]<<" vy: "<<lineParameters[1]<<" vz: "<<lineParameters[2]<<" x0： "<<lineParameters[3]<<" y0 "<<lineParameters[4]<<" z0:"<<lineParameters[5]<<endl;
	
}
void fit3DPillar(Mat &pillarMask)
{
	//提取二维图像的联通域
	vector<vector<Point> > contours;
	vector<Vec4i>hierarchy;
	
	findContours(pillarMask,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
	int contours_num=contours.size();
	 if(contours_num<=0)
		return;
		
	int idx=0; //轮廓序号	
	sort(contours.begin(),contours.end(),ContoursSortFunByNum);//按照轮廓点数进行排序
	
	//最多有3个，记得修改成一个全局变量  方便调试
	for(;idx<min(1,contours_num);idx++)
	//for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
		
		//计算拟合宽与长的比例
		Point2f vtx[4];
        RotatedRect box = minAreaRect(contours[idx]);
		box=minAreaRect(contours[idx]);
		box.points(vtx);
		//矩形宽长比例
		double ratio_of_w_h=min(box.size.width/box.size.height,box.size.height/box.size.width);
		if(ratio_of_w_h>0.4)
			continue;
		
		
		//提取轮廓中的点
		//vector<Point3f>pillar_points  ;//轮廓中的点
		
		//计算最小可以覆盖轮廓的矩形
		vector<Point>contours_Points=contours[idx];
		vector<Point3f>fitPillar_Points;
		int minContoursX=9999;
		int maxContoursX=-1;
		int minContoursY=9999;
		int maxContoursY=-1;
        for(size_t i=0;i<contours_Points.size();i++)
		{
			if(contours_Points[i].x<minContoursX)
			{
				minContoursX=contours_Points[i].x;
			}
			if(contours_Points[i].y<minContoursY)
			{
				minContoursY=contours_Points[i].y;
			}
			if(contours_Points[i].x>maxContoursX)
			{
				maxContoursX=contours_Points[i].x;
			}
			if(contours_Points[i].y>maxContoursY)
			{
				maxContoursY=contours_Points[i].y;
			}
		}
		
		//cout<<"minRoiX: "<<minContoursX<<" maxRoiX: "<<maxContoursX<<"  minRoiY: "<<minContoursY<<" maxContoursY: "<<maxContoursY<<endl;
		
		//提取ROI
		Mat pillarRoiImage[3];
		Rect pillarBox=Rect(minContoursX,minContoursY,maxContoursX-minContoursX,maxContoursY-minContoursY);
		
		pillarRoiImage[0]=g_x_img(pillarBox);
		pillarRoiImage[1]=g_y_img(pillarBox);
		pillarRoiImage[2]=g_z_img(pillarBox);
		//转化数据格式
		dataAdjustToPillar(pillarRoiImage[0],pillarRoiImage[1],pillarRoiImage[2],fitPillar_Points);
		
		if(fitPillar_Points.size()<100)   //点数太少  直接返回
			return;
		Vec6f pillar;
		//cout<<"拟合柱子点数: "<<fitPillar_Points.size()<<endl;
		fit3DPillar(fitPillar_Points,pillar);
		
		
		//计算柱子与拟合的地面成的角度
		// cos (theta)=(A*vx+B*vy+C*vz)/(1*1)  因为两个法向量已经归一化
	
		double cosThetaPillarToGround=pillar[0]*g_groundEquation[0]+pillar[1]*g_groundEquation[1]+pillar[2]*g_groundEquation[2];
		double thetaPillarToGround=asin(abs(cosThetaPillarToGround))/pi*180;
		//cout<<"柱子与地面成的角度: "<<thetaPillarToGround<<endl;
		
		//计算柱子与地面的交点的相机坐标
		Vec3f cameraLocation;
		calCameraLocation(pillar,cameraLocation);
		
		
		Mat pillarWorldLocation;
		//cameraCoorToWorld(cameraLocation[0],cameraLocation[1],cameraLocation[2],pillarWorldLocation,1);
		
	}
	
	

	
	//画最大的
	/*
	按照边界长度进行排序
	sort(contours.begin(),contours.end(),ContoursSortFunByNum);
	Point2f vtx[4];
	RotatedRect box = minAreaRect(contours[0]);
		
	box=minAreaRect(contours[0]);
	box.points(vtx);
		
		// Draw the bounding box
	for( int i = 0; i < 4; i++ )
		line(tmp, vtx[i], vtx[(i+1)%4], Scalar(255, 255, 0), 1, LINE_AA);
	

	drawContours( tmp, contours, 0, Scalar(0,255,0), FILLED, 8, hierarchy );
	*/	
	
	//采用特征长度不大行呀
	//cout<<"序号:"<<idx<<"近似宽除以高：  "<<4*contourArea(contours[idx])/(contours[idx].size()*contours[idx].size())<<endl;
	//drawContours( tmp, contours, 0, Scalar(0,255,255), 1, 1, hierarchy );
	//imshow("珠子彩色",tmp);
	
	/* */
	//vector<Point3f>points;
	
	
}




void fit3DGuard(Mat &guardMask)
{
	//首先得到要进行拟合的点
	//提取二维图像的联通域
	vector<vector<Point> > contours;
	vector<Vec4i>hierarchy;
	
	findContours(guardMask,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
	int contours_num=contours.size();
	if(contours_num<=0)
		return ;
	sort(contours.begin(),contours.end(),ContoursSortFunByNum);//按照轮廓点数进行排序
	int idx=0;
	
	//选取最大的那个轮廓进行处理
	for(;idx<min(1,contours_num);idx++)
	{
		//计算拟合宽与长的比例
		Point2f vtx[4];
        RotatedRect box = minAreaRect(contours[idx]);
		box=minAreaRect(contours[idx]);
		box.points(vtx);
		//矩形宽长比例
		double ratio_of_w_h=min(box.size.width/box.size.height,box.size.height/box.size.width);
		if(ratio_of_w_h>0.2)
			continue;
			
			
			//提取轮廓中的点
		//Mat guard_points  ;//轮廓中的点
		
		//计算最小可以覆盖轮廓的矩形
		vector<Point>contours_Points=contours[idx];
		vector<Point3f>fitGuard_Points;
		int minContoursX=9999;
		int maxContoursX=-1;
		int minContoursY=9999;
		int maxContoursY=-1;
        for(size_t i=0;i<contours_Points.size();i++)
		{
			if(contours_Points[i].x<minContoursX)
			{
				minContoursX=contours_Points[i].x;
			}
			if(contours_Points[i].y<minContoursY)
			{
				minContoursY=contours_Points[i].y;
			}
			if(contours_Points[i].x>maxContoursX)
			{
				maxContoursX=contours_Points[i].x;
			}
			if(contours_Points[i].y>maxContoursY)
			{
				maxContoursY=contours_Points[i].y;
			}
		}
		
		
		//提取ROI
		Mat guardRoiImage[4];
		
		//少计算一些点
		int cenContoursX=(minContoursX+maxContoursX)/2;
		int cenContoursY=(maxContoursY+minContoursY)/2;
		int guardFitBoxWidth=min(100,cenContoursX-minContoursX);
		int guardFitBoxHeight=min(20,cenContoursY-minContoursY);
		Rect guardBox=Rect(cenContoursX-guardFitBoxWidth/2,cenContoursY-guardFitBoxHeight/2,guardFitBoxWidth,guardFitBoxHeight);
		
		guardRoiImage[0]=g_x_img(guardBox);
		guardRoiImage[1]=g_y_img(guardBox);
		guardRoiImage[2]=g_z_img(guardBox);
		guardRoiImage[3]=g_dist_image(guardBox);
		//转化数据格式
		
		
		//Mat guard_points(guardRoiImage[0].rows*guardRoiImage[0].cols,3,CV_32FC1);
		Mat guard_points;
		//显示处理的区域
		Mat tmpSrcImage=g_src_img.clone();
		rectangle(tmpSrcImage,Point(guardBox.x,guardBox.y),Point(guardBox.x+guardBox.width,guardBox.y+guardBox.height),Scalar(0,255,0),1,8,0);

        dataAdjustToGuard(guardRoiImage[0],guardRoiImage[1],guardRoiImage[2],guardRoiImage[3],guard_points);
		
		
		imshow("拟合挡板",tmpSrcImage);
		if(guard_points.rows<30)
			return;
		//cout<<"拟合宽度： "<<guardFitBoxWidth<<"拟合高度 "<<guardFitBoxHeight<<endl;
		
		//更新坐标数据
		UpdateGuardPoints(guard_points);
		
		//cout<<"开始拟合挡板"<<endl;
		//拟合方程
		//float guardParameters[4];
		//fit3DGuard(guard_points,guardParameters);
		fitPlane(guard_points, g_guardEquation);
		cout<<"挡板方程： "<<g_guardEquation[0]<<" "<<g_guardEquation[1]<<"  "<<g_guardEquation[2]<<"  "<<g_guardEquation[3]<<endl;
		
		calRotationMatrixbyGuard();
	
		cout<<"旋转矩阵2"<<rotationMatrixCameraToGuard_pie<<endl;

        UpdateGuardPoints_Final(guard_points);
	}
}
