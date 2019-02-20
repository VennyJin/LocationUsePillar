#include "CoordinateTrans.h"

void MyRodrigues(vector<double> &r_vector,Mat &r_matrix)
{
    double theta=0;
    for(size_t i=0;i<r_vector.size();i++)
    {
        theta+=(r_vector[i])*r_vector[i];
    }
    theta=sqrt(theta);

    double cos_theta=cos(theta);
    double sin_theta=sin(theta);
    cout<<"theta "<<theta<<endl;
    cout<<"cos "<<cos_theta<<endl;

    Mat cos_Matrix=Mat::eye(3,3,CV_64FC1).mul(Mat::eye(3,3,CV_64FC1),cos_theta);
//    Mat sin_Matrix=Mat::eye(3,3,CV_64FC1).mul(Mat::eye(3,3,CV_64FC1),sin_theta);
    Mat r_vector_m=Mat::zeros(3,1,CV_64FC1);
    Mat n_mul_n_T;
    Mat n_pie=Mat::zeros(3,3,CV_64FC1);

    n_pie.at<double>(0,1)=-r_vector[2];
    n_pie.at<double>(1,0)=r_vector[2];

    n_pie.at<double>(0,2)=r_vector[1];
    n_pie.at<double>(2,0)=-r_vector[1];

    n_pie.at<double>(1,2)=-r_vector[0];
    n_pie.at<double>(2,1)=r_vector[0];


    for(int i=0;i<3;i++)
    {
        r_vector_m.at<double>(i,0)=r_vector[i];
    }

    gemm(r_vector_m,r_vector_m,1,10,0,n_mul_n_T,GEMM_2_T);


    cout<<"sin_matrix"<<sin_Matrix<<endl;
    cout<<"n_nt"<<n_mul_n_T<<endl;
    cout<<"n_pie"<<n_pie<<endl;
    r_matrix=cos_Matrix+(Mat::eye(3,3,CV_64FC1)-cos_Matrix)*n_mul_n_T+sin_Matrix*n_pie;

    cout<<r_matrix;
}
void cameraCoorToWorld(float &camera_x,float &camera_y,float &camera_z,Mat & worrldCoor,int pillarIdx)
{
    double groundWorldNormalVector[3]={0.0,1,0.0}; //需要事先标定相机安装的角度
    float groundCameraNormalVector[3];


    //计算旋转轴
    //groundWorldNormalVector叉乘拟合地面的方程
    double rotationAxis[3];

    //地面的向量怎么定方向
    //就先假设  y方向是正的吧  好像无所谓 之后再做处理 算了还是调整一下吧

    //cout<<"地面方程系数: "<< g_groundEquation[0] <<" "<<g_groundEquation[1]<<" "<<g_groundEquation[2]<<endl;
    if(g_groundEquation[1]<0.0)
    {
        for(int i=0;i<3;i++)
        {
            groundCameraNormalVector[i]=-g_groundEquation[i];
        }
    }
    else
    {
        for(int i=0;i<3;i++)
        {
            groundCameraNormalVector[i]=g_groundEquation[i];
        }
    }

    //计算旋转角
    double crossProduct=groundWorldNormalVector[0]*groundCameraNormalVector[0]+groundWorldNormalVector[1]*groundCameraNormalVector[1]+groundWorldNormalVector[2]*groundCameraNormalVector[2];
    cout<<"点乘："<<crossProduct<<endl;
    double theta=acos(crossProduct);
    cout<<"角度: "<<theta*180/pi<<endl;
    cout<<"弧度: "<<theta<<endl;
    //旋转前向量是 相机看到的向量 groundCameraNormalVector
    //旋转后向量是 规定的向量 groundWorldNormalVector
    rotationAxis[0]=groundCameraNormalVector[1]*groundWorldNormalVector[2]-groundCameraNormalVector[2]*groundWorldNormalVector[1];
    rotationAxis[1]=groundCameraNormalVector[2]*groundWorldNormalVector[0]-groundCameraNormalVector[0]*groundWorldNormalVector[2];
    rotationAxis[2]=groundCameraNormalVector[0]*groundWorldNormalVector[1]-groundCameraNormalVector[1]*groundWorldNormalVector[0];

    //归一化之前
    //cout<<"归一化之前: "<< rotationAxis[0] <<" "<<rotationAxis[1]<<" "<<rotationAxis[2]<<endl;

    //归一化系数的倒数
    double normalCof=sqrt(rotationAxis[0]*rotationAxis[0]+rotationAxis[1]*rotationAxis[1]+rotationAxis[2]*rotationAxis[2]);
    //cout<<"归一化系数: "<<normalCof<<endl;
    vector<double>rotationVector;
    for(int i=0;i<3;i++)
    {
        rotationVector.push_back(theta*rotationAxis[i]/normalCof);
    }
    cout<<"旋转向量: "<< rotationVector[0] <<" "<<rotationVector[1]<<" "<<rotationVector[2]<<endl;
    Mat rotationMatrix=Mat::zeros(3,3,CV_64FC1);
    Rodrigues(rotationVector,rotationMatrix);
    //MyRodrigues(rotationVector,rotationMatrix);

    cout<<"旋转矩阵:"<<endl;
    cout<<rotationMatrix;


    cout<<"绕x轴角度:"<<atan2(rotationMatrix.at<double>(2,1),rotationMatrix.at<double>(2,2))*180/pi<<endl;
    cout<<"绕z轴角度:"<<atan2(rotationMatrix.at<double>(1,0),(rotationMatrix.at<double>(0,0)))*180/pi<<endl;


    //计算旋转后的坐标
    //真的是难受 必修用64位才可以计算
    //worldCoor=rotationMatrix*cameraCoor;
    cout<<"原始坐标 "<<"  X: "<<camera_x<<" Y: "<<camera_y<<" Z : "<<camera_z<<endl;
}


void calRotationMatrixbyGuard()
{
    float guardWorldNormalVector[3]={0.0,0.0,1.0}; //需要事先标定相机安装的角度
    float guardCameraNormalVector[3];
    //计算旋转轴
    float rotationAxis[3];
    //地面的向量怎么定方向
    //就先假设  y方向是正的吧  好像无所谓 之后再做处理 算了还是调整一下吧
    //cout<<"地面方程系数: "<< g_groundEquation[0] <<" "<<g_groundEquation[1]<<" "<<g_groundEquation[2]<<endl;
    //调整法向量方向
    if(g_guardEquation[2]<0.0)
    {
        for(int i=0;i<3;i++)
        {
            guardCameraNormalVector[i]=-g_guardEquation[i];
        }
    }
    else
    {
        for(int i=0;i<3;i++)
        {
            guardCameraNormalVector[i]=g_guardEquation[i];
        }
    }



    float crossProduct=guardWorldNormalVector[0]*guardCameraNormalVector[0]+guardWorldNormalVector[1]*guardCameraNormalVector[1]+guardWorldNormalVector[2]*guardCameraNormalVector[2];
    float theta=acos(crossProduct);
    cout<<"角度: "<<theta*180/pi<<endl;


    //旋转前向量是 相机看到的挡板的法向量 guardCameraNormalVector
    //旋转后向量是 规定的法向量 guardWorldNormalVector
    //计算旋转向量
    rotationAxis[0]=guardCameraNormalVector[1]*guardWorldNormalVector[2]-guardCameraNormalVector[2]*guardWorldNormalVector[1];
    rotationAxis[1]=guardCameraNormalVector[2]*guardWorldNormalVector[0]-guardCameraNormalVector[0]*guardWorldNormalVector[2];
    rotationAxis[2]=guardCameraNormalVector[0]*guardWorldNormalVector[1]-guardCameraNormalVector[1]*guardWorldNormalVector[0];

    //归一化系数的倒数
    float normalCof=sqrt(rotationAxis[0]*rotationAxis[0]+rotationAxis[1]*rotationAxis[1]+rotationAxis[2]*rotationAxis[2]);
    vector<float>rotationVector;
    //得到旋转向量
    for(int i=0;i<3;i++)
    {
        rotationVector.push_back(theta*rotationAxis[i]/normalCof);
    }

    Rodrigues(rotationVector,rotationMatrixCameraToGuard_pie);

}
void calRotationMatrixbyGround()
{

    float groundWorldNormalVector[3]={0.0,1,0.0}; //需要事先标定相机安装的角度
    float groundCameraNormalVector[3];
    //计算旋转轴
    //groundWorldNormalVector叉乘拟合地面的方程
    float rotationAxis[3];

    //地面的向量怎么定方向
    //就先假设  y方向是正的吧  好像无所谓 之后再做处理 算了还是调整一下吧
    //cout<<"地面方程系数: "<< g_groundEquation[0] <<" "<<g_groundEquation[1]<<" "<<g_groundEquation[2]<<endl;
    if(g_groundEquation[1]<0.0)
    {
        for(int i=0;i<3;i++)
        {
            groundCameraNormalVector[i]=-g_groundEquation[i];
        }
    }
    else
    {
        for(int i=0;i<3;i++)
        {
            groundCameraNormalVector[i]=g_groundEquation[i];
        }
    }

    //计算旋转角
    float crossProduct=groundWorldNormalVector[0]*groundCameraNormalVector[0]+groundWorldNormalVector[1]*groundCameraNormalVector[1]+groundWorldNormalVector[2]*groundCameraNormalVector[2];
    float theta=acos(crossProduct);

    //cout<<"角度: "<<theta*180/pi<<endl;
    //旋转前向量是 相机看到的向量 groundCameraNormalVector
    //旋转后向量是 规定的向量 groundWorldNormalVector

    //计算旋转向量
    rotationAxis[0]=groundCameraNormalVector[1]*groundWorldNormalVector[2]-groundCameraNormalVector[2]*groundWorldNormalVector[1];
    rotationAxis[1]=groundCameraNormalVector[2]*groundWorldNormalVector[0]-groundCameraNormalVector[0]*groundWorldNormalVector[2];
    rotationAxis[2]=groundCameraNormalVector[0]*groundWorldNormalVector[1]-groundCameraNormalVector[1]*groundWorldNormalVector[0];

    //归一化系数的倒数
    float normalCof=sqrt(rotationAxis[0]*rotationAxis[0]+rotationAxis[1]*rotationAxis[1]+rotationAxis[2]*rotationAxis[2]);
    //cout<<"归一化系数: "<<normalCof<<endl;
    vector<float>rotationVector;
    //得到旋转向量
    for(int i=0;i<3;i++)
    {
        rotationVector.push_back(theta*rotationAxis[i]/normalCof);
    }


    Rodrigues(rotationVector,rotateMatrixCameraToGround);


    //	float g_pitch=atan2(rotateMatrixCameraToGround.at<float>(2,1),rotateMatrixCameraToGround.at<float>(2,2));  //绕x轴弧度制
    //	float g_roll=atan2(rotateMatrixCameraToGround.at<float>(1,0),(rotateMatrixCameraToGround.at<float>(0,0))); //绕z轴弧度制


    //cout<<"绕x轴角度:"<<g_pitch*180/pi<<endl;
    //cout<<"绕z轴角度:"<<g_roll*180/pi<<endl;

}

/*
 * 更新坐标，根据旋转矩阵，将
 */
void UpdateCameraCoordinate(Mat &xImage,Mat &yImage,Mat &zImage,Mat & r_Image)
{
    //rotateMatrixCameraToGround
    int rows=xImage.rows;
    int cols=xImage.cols;


    ofstream iniXFile("iniX.txt");
    ofstream iniYFile("iniY.txt");
    ofstream iniZFile("iniZ.txt");
    //Mat rotationPoints=Mat::zeros(3,rows*cols,CV_32FC1);


    Mat out_put=Mat ::zeros(3,rows*cols,CV_32FC1);
    ofstream RotateFile("rotate.txt");


    float *xr_data=out_put.ptr<float>(0);
    float *yr_data=out_put.ptr<float>(1);
    float *zr_data=out_put.ptr<float>(2);


    int num=0;

    for(int y=0;y<rows;y++)
    {
        float *xData=xImage.ptr<float>(y);
        float *yData=yImage.ptr<float>(y);
        float *zData=zImage.ptr<float>(y);


        //Point3f *RotatePoints=r_Image.ptr<Point3f>(y);

        for(int x=0;x<cols;x++)
        {

            xr_data[y*cols+x]=xData[x];
            yr_data[y*cols+x]=yData[x];
            zr_data[y*cols+x]=zData[x];

            iniXFile<<xData[x]<<"\n";
            iniYFile<<yData[x]<<"\n";
            iniZFile<<zData[x]<<"\n";
            num++;
        }

    }
    //cout<<rotateMatrixCameraToGround<<endl;

    Mat tmpRotation_f;
    rotateMatrixCameraToGround.convertTo(tmpRotation_f,CV_32FC1);
    cout<<tmpRotation_f<<endl;
    //cout<<tmpRotation_f.type()<<endl;

    gemm(tmpRotation_f,out_put,1,10,0,out_put,0);
    //cout<<r_Image.type()<<endl;
    //cout<<rotateMatrixCameraToGround.type()<<endl;
    Mat t_r_Image;
    transpose(out_put,t_r_Image);
    for(int y=0;y<t_r_Image.rows;y++)
    {
        float *tmpr_data=t_r_Image.ptr<float>(y);
        for (int x=0;x<t_r_Image.cols;x++)
        {
            RotateFile<<tmpr_data[x]<<" ";

        }
        RotateFile<<endl;
    }
    RotateFile.close();
    iniXFile.close();
    iniYFile.close();
    iniZFile.close();

}


void UpdateGuardPoints(Mat &guard_points)
{
    //Points*R_T
    gemm(guard_points,rotateMatrixCameraToGround,1,10,0,guard_points,GEMM_2_T);

}


void UpdateGuardPoints_Final(cv::Mat &guardPoints)
{
    gemm(guardPoints,rotationMatrixCameraToGuard_pie,1,10,0,guardPoints,GEMM_2_T);

    ofstream guard_Final_file("final.txt");
    for(int y=0;y<guardPoints.rows;y++)
    {
        float *data=guardPoints.ptr<float>(y);
        for(int x=0;x<guardPoints.cols;x++)
        {
            guard_Final_file<<data[x]<<" ";
        }
        guard_Final_file<<endl;
    }
    guard_Final_file.close();

}
