#include "main.h"


//Global Variable
Mat g_src_img = Mat::zeros(IMAGE_Y, IMAGE_X, CV_16UC3);
Mat g_dep_img = Mat::zeros(IMAGE_Y, IMAGE_X, CV_16UC1);//存储的是x值


//存储实际坐标
Mat g_x_img=Mat::zeros(IMAGE_Y,IMAGE_X,CV_32FC1);
Mat g_y_img=Mat::zeros(IMAGE_Y,IMAGE_X,CV_32FC1);
Mat g_z_img=Mat::zeros(IMAGE_Y,IMAGE_X,CV_32FC1);

//存储到地面的距离
Mat g_dist_image=Mat::zeros(IMAGE_Y,IMAGE_X,CV_32FC1);

//地面方程系数   AX+BY+CZ-D=0  其中A*A+B*B+C*C=1
float g_groundEquation[4];  //分别存储A  B   C   D
//挡板方程系数
float g_guardEquation[4];

//相机姿态角
double g_pitch=0.0;  //绕x轴
double g_yaw=0.0;  //绕y轴 
double g_roll=0.0;  //绕z轴


const int cdepthwidth = 320;
const int cdepthheight = 240;
const int ccolorwidth = 320;
const int ccolorheight = 240;

const int frame_num = 30;




#ifdef SAVEVIDEO
//保存视频
VideoWriter rgbWriter;
VideoWriter pillarWriter;
#endif

#ifdef SAVEONLYRESULT

VideoWriter pillarWriter;
#endif

void on_mouse(int event,int x,int y,int flags,void *ustc)
{
	Mat image = *(cv::Mat*) ustc;
	
	
	if(event==CV_EVENT_RBUTTONDOWN)
	{
			cout<<x<<' '<<y<<endl;
			cout<<"X "<<g_x_img.at<float>(y,x)<<endl;
			cout<<"Y "<<g_y_img.at<float>(y,x)<<endl;
			cout<<"Z "<<g_z_img.at<float>(y,x)<<endl;
			cout<<"dist "<<g_dist_image.at<float>(y,x)<<endl;
	}
}
int main(int argc, char **argv)
{
	
#ifdef SAVEVIDEO
	rgbWriter.open("./Color/"+nowTime2String()+".avi",CV_FOURCC('M','J','P','G'),30,Size(IMAGE_X,IMAGE_Y));
	pillarWriter.open("./Pillar/"+nowTime2String()+"_pillar.avi",CV_FOURCC('M','J','P','G'),30,Size(IMAGE_X,IMAGE_Y));
	if(!rgbWriter.isOpened()||!pillarWriter.isOpened())
	{
		cout<<"video righter opened failed"<<endl;
		return -1;
	}
#endif
#ifdef SAVEONLYRESULT
	pillarWriter.open("./Pillar/"+nowTime2String()+".avi",CV_FOURCC('M','J','P','G'),30,Size(IMAGE_X,IMAGE_Y));

	
#endif 
	// Initialize OpenNI Environment
    OpenNI::initialize();
   // 声明并打开Device设备，我用的是Kinect。
    Device devAnyDevice;
    devAnyDevice.open(ANY_DEVICE );
   // 创建深度数据流
    VideoStream streamDepth;
    streamDepth.create( devAnyDevice, SENSOR_DEPTH );
    // 创建彩色图像数据流
    VideoStream streamColor;
    streamColor.create( devAnyDevice, SENSOR_COLOR );
    // 设置深度图像视频模式
    VideoMode mModeDepth;
    // 分辨率大小
    mModeDepth.setResolution( cdepthwidth, cdepthheight );
    // 每秒30帧
    mModeDepth.setFps( frame_num );
    // 像素格式
    mModeDepth.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );

    streamDepth.setVideoMode( mModeDepth);

    // 同样的设置彩色图像视频模式
    VideoMode mModeColor;
    mModeColor.setResolution( ccolorwidth, ccolorheight );
    mModeColor.setFps( frame_num );
    mModeColor.setPixelFormat( PIXEL_FORMAT_RGB888 );
	
	
    streamColor.setVideoMode( mModeColor);
	auto camSetting =streamColor.getCameraSettings();
	camSetting->setAutoWhiteBalanceEnabled(true);
	camSetting->setAutoExposureEnabled(true);
	//camSetting->setExposure(70)!=openni::STATUS_OK;
    // 图像模式注册
    if( devAnyDevice.isImageRegistrationModeSupported(
        IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
    {
        devAnyDevice.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    }
	
	  // 打开深度和图像数据流
    streamDepth.start();
    streamColor.start();
	
	
	Mat cImageBGR;
	Mat mScaledDepth;

    // 获得最大深度值
    int iMaxDepth = streamDepth.getMaxPixelValue();

    // 循环读取数据流信息并保存在VideoFrameRef中
    VideoFrameRef  frameDepth;
    VideoFrameRef  frameColor;
	
	cout<<"initial okay"<<endl;
	
	 // 创建OpenCV图像窗口
	namedWindow( "Depth Image" );
	namedWindow( "Color Image" );
	setMouseCallback("Color Image", on_mouse,(void *)&g_dep_img);
	
	while(1)
	{
		double start = static_cast<double>(cvGetTickCount());
		streamDepth.readFrame( &frameDepth );
		streamColor.readFrame( &frameColor );
		
		
		// 将深度数据转换成OpenCV格式
		const Mat mImageDepth( frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());
		g_dep_img = mImageDepth.clone();
		
		// 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
		mImageDepth.convertTo( mScaledDepth, CV_8U, 255.0 / iMaxDepth );
            // 显示出深度图像
		imshow( "Depth Image", mScaledDepth );
            // 同样的将彩色图像数据转化成OpenCV格式
		const Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
            // 首先将RGB格式转换为BGR格式
		cvtColor( mImageRGB, cImageBGR, COLOR_RGB2BGR );
		g_src_img = cImageBGR.clone();
		
		
		//处理图片
        imageProcess();

        // 然后显示彩色图像
        imshow( "Color Image", cImageBGR );

		//cout<<"okokokok"<<endl;
			
		char c = waitKey(30);
		if (c == 'q') break;
		
		
		double time = ((double)cvGetTickCount() - start) / cvGetTickFrequency();
		//cout << "所用时间为:" << time/1000 << "ms" << endl;
	}
	
	streamDepth.stop();
    streamColor.stop();
		
	devAnyDevice.close();
	
	
}



String nowTime2String()
{
	struct timeval tv;
	struct tm* ptm;
	char time_string[40];
	String time;
	gettimeofday(&tv,NULL);
	ptm = localtime (&tv.tv_sec);
	strftime (time_string, sizeof (time_string),"%Y_%m_%d_%H_%M_%S", ptm);
	time=time_string;
	return time;
}
