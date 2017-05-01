/*鼠标点击事件，显示该点深度距离*/
#include <stdio.h>
#include <Kinect.h>
#include <Windows.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>  
#include <cv.h>
using namespace cv;
using namespace std;

int depth_width = 512; //depth图像大小
int depth_height = 424;
Mat depthImg_show =Mat::zeros(depth_height, depth_width, CV_8UC3);//原始UINT16 深度图像不适合用来显示，所以需要砍成8位的就可以了，但是显示出来也不是非常好，最好能用原始16位图像颜色编码，凑合着看了  
Mat tempImage(depth_height, depth_width, CV_16UC1);
UINT16 * pixData = new UINT16[depth_width*depth_height];
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

void on_mouse(int event,int xpos,int ypos,int flags,void *ustc){
	static Point pre_pt = (-1, -1);
	static Point cur_pt = (-1, -1);
	Mat tmp = Mat(depth_height, depth_width, CV_8UC3);
	char temp[16];
	if (event == CV_EVENT_LBUTTONDOWN){
		//depthImg_show.copyTo(tmp);
		//sprintf(temp, "(%d,%d)", xpos, ypos);
		pre_pt = Point(xpos, ypos);
		//putText(depthImg_show, temp, pre_pt, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0, 255), 1, 8);//在窗口上显示坐标
		//circle(depthImg_show, pre_pt, 2, Scalar(255, 0, 0, 0), CV_FILLED, CV_AA, 0);//划圆  
		/*imshow("depth", depthImg_show);
		if (waitKey(0) == 27){
			exit(0);
		}*/
		cout << '('<< xpos << "," << ypos<<")" <<endl;
		
		if (tempImage.data!=NULL)
		{
		
			//计算出是第多少个像素点
			INT32 pixelIndex = (INT32)(pre_pt.x + ((INT32)pre_pt.y *depth_width));
	
			//double depth = (pixData[pixelIndex]>>3);
			double depth = (pixData[pixelIndex]);

			double distance = depth / 10;
			cout << "深度距离为: " << distance << "cm" << endl;
		}

	}
	
}

cv::Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		USHORT depth = *pBuffer;

		BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth % 256) : 0);

		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;

		++pBuffer;
	}
	return img;
}
// 转换color图像到cv::Mat  
cv::Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		*p_mat = pBuffer->rgbBlue;
		p_mat++;
		*p_mat = pBuffer->rgbGreen;
		p_mat++;
		*p_mat = pBuffer->rgbRed;
		p_mat++;

		++pBuffer;
	}
	return img;
}


void main()
{
	////////////////////////////////////////////////////////////////  

	int color_widht = 1920; //color图像大小
	int color_height = 1080;
	Size dsize = Size(depth_width, depth_height);
	cv::Mat colorImg = cv::Mat::zeros(color_height, color_widht, CV_8UC3);//the color image  

	cv::Mat depthImg = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);//the depth image  
	
	// Current Kinect  
	IKinectSensor* m_pKinectSensor = NULL;
	// Depth reader  
	IDepthFrameReader*  m_pDepthFrameReader = NULL;
	// Color reader  
	IColorFrameReader*  m_pColorFrameReader = NULL;
	RGBQUAD* m_pColorRGBX = new RGBQUAD[color_widht * color_height];
	//open it!  
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		cout << "Can not find the Kinect!" << endl;
		cv::waitKey(0);
		exit(0);
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader  
		IDepthFrameSource* pDepthFrameSource = NULL;
		IFrameDescription * pFrameDescription = NULL;
		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->get_FrameDescription(&pFrameDescription);
		}
		pFrameDescription->get_Width(&depth_width);
		pFrameDescription->get_Height(&depth_height);
		SafeRelease(pFrameDescription);
		SafeRelease(pDepthFrameSource);

		// for color  
		// Initialize the Kinect and get the color reader  
		IColorFrameSource* pColorFrameSource = NULL;
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}

		SafeRelease(pColorFrameSource);
	}

	//valify the depth reader  
	if (!m_pDepthFrameReader)
	{
		cout << "Can not find the m_pDepthFrameReader!" << endl;
		cv::waitKey(0);
		exit(0);
	}
	//valify the color reader  
	if (!m_pDepthFrameReader)
	{
		cout << "Can not find the m_pColorFrameReader!" << endl;
		cv::waitKey(0);
		exit(0);
	}
	// get the data!  
	UINT nBufferSize_depth = 0;
	UINT16 *pBuffer_depth = NULL;
	UINT nBufferSize_coloar = 0;
	RGBQUAD *pBuffer_color = NULL;

	char key = 0;
//	UINT16 *frameData = NULL;
	cvNamedWindow("depth");
	setMouseCallback("depth", on_mouse, 0);//调用回调函数  
	while (true)
	{
		IDepthFrame* pDepthFrame = NULL;
		HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hr))
		{
			USHORT nDepthMinReliableDistance = 0;
			USHORT nDepthMaxReliableDistance = 0;
			UINT Capacity = depth_width*depth_height;
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
			}
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);
				depthImg_show = ConvertMat(pBuffer_depth, depth_width, depth_height, nDepthMinReliableDistance, nDepthMaxReliableDistance);
			}
			if (SUCCEEDED(hr)){
				//hr = pDepthFrame->CopyFrameDataToArray(Capacity, reinterpret_cast<UINT16 *> (tempImage.data));
				hr = pDepthFrame->CopyFrameDataToArray(Capacity, pixData);
				
			}
			
		}
		SafeRelease(pDepthFrame);
		//cout <<tempImage.at<Vec3b>(156,156)[0]<<endl;

		//for color  
		IColorFrame* pColorFrame = NULL;
		hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
		ColorImageFormat imageFormat = ColorImageFormat_None;
		if (SUCCEEDED(hr))
		{
			ColorImageFormat imageFormat = ColorImageFormat_None;
			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}
			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}
			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)//这里有两个format，不知道具体含义，大概一个预先分配内存，一个需要自己开空间吧  
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize_coloar, reinterpret_cast<BYTE**>(&pBuffer_color));
				}
				else if (m_pColorRGBX)
				{
					pBuffer_color = m_pColorRGBX;
					nBufferSize_coloar = color_widht * color_height * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize_coloar, reinterpret_cast<BYTE*>(pBuffer_color), ColorImageFormat_Bgra);
				}
				else
				{
					hr = E_FAIL;
				}
				colorImg = ConvertMat(pBuffer_color, color_widht, color_height);
			}

			SafeRelease(pColorFrame);
		}
		Mat NewcolorImg = cv::Mat::zeros(color_height, color_widht, CV_8UC3);
		resize(colorImg, NewcolorImg, dsize);
		cv::imshow("depth", depthImg_show);
	//	cv::imshow("color", colorImg);
	//	imshow("Newcolor", NewcolorImg);
		key = cv::waitKey(1);
		if (key == 27)
		{
			break;
		}
	}


	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}
	// close the Kinect Sensor  
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pKinectSensor);

}