#pragma once

#include <windows.h>
#include <thread>
#include <iostream>

#include "CameraApi.h"
#include "opencv2/opencv.hpp"

#pragma comment(lib,"MVCAMSDK.lib")

#if defined _DEBUG
#pragma comment(lib,"opencv_world310d.lib")
#else
#pragma comment(lib,"opencv_core247.lib")
#pragma comment(lib,"opencv_imgproc247.lib")
#pragma comment(lib,"opencv_highgui247.lib")
#endif

using namespace cv;
using namespace std;

#pragma once
class kuMiMCamera
{
public:
	
	tSdkCameraCapbility		sCameraInfo;

	Mat						m_CamFrame;
	Mat						m_CamFrameTemp;

	thread					m_CamDisplayThread;

	char				*	m_DisplayWndName;

	int						m_hCamera;					// Camera handle
	int						m_ImageWidth;
	int						m_ImageHeight;
	int						m_ImageChannel;

	bool					m_FoundFreeCamera;
	bool					m_IsCameraInitialized;

	kuMiMCamera();
	~kuMiMCamera();

	bool InitialCamera();
	void CloseCamera();
	bool CaptureFrame();
	void StartCameraDisplay(char * WndName);
	void StopCameraDisplay();

private:
	
	BYTE				*	m_pFrameBuffer;			// RGB

	bool					m_IsCameraThreadStarted;

	void DisplayCameraFrame(char * WndName);		// Display Thread Function
};

