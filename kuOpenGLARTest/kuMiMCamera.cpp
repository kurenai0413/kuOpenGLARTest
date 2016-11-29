#include "kuMiMCamera.h"

kuMiMCamera::kuMiMCamera()
{
	m_IsCameraInitialized   = false;
	m_IsCameraThreadStarted = false;
	
	m_pFrameBuffer = NULL;
	m_CamFrame.data = NULL;
}


kuMiMCamera::~kuMiMCamera()
{
	if (m_IsCameraThreadStarted)
	{
		m_IsCameraThreadStarted = false;
		m_CamDisplayThread.join();	
	}

	if (m_IsCameraInitialized)
	{
		m_IsCameraInitialized = false;
		CameraAlignFree(m_pFrameBuffer);
		CameraUnInit(m_hCamera);
	}
}


void kuMiMCamera::DisplayCameraFrame(char * WndName)			// Display Thread Function
{
	while (m_IsCameraThreadStarted)
	{
		bool IsFrameCaptured = CaptureFrame();

		if (IsFrameCaptured)
		{
			imshow(WndName, m_CamFrame);
			waitKey(1);
		}
	}
}


bool kuMiMCamera::InitialCamera()
{
	int						i;
	int						TotalCamNum = 2;			// 設置num=2表示最多只抓2個攝影機
	CameraSdkStatus			status;
	tSdkCameraDevInfo		m_CameraList[2];

	m_FoundFreeCamera = false;
	
	if (!m_IsCameraInitialized)
	{
		// 列舉camera 最大只抓TotalCamNum台
		if (status = CameraEnumerateDevice(m_CameraList, &TotalCamNum) != CAMERA_STATUS_SUCCESS)
		{
			return false;
		}

		// initial一台非使用中的camera
		for (i = 0; i < TotalCamNum; i++)
		{
			status = CameraInit(&m_CameraList[i], -1, -1, &m_hCamera);
			if (status == CAMERA_STATUS_SUCCESS)
			{
				m_FoundFreeCamera = true;
				break;
			}
		}

		if (!m_FoundFreeCamera)
		{
			return false;
		}
		else
		{
			// 獲得該相機的特性描述
			if (status = CameraGetCapability(m_hCamera, &sCameraInfo) != CAMERA_STATUS_SUCCESS)
			{
				return false;
			}
			else
			{
				m_ImageWidth = sCameraInfo.sResolutionRange.iWidthMax;
				m_ImageHeight = sCameraInfo.sResolutionRange.iHeightMax;

				if (sCameraInfo.sIspCapacity.bMonoSensor)
				{
					m_ImageChannel = 1;
					CameraSetIspOutFormat(m_hCamera, CAMERA_MEDIA_TYPE_MONO8);
				}
				else
				{
					m_ImageChannel = 3;
				}
			}

			m_pFrameBuffer
				= (BYTE *)CameraAlignMalloc(sCameraInfo.sResolutionRange.iWidthMax * sCameraInfo.sResolutionRange.iHeightMax * m_ImageChannel, 16);

			if (m_CamFrame.data == NULL)
			{
				if (m_ImageChannel == 1)
					m_CamFrameTemp.create(m_ImageHeight, m_ImageWidth, CV_8UC1);
				else
					m_CamFrameTemp.create(m_ImageHeight, m_ImageWidth, CV_8UC3);
			}

			CameraPlay(m_hCamera);

			m_IsCameraInitialized = true;

			return true;
		}
	}
	else
	{
		return false;
	}
}


void kuMiMCamera::CloseCamera()
{
	if (m_IsCameraInitialized)
	{
		m_IsCameraInitialized = false;
		CameraAlignFree(m_pFrameBuffer);
		CameraUnInit(m_hCamera);
	}
}


bool kuMiMCamera::CaptureFrame()
{
	CameraSdkStatus			status;
	tSdkFrameHead 			sFrameInfo;
	BYTE				*	pbyBuffer;				// raw

	#pragma region // 取像部分 //
	////////////////////////////////////////////////////////////////////////////////////
	status = CameraGetImageBuffer(m_hCamera, &sFrameInfo, &pbyBuffer, 1000);

	if (status == CAMERA_STATUS_SUCCESS)
	{
		status = CameraImageProcess(m_hCamera, pbyBuffer, m_pFrameBuffer, &sFrameInfo); //連續模式

		if (status == CAMERA_STATUS_SUCCESS)
		{
			memcpy(m_CamFrameTemp.data,m_pFrameBuffer,
				   m_ImageHeight*m_ImageWidth*m_ImageChannel*sizeof(unsigned char));
			flip(m_CamFrameTemp,m_CamFrame,0);
		}
		else
		{
			return false;
		}

		CameraReleaseImageBuffer(m_hCamera, pbyBuffer);

		return true;
	}
	else
	{
		return false;
	}
	////////////////////////////////////////////////////////////////////////////////////
	#pragma endregion
}


void kuMiMCamera::StartCameraDisplay(char * WndName)
{
	m_DisplayWndName = WndName;

	m_IsCameraThreadStarted = true;
	m_CamDisplayThread = thread(&kuMiMCamera::DisplayCameraFrame, this, m_DisplayWndName);
}

void kuMiMCamera::StopCameraDisplay()
{
	m_IsCameraThreadStarted = false;
	m_CamDisplayThread.join();
}



