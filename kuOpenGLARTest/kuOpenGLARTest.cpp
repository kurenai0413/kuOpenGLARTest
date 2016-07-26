#include <windows.h>
#include "gl/gl.h"
#include "gl/glu.h"
#include "gl/glut.h"
#include "opencv2/opencv.hpp"
#include <GLFW/glfw3.h>

#pragma comment(lib,"glfw3.lib")
#pragma comment(lib,"opencv_core2413d.lib")
#pragma comment(lib,"opencv_calib3d2413d.lib")
#pragma comment(lib,"opencv_imgproc2413d.lib")
#pragma comment(lib,"opencv_highgui2413d.lib")

using namespace cv;
using namespace std;

#define		Left		0
#define		Right		1
#define		ImgWidth	640
#define		ImgHeight	480
#define     farClip		50
#define		nearClip	5000
#define		cbSize		25

GLFWwindow		*	window;


Mat					CamFrame[2];
Mat					UndistortedFrame[2];
Mat					StereoFrame;

VideoCapture	*	cap[2];

Mat					IntrinsicMat[2];
Mat					DistParam[2];
vector<Point3f>		CB3DPts;
vector<Point2f>		CB2DPts[2];
vector<Point2f>		Projected2DPts[2];
Mat					RotationVec[2];
Mat					RotationMat[2];
Mat					TranslationVec[2];

double				m[2][16];
double				exglpara[2][16];
double				m_right[16];

void Init();
void DispFunc();
void DispIntrinsicParam(int side);
//void DispExtParam();
void SetCB3DPts(int CBSize);
bool LoadIntrinsicParam(char * Filename, int side);
//void SaveExtrinsicParameters(char * Filename);
void DrawAxes(float length);
void IntrinsicCVtoGL(Mat IntParam, double GLProjection[16]);
void ExtrinsicCVtoGL(Mat RotMat, Mat TransVec, double GLModelView[16]);
void DrawStereoFrame(Mat Frame[2]);
void RenderWireCubes(int CBSize);
bool DetectChessboardCV(int side);
bool CalExtrinsicParamCV(int side);
void SetGLProjectionMat(int side, double m[16]);
void SetGLModelviewMat(int side, double gl_para[16]);

static void error_callback(int error, const char* description);
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

int main()
{
	Init();

	while (!glfwWindowShouldClose(window))
	{
		DispFunc();

		glfwSwapBuffers(window);
		glfwPollEvents();	// This function processes only those events that are already 
							// in the event queue and then returns immediately
	}

	glfwDestroyWindow(window);
	glfwTerminate();

	exit(EXIT_SUCCESS);
}

void Init()
{
	int			i;

	#pragma region // OpenGL initialization by GLFW //
	/////////////////////////////////////////////////////////////////////////////////////
	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	window = glfwCreateWindow(2 * ImgWidth, ImgHeight, "Stereo", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);					// the number of screen updates to wait from the time
	glfwSetKeyCallback(window, key_callback);
	/////////////////////////////////////////////////////////////////////////////////////
	#pragma endregion

	// initialize OpenCV video capture
	cap[Left]  = new VideoCapture(1);
	cap[Right] = new VideoCapture(0);

	StereoFrame.create(ImgHeight, 2 * ImgWidth, CV_8UC3);

	for (i = 0; i < 2; i++)
	{
		IntrinsicMat[i].create(3, 3, CV_32FC1);
		DistParam[i].create(1, 4, CV_32FC1);
		RotationVec[i].create(3, 1, CV_64FC1);
		RotationMat[i].create(3, 3, CV_64FC1);
		TranslationVec[i].create(3, 1, CV_64FC1);
	}

	LoadIntrinsicParam("IntParam_Left.txt", Left);
	LoadIntrinsicParam("IntParam_Right.txt", Right);
	
	for (i = 0; i < 2;i++)
		DispIntrinsicParam(i);

	SetCB3DPts(cbSize);
}

void DispFunc()
{
	int		i;
	bool	CBFound[2];

	// clear the window
	glClear(GL_COLOR_BUFFER_BIT);

	for (i = 0; i < 2; i++)
	{
		cap[i]->read(CamFrame[i]);				// capture image
		undistort(CamFrame[i], UndistortedFrame[i], IntrinsicMat[i], DistParam[i]);
	}

	//imshow("Left Image",CamFrame[Left]);
	//imshow("Right Image",CamFrame[Right]);

	DrawStereoFrame(UndistortedFrame);
	// render background first, or the stereoframe will overlay the virtual object.
	
	/*
	for (i = 0; i < 2; i++)
	{
		glViewport(ImgWidth * i, 0, ImgWidth, ImgHeight);
		SetGLProjectionMat(i, m[i]);

		CBFound[i] = CalExtrinsicParamCV(i);

		if (CBFound[i])
		{
			SetGLModelviewMat(i, exglpara[i]);

			//now that the camera params have been set, draw your 3D shapes
			//first, save the current matrix
			glPushMatrix();

			RenderWireCubes(cbSize);
			DrawAxes(cbSize);

			glPopMatrix();
		}
	}
	*/
	glClear(GL_DEPTH_BUFFER_BIT);
}

void DrawAxes(float length)
{
	glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_LIGHTING);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(length, 0, 0); // X-axis => red

	glColor3f(0, 1, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, length, 0); // Y-axis => green

	glColor3f(0, 0, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, length); // Z-axis => blue
	glEnd();

	glPopAttrib();
}

void IntrinsicCVtoGL(Mat IntParam, double GLProjection[16])
{
	int			i, j;
	double		p[3][3];
	double		q[4][4];

	memset(GLProjection, 0, 16 * sizeof(double));

	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			p[i][j] = IntParam.at<float>(i, j);
		}
	}

	for (i = 0; i < 3; i++)
	{
		p[1][i] = (ImgHeight - 1) * p[2][i] - p[1][i];
	}

	q[0][0] = (2.0 * p[0][0] / (ImgWidth - 1));
	q[0][1] = (2.0 * p[0][1] / (ImgWidth - 1));
	q[0][2] = ((2.0 * p[0][2] / (ImgWidth - 1)) - 1.0);
	q[0][3] = 0.0;

	q[1][0] = 0.0;
	q[1][1] = (2.0 * p[1][1] / (ImgHeight - 1));
	q[1][2] = ((2.0 * p[1][2] / (ImgHeight - 1)) - 1.0);
	q[1][3] = 0.0;

	q[2][0] = 0.0;
	q[2][1] = 0.0;
	q[2][2] = (farClip + nearClip) / (farClip - nearClip);
	q[2][3] = -2.0 * farClip * nearClip / (farClip - nearClip);

	q[3][0] = 0.0;
	q[3][1] = 0.0;
	q[3][2] = 1.0;
	q[3][3] = 0.0;

	// transpose
	for (i = 0; i < 4; i++)
	{
		for (j = 0; j < 4; j++)
		{
			GLProjection[4 * i + j] = q[j][i];
		}
	}
}

void ExtrinsicCVtoGL(Mat RotMat, Mat TransVec, double GLModelView[16])
{
	memset(GLModelView, 0, 16 * sizeof(double));
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			GLModelView[4 * i + j] = RotMat.at<double>(j, i);
		}
		GLModelView[12 + i] = TransVec.at<double>(i, 0);
	}
	GLModelView[15] = 1;
}

void DrawStereoFrame(Mat Frame[2])
{
	int					i;
	Mat					GLFlipedFrame;			// Fliped camera frame for GL display 

	for (i = 0; i < 2; i++)
	{
		flip(Frame[i], GLFlipedFrame, 0);
		GLFlipedFrame.copyTo(StereoFrame(Rect(ImgWidth*i, 0, ImgWidth, ImgHeight)));
	}

	glDrawPixels(2 * ImgWidth, ImgHeight,
				 GL_BGR_EXT, GL_UNSIGNED_BYTE, StereoFrame.ptr());
}

void RenderWireCubes(int CBSize)
{
	glColor3f(0, 1, 0);
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			glPushMatrix();
			glTranslatef(-1.5*CBSize + 2 * CBSize * i, 2.5*CBSize - 2 * CBSize * j, 0.5*CBSize);
			glutWireCube(CBSize);
			glPopMatrix();
			glPushMatrix();
			glTranslatef(-0.5*CBSize + 2 * CBSize * i, 1.5*CBSize - 2 * CBSize * j, 0.5*CBSize);
			glutWireCube(CBSize);
			glPopMatrix();
		}
	}
}

bool DetectChessboardCV(int side)
{
	Mat		GrayImg;
	
	cvtColor(UndistortedFrame[side], GrayImg, CV_RGB2GRAY);

	bool CBFound = findChessboardCorners(GrayImg, Size(5, 7), CB2DPts[side],
										 CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
	//drawChessboardCorners(UndistortedFrame[side], Size(5, 7), Mat(CB2DPts[side]), CBFound);

	if (CBFound == true)
		return true;
	else
		return false;
}

bool CalExtrinsicParamCV(int side)
{
	bool CBFound = DetectChessboardCV(side);
	
	if (CBFound)
	{
		solvePnP(CB3DPts, CB2DPts[side],
				 IntrinsicMat[side], DistParam[side],
				 RotationVec[side], TranslationVec[side]);
		Rodrigues(RotationVec[side], RotationMat[side]);
		
		return CBFound;
	}
	else
		return false;
}

void SetGLProjectionMat(int side, double m[16])
{
	glMatrixMode(GL_PROJECTION);
	IntrinsicCVtoGL(IntrinsicMat[side], m);
	glLoadMatrixd(m);
}

void SetGLModelviewMat(int side, double gl_para[16])
{
	// you will have to set modelview matrix using extrinsic camera params
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	ExtrinsicCVtoGL(RotationMat[side], TranslationVec[side], gl_para);
	glLoadMatrixd(gl_para);
}

void SetCB3DPts(int CBSize)
{
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			CB3DPts.push_back(Point3f(-2 * CBSize + CBSize * j, 3* CBSize - CBSize * i, 0));
		}
	}
}

void DispIntrinsicParam(int side)
{
	char CamSideDescription[80];
	switch (side)
	{
		case Left:
			sprintf_s(CamSideDescription, "Left Intrinsic Parameters:");
			break;
		case Right:
			sprintf_s(CamSideDescription, "Right Intrinsic Parameters:");
			break;
	}
	cout << CamSideDescription << endl;
	for (int i = 0; i < 3; i++)
	{
		cout << IntrinsicMat[side].at<float>(i, 0) << " " << IntrinsicMat[side].at<float>(i, 1) << " "
			 << IntrinsicMat[side].at<float>(i, 2) << endl;
	}
	cout << DistParam[side].at<float>(0, 0) << " " << DistParam[side].at<float>(0, 1) << " "
		 << DistParam[side].at<float>(0, 2) << " " << DistParam[side].at<float>(0, 3) << endl << endl;
}

//void DispExtParam()
//{
//	for (int i = 0; i < 3; i++)
//	{
//		cout << RotationMat.at<double>(i, 0) << " "
//			 << RotationMat.at<double>(i, 1) << " "
//			 << RotationMat.at<double>(i, 2) << endl;
//	}
//
//	cout << endl;
//	cout << TranslationVec.at<double>(0, 0) << " "
//		 << TranslationVec.at<double>(1, 0) << " "
//		 << TranslationVec.at<double>(2, 0) << endl;
//	cout << endl;
//}

bool LoadIntrinsicParam(char * Filename, int side)
{
	FILE	*	fp;
	errno_t		err;

	err = fopen_s(&fp, Filename, "r");
	if (err != 0)
	{
		return false;
	}
	else
	{
		for (int i = 0; i < 3; i++)
		{
			fscanf_s(fp, "%f %f %f\n", &IntrinsicMat[side].at<float>(i, 0),
									   &IntrinsicMat[side].at<float>(i, 1),
									   &IntrinsicMat[side].at<float>(i, 2));
		}
		fscanf_s(fp, "%f %f %f %f\n", &DistParam[side].at<float>(0, 0),
									  &DistParam[side].at<float>(0, 1),
									  &DistParam[side].at<float>(0, 2),
									  &DistParam[side].at<float>(0, 3));
		fclose(fp);

		return true;
	}
}

//void SaveExtrinsicParameters(char * Filename)
//{
//	FILE * fp;
//
//	errno_t err = fopen_s(&fp, Filename, "w");
//	for (int i = 0; i < 3; i++)
//	{
//		fprintf(fp, "%f %f %f %f\n", RotationMat.at<double>(i, 0),
//									 RotationMat.at<double>(i, 1),
//									 RotationMat.at<double>(i, 2),
//									 TranslationVec.at<double>(i, 0));
//	}
//	fprintf(fp, "0.0 0.0 0.0 1.0");
//	fclose(fp);
//}

static void error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
}