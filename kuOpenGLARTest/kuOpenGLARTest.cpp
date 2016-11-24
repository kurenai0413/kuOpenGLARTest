#include <windows.h>
#include "gl/glut.h"
#include "opencv2/opencv.hpp"
#include <GLFW/glfw3.h>

#pragma comment(lib, "opencv_world310d.lib")
#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "opengl32.lib")

#pragma comment(lib, "opencv_world310d.lib")

using namespace cv;
using namespace std;

#define		ImgWidth	640
#define		ImgHeight	480
#define     farClip		50
#define		nearClip	5000

Mat					FrameRaw;
Mat					frame;
Mat					GrayImg;
VideoCapture	*	cap = NULL;

Mat					IntrinsicMat;
Mat					DistParam;
vector<Point3f>		CB3DPts;
vector<Point2f>		CB2DPts;
vector<Point2f>		Projected2DPts;
Mat					RotationVec;
Mat					RotationMat;
Mat					TranslationVec;

GLFWwindow		*	window;

bool				isIntrinsicLoaded;

double				m[16];

void Init();
static void error_callback(int error, const char* description);
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

void DispFunc();
void DispParam();
void DispExtParam();
void SetCB3DPts();
bool LoadCameraParameters(char * Filename);
void SaveExtrinsicParameters(char * Filename);
void DrawAxes(float length);
void IntrinsicCVtoGL(Mat IntParam, double GLProjection[16]);
void ExtrinsicCVtoGL(Mat RotMat, Mat TransVec, double GLModelView[16]);
//void myKeyboard(unsigned char key, int mouseX, int mouseY);
//void myReshape(int width, int height);

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
	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	window = glfwCreateWindow(ImgWidth, ImgHeight, "Stereo", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);					// the number of screen updates to wait from the time
	glfwSetKeyCallback(window, key_callback);

	// initialize OpenCV video capture
	cap = new VideoCapture(0);

	IntrinsicMat.create(3, 3, CV_32FC1);
	DistParam.create(1, 4, CV_32FC1);
	RotationVec.create(3, 1, CV_64FC1);
	RotationMat.create(3, 3, CV_64FC1);
	TranslationVec.create(3, 1, CV_64FC1);

	if (isIntrinsicLoaded = LoadCameraParameters("IntParam_Left.txt"))
	{
		// set projection matrix using intrinsic camera params
		glMatrixMode(GL_PROJECTION);
		IntrinsicCVtoGL(IntrinsicMat, m);
		glLoadMatrixd(m);

		DispParam();
	}

	SetCB3DPts();
}

void DispFunc()
{
	Mat tempimage;
	Mat UndistortImg;

	// clear the window
	glClear(GL_COLOR_BUFFER_BIT);

	cap->read(frame);				// capture image

	if (isIntrinsicLoaded)
	{
		undistort(frame, UndistortImg, IntrinsicMat, DistParam);
	}

	cvtColor(frame, GrayImg, CV_RGB2GRAY);

	bool CBFound = findChessboardCorners(GrayImg, Size(5, 7), CB2DPts,
										 CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
	//drawChessboardCorners(UndistortImg, Size(5, 7), Mat(CB2DPts), CBFound);

	if (CBFound)
	{
		solvePnP(CB3DPts, CB2DPts, IntrinsicMat, DistParam, RotationVec, TranslationVec);
		Rodrigues(RotationVec, RotationMat);
	}
	
	flip(UndistortImg, tempimage, 0);
	glDrawPixels(ImgWidth, ImgHeight,
				 GL_BGR_EXT, GL_UNSIGNED_BYTE, tempimage.ptr());
	
	// set viewport
	glViewport(0, 0, ImgWidth, ImgHeight);

	// you will have to set modelview matrix using extrinsic camera params
	double gl_para[16];
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	ExtrinsicCVtoGL(RotationMat, TranslationVec, gl_para);
	glLoadMatrixd(gl_para);

	FILE * fp;
	errno_t err = fopen_s(&fp, "GLParam.txt", "w");
	fprintf(fp, "m:\n");
	for (int i = 0; i < 4; i++)
	{
		fprintf(fp, "%f %f %f %f\n", m[4 * i], m[4 * i + 1], m[4 * i + 2], m[4 * i + 3]);
	}
	fprintf(fp, "\ngl_para:\n");
	for (int i = 0; i < 4; i++)
	{
		fprintf(fp, "%f %f %f %f\n", gl_para[4 * i], gl_para[4 * i + 1], gl_para[4 * i + 2], gl_para[4 * i + 3]);
	}
	fclose(fp);

	/////////////////////////////////////////////////////////////////////////////////
	// Drawing routine

	//now that the camera params have been set, draw your 3D shapes
	//first, save the current matrix
	glPushMatrix();
	//move to the position where you want the 3D object to go
//	glutWireTeapot(0.5);
//	glutSolidSphere(.3, 100, 100);
//	glutWireCube(25);
//	DrawAxes(25.0);
	glColor3f(0, 1, 0);
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			glPushMatrix();
			glTranslatef(-37.5f + 50 * i, 62.5f - 50 * j, 12.5f);
			glutWireCube(25.0);
			glPopMatrix();
			glPushMatrix();
			glTranslatef(-12.5f + 50 * i, 37.5f - 50 * j, 12.5f);
			glutWireCube(25.0);
			glPopMatrix();
		}
	}

	glPopMatrix();
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

void SetCB3DPts()
{
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			CB3DPts.push_back(Point3f(-50 + 25 * j, 75 - 25 * i, 0));
		}
	}
}

void DispParam()
{
	for (int i = 0; i < 3; i++)
	{
		printf("%f %f %f\n", IntrinsicMat.at<float>(i, 0), IntrinsicMat.at<float>(i, 1),
							 IntrinsicMat.at<float>(i, 2));
	}
	printf("%f %f %f %f\n", DistParam.at<float>(0, 0), DistParam.at<float>(0, 1),
							DistParam.at<float>(0, 2), DistParam.at<float>(0, 3));
}

void DispExtParam()
{
	for (int i = 0; i < 3; i++)
	{
		cout << RotationMat.at<double>(i, 0) << " "
			 << RotationMat.at<double>(i, 1) << " "
			 << RotationMat.at<double>(i, 2) << endl;
	}

	cout << endl;
	cout << TranslationVec.at<double>(0, 0) << " "
		 << TranslationVec.at<double>(1, 0) << " "
		 << TranslationVec.at<double>(2, 0) << endl;
	cout << endl;
}

bool LoadCameraParameters(char * Filename)
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
			fscanf_s(fp, "%f %f %f\n", &IntrinsicMat.at<float>(i, 0),
									   &IntrinsicMat.at<float>(i, 1),
									   &IntrinsicMat.at<float>(i, 2));
		}
		fscanf_s(fp, "%f %f %f %f\n", &DistParam.at<float>(0, 0),
									  &DistParam.at<float>(0, 1),
									  &DistParam.at<float>(0, 2),
									  &DistParam.at<float>(0, 3));
		fclose(fp);

		return true;
	}
}

void SaveExtrinsicParameters(char * Filename)
{
	FILE * fp;

	errno_t err = fopen_s(&fp, Filename, "w");
	for (int i = 0; i < 3; i++)
	{
		fprintf(fp, "%f %f %f %f\n", RotationMat.at<double>(i, 0),
									 RotationMat.at<double>(i, 1),
									 RotationMat.at<double>(i, 2),
									 TranslationVec.at<double>(i, 0));
	}
	fprintf(fp, "0.0 0.0 0.0 1.0");
	fclose(fp);
}

static void error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);
}
