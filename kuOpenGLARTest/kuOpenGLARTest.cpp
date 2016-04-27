#include <windows.h>
#include "gl/gl.h"
#include "gl/glu.h"
#include "gl/glut.h"
#include "opencv2/opencv.hpp"

#pragma comment(lib,"opencv_world310d.lib")

using namespace cv;
using namespace std;

Mat					FrameRaw;
Mat					frame;
Mat					GrayImg;
VideoCapture	*	cap = NULL;

Mat					IntParam;
Mat					DistParam;
vector<Point3f>		CB3DPts;
vector<Point2f>		CB2DPts;
vector<Point2f>		Projected2DPts;
Mat					RotationVec;
Mat					RotationMat;
Mat					TranslationVec;

float				fovx;
float				fovy;
float				aspect;

void Init();
void DispFunc();
void DispParam();
void DispExtParam();
void SetCB3DPts();
bool LoadCameraParameters(char * Filename);
void SaveExtrinsicParameters(char * Filename);
void DrawAxes(float length);
//void myKeyboard(unsigned char key, int mouseX, int mouseY);
//void myReshape(int width, int height);

int main()
{
	Init();

	// set the display callback
	glutDisplayFunc(DispFunc);

//	glutKeyboardFunc(myKeyboard);
//	glutReshapeFunc(myReshape);

	glutMainLoop();
}

void Init()
{
	// initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("OpenGLQQ");

	// initialize OpenCV video capture
	cap = new cv::VideoCapture(0);

	IntParam.create(3, 3, CV_32FC1);
	DistParam.create(1, 4, CV_32FC1);
	RotationVec.create(3, 1, CV_64FC1);
	RotationMat.create(3, 3, CV_64FC1);
	TranslationVec.create(3, 1, CV_64FC1);

	frame = imread("CamFrame.bmp", 1);

	if (LoadCameraParameters("IntParam_Left.txt"))
	{
		float fx = IntParam.at<float>(0, 0);
		float fy = IntParam.at<float>(1, 1);

		fovx = 2 * atan(0.5 * 640 / fx) * 180 / 3.1415926;
		fovy = 2 * atan(0.5 * 480 / fy) * 180 / 3.1415926;
		aspect = (640 * fy) / (480 * fx);

		DispParam();
	}

	SetCB3DPts();
}

void DispFunc()
{
	// clear the window
	glClear(GL_COLOR_BUFFER_BIT);

	//cap->read(frame);				// capture image
	
	// flip camera frame
	Mat tempimage;
	flip(frame, tempimage, 0);

	cvtColor(tempimage, GrayImg, CV_RGB2GRAY);

	bool CBFound = findChessboardCorners(GrayImg, Size(5, 7), CB2DPts,
										 CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
	drawChessboardCorners(tempimage, Size(5, 7), Mat(CB2DPts), CBFound);

	if (CBFound)
	{
		//imwrite("CamFrame.bmp", FrameRaw);

		solvePnP(CB3DPts, CB2DPts, IntParam, DistParam, RotationVec, TranslationVec);
		Rodrigues(RotationVec, RotationMat);

		vector<Point3f> Test3DPts;
		vector<Point2f> Projected2DPts;

		Test3DPts.resize(4);
		Test3DPts[0] = Point3f(12.5, 0, 0);
		Test3DPts[1] = Point3f(37.5, 25, 0);
		Test3DPts[2] = Point3f(37.5, 75, 0);
		Test3DPts[3] = Point3f(-37.5, 100, 0);

		projectPoints(Test3DPts,
			RotationVec, TranslationVec,
			IntParam, DistParam, Projected2DPts);

		for (int i = 0; i < 4; i++)
		{
			circle(tempimage, Projected2DPts[i], 1, CV_RGB(255, 0, 0), 5, CV_AA);
		}

		Test3DPts.clear();

		DispExtParam();
		SaveExtrinsicParameters("ExtParam_Left.txt");
	}

	glDrawPixels(tempimage.size().width, tempimage.size().height, GL_BGR_EXT, GL_UNSIGNED_BYTE, tempimage.ptr());

	// set viewport
	glViewport(0, 0, tempimage.size().width, tempimage.size().height);

	// set projection matrix using intrinsic camera params
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//gluPerspective is arbitrarily set, you will have to determine these values based
	//on the intrinsic camera parameters
	gluPerspective(60, tempimage.size().width*1.0 / tempimage.size().height, 1, 20);

	// you will have to set modelview matrix using extrinsic camera params
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);

	/////////////////////////////////////////////////////////////////////////////////
	// Drawing routine

	//now that the camera params have been set, draw your 3D shapes
	//first, save the current matrix
	glPushMatrix();
	//move to the position where you want the 3D object to go
	glTranslatef(0, 0, 0); //this is an arbitrary position for demonstration
						   //you will need to adjust your transformations to match the positions where
						   //you want to draw your objects(i.e. chessboard center, chessboard corners)
//	glutSolidTeapot(0.5);
	glutSolidSphere(.3, 100, 100);
	DrawAxes(1.0);
	glPopMatrix();
	glClear(GL_DEPTH_BUFFER_BIT);

	// show the rendering on the screen
	glutSwapBuffers();

	// post the next redisplay
	glutPostRedisplay();
}

void DrawAxes(float length)
{
	glPushAttrib(GL_POLYGON_BIT | GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_LIGHTING);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(length, 0, 0);

	glColor3f(0, 1, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, length, 0);

	glColor3f(0, 0, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, length);
	glEnd();

	glPopAttrib();
}

void SetCB3DPts()
{
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			CB3DPts.push_back(Point3f(50 - 25 * j, 25 * i, 0));
		}
	}
}

void DispParam()
{
	for (int i = 0; i < 3; i++)
	{
		printf("%f %f %f\n", IntParam.at<float>(i, 0), IntParam.at<float>(i, 1),
			IntParam.at<float>(i, 2));
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
			fscanf_s(fp, "%f %f %f\n", &IntParam.at<float>(i, 0),
				&IntParam.at<float>(i, 1),
				&IntParam.at<float>(i, 2));


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

//void myKeyboard(unsigned char key, int mouseX, int mouseY)
//{
//}

//void myReshape(int width, int height)
//{
//}