#include <windows.h>
#include "gl/gl.h"
#include "gl/glu.h"
#include "gl/glut.h"
#include "opencv2/opencv.hpp"

#pragma comment(lib,"opencv_world310d.lib")

using namespace cv;
using namespace std;

#define		ImgWidth	640
#define		ImgHeight	480

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

float				fx, fy, px, py;

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
void DrawCubeCV(Mat Img, vector<Point2f> CubeVertex, const cv::Scalar &color);
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

//	frame = imread("CamFrame.bmp", 1);

	if (LoadCameraParameters("IntParam_Left.txt"))
	{
		fx = IntParam.at<float>(0, 0);
		fy = IntParam.at<float>(1, 1);
		px = IntParam.at<float>(0, 2);
		py = IntParam.at<float>(1, 2);

		fovx = 2 * atan(0.5 * 640 / fx) * 180 / 3.1415926;
		fovy = 2 * atan(0.5 * 480 / fy) * 180 / 3.1415926;
		aspect = (640 * fy) / (480 * fx);

		DispParam();
	}

	SetCB3DPts();
}

void DispFunc()
{
	Mat UndistortImg;

	// clear the window
	glClear(GL_COLOR_BUFFER_BIT);

	cap->read(frame);				// capture image
	
	// flip camera frame
	Mat tempimage;
	Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1); 
	Mat ProjectionMatrix = cv::Mat::zeros(4, 4, CV_64FC1);
	cv::Mat glViewMatrix = cv::Mat::zeros(4, 4, CV_64F);
	cv::Mat glProjectionMatrix = cv::Mat::zeros(4, 4, CV_64F);
	
	undistort(frame,UndistortImg,IntParam,DistParam);
	cvtColor(frame, GrayImg, CV_RGB2GRAY);

	bool CBFound = findChessboardCorners(GrayImg, Size(5, 7), CB2DPts,
										 CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
	drawChessboardCorners(UndistortImg, Size(5, 7), Mat(CB2DPts), CBFound);

	if (CBFound)
	{
		//imwrite("CamFrame.bmp", FrameRaw);

		solvePnP(CB3DPts, CB2DPts, IntParam, DistParam, RotationVec, TranslationVec);
		Rodrigues(RotationVec, RotationMat);

		
		vector<Point3f> CubeA3DPts;
		vector<Point3f> CubeB3DPts;

		vector<Point2f> CubeAProjected2DPts;
		vector<Point2f> CubeBProjected2DPts;


		CubeA3DPts.resize(8);
		CubeA3DPts[0] = Point3f(-25, 0, 0);
		CubeA3DPts[1] = Point3f(0, 0, 0);
		CubeA3DPts[2] = Point3f(0, 25, 0);
		CubeA3DPts[3] = Point3f(-25, 25, 0);
		CubeA3DPts[4] = Point3f(-25, 0, 25);
		CubeA3DPts[5] = Point3f(0, 0, 25);
		CubeA3DPts[6] = Point3f(0, 25, 25);
		CubeA3DPts[7] = Point3f(-25, 25, 25);

		CubeB3DPts.resize(8);
		CubeB3DPts[0] = Point3f(-50, 0, 0);
		CubeB3DPts[1] = Point3f(0, 0, 0);
		CubeB3DPts[2] = Point3f(0, 50, 0);
		CubeB3DPts[3] = Point3f(-50, 50, 0);
		CubeB3DPts[4] = Point3f(-50, 0, 50);
		CubeB3DPts[5] = Point3f(0, 0, 50);
		CubeB3DPts[6] = Point3f(0, 50, 50);
		CubeB3DPts[7] = Point3f(-50, 50, 50);


		projectPoints(CubeA3DPts,
					  RotationVec, TranslationVec,
					  IntParam, DistParam, CubeAProjected2DPts);

		projectPoints(CubeB3DPts,
					  RotationVec, TranslationVec,
					  IntParam, DistParam, CubeBProjected2DPts);


		DrawCubeCV(frame, CubeAProjected2DPts, CV_RGB(0,255,0));
		DrawCubeCV(frame, CubeBProjected2DPts, CV_RGB(255,0,0));
		

		ProjectionMatrix.at<double>(0, 0) = fx/px;
		ProjectionMatrix.at<double>(1, 1) = fy/py;
		ProjectionMatrix.at<double>(2, 2) = -(10000 + 40) / (10000 - 40);
		ProjectionMatrix.at<double>(2, 3) = -2.0 * 10000 * 40 / (10000 - 40);
		ProjectionMatrix.at<double>(3, 2) = -1;
		cv::transpose(ProjectionMatrix, glProjectionMatrix);

		for (unsigned int row = 0; row < 4; ++row)
		{
			for (unsigned int col = 0; col < 4; ++col)
			{
				cout << ProjectionMatrix.at<double>(row, col) << "¡@";
			}
			cout << endl;
		}

		for (unsigned int row = 0; row<3; ++row)
		{
			for (unsigned int col = 0; col<3; ++col)
			{
				viewMatrix.at<double>(row, col) = RotationMat.at<double>(row, col);
			}
			viewMatrix.at<double>(row, 3) = TranslationVec.at<double>(row, 0);
		}
		viewMatrix.at<double>(3, 3) = 1.0f;

		cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F); 
		cvToGl.at<double>(0, 0) = 1.0f; 
		cvToGl.at<double>(1, 1) = -1.0f; 
		// Invert the y axis 
		cvToGl.at<double>(2, 2) = -1.0f; 
		// invert the z axis 
		cvToGl.at<double>(3, 3) = 1.0f; 
		viewMatrix = cvToGl * viewMatrix;

		cv::transpose(viewMatrix, glViewMatrix);

		DispExtParam();
		SaveExtrinsicParameters("ExtParam_Left.txt");

		for (unsigned int row = 0; row < 4; ++row)
		{
			for (unsigned int col = 0; col < 4; ++col)
			{
				cout << glViewMatrix.at<double>(row, col) << "¡@";
			}
			cout << endl;
		}
	}
	
	imshow("ImageQQ", frame);

	flip(UndistortImg, tempimage, 0);
	glDrawPixels(tempimage.size().width, tempimage.size().height, GL_BGR_EXT, GL_UNSIGNED_BYTE, tempimage.ptr());

	// set viewport
	glViewport(0, 0, tempimage.size().width, tempimage.size().height);

	// set projection matrix using intrinsic camera params
	glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	glLoadMatrixd(&glProjectionMatrix.at<double>(0, 0));

	//gluPerspective is arbitrarily set, you will have to determine these values based
	//on the intrinsic camera parameters
	//gluPerspective(fovy, aspect, 40, 10000);

	// you will have to set modelview matrix using extrinsic camera params
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLoadMatrixd(&glViewMatrix.at<double>(0, 0));
//	gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);

	/////////////////////////////////////////////////////////////////////////////////
	// Drawing routine

	//now that the camera params have been set, draw your 3D shapes
	//first, save the current matrix
	glPushMatrix();
	//move to the position where you want the 3D object to go
	glTranslatef(0, 0, 0); //this is an arbitrary position for demonstration
						   //you will need to adjust your transformations to match the positions where
						   //you want to draw your objects(i.e. chessboard center, chessboard corners)
//	glutWireTeapot(0.5);
//	glutSolidSphere(.3, 100, 100);
	glutWireCube(25);
	DrawAxes(25.0);
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

void DrawCubeCV(Mat Img, vector<Point2f> CubeVertex, const cv::Scalar &color)
{
	for (int i = 0; i < 8; i++)
	{
		circle(Img, CubeVertex[i], 1, color, 5, CV_AA);
	}

	line(Img, CubeVertex[0], CubeVertex[1], color, 1, CV_AA);
	line(Img, CubeVertex[1], CubeVertex[2], color, 1, CV_AA);
	line(Img, CubeVertex[2], CubeVertex[3], color, 1, CV_AA);
	line(Img, CubeVertex[3], CubeVertex[0], color, 1, CV_AA);

	line(Img, CubeVertex[4], CubeVertex[5], color, 1, CV_AA);
	line(Img, CubeVertex[5], CubeVertex[6], color, 1, CV_AA);
	line(Img, CubeVertex[6], CubeVertex[7], color, 1, CV_AA);
	line(Img, CubeVertex[7], CubeVertex[4], color, 1, CV_AA);

	line(Img, CubeVertex[0], CubeVertex[4], color, 1, CV_AA);
	line(Img, CubeVertex[1], CubeVertex[5], color, 1, CV_AA);
	line(Img, CubeVertex[2], CubeVertex[6], color, 1, CV_AA);
	line(Img, CubeVertex[3], CubeVertex[7], color, 1, CV_AA);
}

//void myKeyboard(unsigned char key, int mouseX, int mouseY)
//{
//}

//void myReshape(int width, int height)
//{
//}