#include <windows.h>
#include "gl/gl.h"
#include "gl/glu.h"
#include "gl/glut.h"
#include "opencv2/opencv.hpp"

#pragma comment(lib,"opencv_world310d.lib")

cv::Mat					frame;
cv::VideoCapture	*	cap = NULL;

void Init();
void DispFunc();
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
}

void DispFunc()
{
	// clear the window
	glClear(GL_COLOR_BUFFER_BIT);

	cap->read(frame);				// capture image

	// flip camera frame
	cv::Mat tempimage;
	cv::flip(frame, tempimage, 0);
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

//void myKeyboard(unsigned char key, int mouseX, int mouseY)
//{
//}

//void myReshape(int width, int height)
//{
//}