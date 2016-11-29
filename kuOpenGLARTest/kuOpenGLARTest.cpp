#include <windows.h>
#include <opencv2/opencv.hpp>
#include <GLEW/glew.h>
#include <GLFW/glfw3.h>
#include <GLM/glm.hpp>
#include <GLM/gtc/matrix_transform.hpp>
#include <GLM/gtc/type_ptr.hpp>

#include "kuMiMCamera.h"
#include "kuShaderHandler.h"

#pragma comment(lib, "opencv_world310d.lib")
#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "glew32.lib")
#pragma comment(lib, "opengl32.lib")

#pragma comment(lib, "opencv_world310d.lib")

using namespace cv;
using namespace std;

#define		pi			3.1415926
#define		WndWidth	640
#define		WndHieght	480
#define     farClip		5000.0
#define		nearClip	1.0

kuMiMCamera				MiMCam;

int						ImgWidth;
int						ImgHeight;

Mat						GrayImg;

Mat						IntrinsicMat;
Mat						DistParam;
vector<Point3f>			CB3DPts;
vector<Point2f>			CB2DPts;
vector<Point2f>			Projected2DPts;
Mat						RotationVec;
Mat						RotationMat;
Mat						TranslationVec;

GLFWwindow			*	window;

bool					isIntrinsicLoaded;

GLfloat					IntrinsicProjMat[16];
GLfloat					ExtrinsicViewMat[16];

void					Init();
static void				error_callback(int error, const char* description);
static void				key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

void					DispParam();
void					DispExtParam();
void					SetCB3DPts();
bool					LoadCameraParameters(char * Filename);
void					SaveExtrinsicParameters(char * Filename);
void					DrawAxes(float length);
void					IntrinsicCVtoGL(Mat IntParam, GLfloat GLProjection[16]);
void					ExtrinsicCVtoGL(Mat RotMat, Mat TransVec, GLfloat GLModelView[16]);

GLuint					CreateTexturebyImage(Mat Img);
void					DrawBGImage(Mat BGImg, kuShaderHandler BGShader);

const GLfloat	CubeVertices[]
= {
	// Frontal Face
	-50.0f,  50.0f,  100.0f,			// 0:  Top Left  
	 50.0f,  50.0f,  100.0f,			// 1:  Top Right
	 50.0f, -50.0f,  100.0f,			// 2:  Bottom Right
	-50.0f, -50.0f,  100.0f,			// 3:  Bottom Left

	// Right Face
	-50.0f,  50.0f,   0.0f,			// 4:  Top Left
	-50.0f,  50.0f,  100.0f,			// 5:  Top Right
	-50.0f, -50.0f,  100.0f,			// 6:  Bottom Right
	-50.0f, -50.0f,   0.0f,			// 7:  Bottom Left

	// Back face
	 50.0f,  50.0f,   0.0f,			// 8:  Top Left 
	-50.0f,  50.0f,   0.0f,			// 9:  Top Right
	-50.0f, -50.0f,   0.0f,			// 10: Bottom Right
	 50.0f, -50.0f,   0.0f,			// 11: Bottom Left

	// Left Face
	 50.0f,  50.0f,  100.0f,			// 12: Top Left 
	 50.0f,  50.0f,   0.0f, 		// 13: Top Right
	 50.0f, -50.0f,   0.0f, 		// 14: Bottom Right
	 50.0f, -50.0f,  100.0f,  		// 15: Bottom Left

	// Up Face
	-50.0f,  50.0f,   0.0f,  		// 16: Top Left 
	 50.0f,  50.0f,   0.0f,  		// 17: Top Right
	 50.0f,  50.0f,  100.0f,  		// 18: Bottom Right
	-50.0f,  50.0f,  100.0f,  		// 19: Bottom Left

	// Down Face
	-50.0f, -50.0f,  100.0f,			// 20: Top Left 
	 50.0f, -50.0f,  100.0f,  		// 21: Top Right
	 50.0f, -50.0f,   0.0f,    		// 22: Bottom Right
	-50.0f, -50.0f,   0.0f  		// 23: Bottom Left
};

const GLfloat   CubeTexCoords[]
= {
	0.0f, 0.0f,    1.0f, 0.0f,    1.0f, 1.0f,    0.0f, 1.0f,
	0.0f, 0.0f,    1.0f, 0.0f,    1.0f, 1.0f,    0.0f, 1.0f,
	0.0f, 0.0f,    1.0f, 0.0f,    1.0f, 1.0f,    0.0f, 1.0f,
	0.0f, 0.0f,    1.0f, 0.0f,    1.0f, 1.0f,    0.0f, 1.0f,
	0.0f, 0.0f,    1.0f, 0.0f,    1.0f, 1.0f,    0.0f, 1.0f,
	0.0f, 0.0f,    1.0f, 0.0f,    1.0f, 1.0f,    0.0f, 1.0f
};

const GLuint    CubeIndices[]
= {
	// Frontal 
	0,  1,  3,	  1,  2,  3,
	// Right
	4,  5,  7,	  5,  6,  7,
	// Back
	8,  9, 11,	  9, 10, 11,
	// Left
	12, 13, 15,  13, 14, 15,
	// Up
	16, 17, 19,  17, 18, 19,
	// Down 
	20, 21, 23,  21, 22, 23
};

int main()
{
	Init();

	kuShaderHandler		BGImgShaderHandler;
	kuShaderHandler		ObjShaderHandler;
	BGImgShaderHandler.Load("BGImgVertexShader.vert", "BGImgFragmentShader.frag");
	ObjShaderHandler.Load("ObjectVertexShader.vert", "ObjectFragmentShader.frag");

	GLuint CubeVertexArray = 0;
	glGenVertexArrays(1, &CubeVertexArray);
	GLuint CubeVertexBuffer = 0;							// Vertex Buffer Object (VBO)
	glGenBuffers(1, &CubeVertexBuffer);						// give an ID to vertex buffer
	GLuint CubeTexCoordBuffer = 0;
	glGenBuffers(1, &CubeTexCoordBuffer);
	GLuint CubeElementBuffer = 0;							// Element Buffer Object (EBO)
	glGenBuffers(1, &CubeElementBuffer);

	glBindVertexArray(CubeVertexArray);

	glBindBuffer(GL_ARRAY_BUFFER, CubeVertexBuffer);		// Bind buffer as array buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(CubeVertices), CubeVertices, GL_STATIC_DRAW);
	// Position
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, CubeTexCoordBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(CubeTexCoords), CubeTexCoords, GL_STATIC_DRAW);
	// TexCoord
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, CubeElementBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(CubeIndices), CubeIndices, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	Mat		CubeTextureImg = imread("ihatepeople.jpg");
	GLuint	CubeTextureID = CreateTexturebyImage(CubeTextureImg);

	GLuint		ModelMatLoc, ViewMatLoc, ProjMatLoc;
	glm::mat4	ModelMat, ProjMat, ViewMat;

	ViewMatLoc  = glGetUniformLocation(ObjShaderHandler.ShaderProgramID, "ViewMat");
	ProjMatLoc  = glGetUniformLocation(ObjShaderHandler.ShaderProgramID, "ProjMat");
	ModelMatLoc = glGetUniformLocation(ObjShaderHandler.ShaderProgramID, "ModelMat");

	ProjMat = glm::perspective(45.0f, (GLfloat)ImgWidth / (GLfloat)ImgHeight, (GLfloat)nearClip, (GLfloat)farClip);

	while (!glfwWindowShouldClose(window))
	{
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		MiMCam.CaptureFrame();

		cvtColor(MiMCam.m_CamFrame, GrayImg, CV_BGR2GRAY);

		bool CBFound = findChessboardCorners(GrayImg, Size(5, 7), CB2DPts,
											 CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
		//drawChessboardCorners(CamFrame, Size(5, 7), Mat(CB2DPts), CBFound);

		DrawBGImage(MiMCam.m_CamFrame, BGImgShaderHandler);

		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);

		if (CBFound)
		{
			solvePnP(CB3DPts, CB2DPts, IntrinsicMat, DistParam, RotationVec, TranslationVec);
			Rodrigues(RotationVec, RotationMat);

			ExtrinsicCVtoGL(RotationMat, TranslationVec, ExtrinsicViewMat);

			ObjShaderHandler.Use();

			glBindTexture(GL_TEXTURE_2D, CubeTextureID);

			glUniformMatrix4fv(ModelMatLoc, 1, GL_FALSE, glm::value_ptr(ModelMat));
			glUniformMatrix4fv(ViewMatLoc, 1, GL_FALSE, ExtrinsicViewMat);
			glUniformMatrix4fv(ProjMatLoc, 1, GL_FALSE, IntrinsicProjMat);

			glBindVertexArray(CubeVertexArray);
			glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
			glBindVertexArray(0);
		}

		glfwSwapBuffers(window);
		glfwPollEvents();					// This function processes only those events that are already 
											// in the event queue and then returns immediately
	}

	glfwDestroyWindow(window);
	glfwTerminate();
	MiMCam.CloseCamera();

	exit(EXIT_SUCCESS);
}

void Init()
{
	MiMCam.InitialCamera();
	ImgWidth  = MiMCam.m_ImageWidth;
	ImgHeight = MiMCam.m_ImageHeight;

	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	window = glfwCreateWindow(WndWidth, WndHieght, "OpenGLWnd", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);					// the number of screen updates to wait from the time

	// need to create OpenGL window before glew initialization.
	//glewExperimental = GL_TRUE;
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		//Problem: glewInit failed, something is seriously wrong.
		cout << "glewInit failed, aborting." << endl;
	}

	// Define the viewport dimensions
	glViewport(0, 0, WndWidth, WndHieght);

	// Setup OpenGL options (z-buffer)
	glEnable(GL_DEPTH_TEST);

	// get version info
	const GLubyte* renderer = glGetString(GL_RENDERER);
	// get renderer string (graphic card)
	const GLubyte* version = glGetString(GL_VERSION);
	// version as a string (OpenGL supported version and graphic card driver version)
	cout << "Renderer: " << renderer << endl;
	cout << "OpenGL version supported " << version << endl;

	glfwSetKeyCallback(window, key_callback);

	IntrinsicMat.create(3, 3, CV_32FC1);
	DistParam.create(1, 4, CV_32FC1);
	RotationVec.create(3, 1, CV_64FC1);
	RotationMat.create(3, 3, CV_64FC1);
	TranslationVec.create(3, 1, CV_64FC1);

	if (isIntrinsicLoaded = LoadCameraParameters("IntParam_Left_MiM.txt"))
	{
		IntrinsicCVtoGL(IntrinsicMat, IntrinsicProjMat);

		for (int i = 0; i < 4; i++)
		{
			cout << IntrinsicProjMat[4 * i] << " " << IntrinsicProjMat[4 * i + 1] << " "
				 << IntrinsicProjMat[4 * i + 2] << " " << IntrinsicProjMat[4 * i + 3] << endl;
		}
		cout << endl;

		DispParam();
	}

	SetCB3DPts();
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

void IntrinsicCVtoGL(Mat IntParam, GLfloat GLProjection[16])
{
	int			i, j;
	double		p[3][3];
	double		q[4][4];

	memset(GLProjection, 0, 16 * sizeof(GLfloat));

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

void ExtrinsicCVtoGL(Mat RotMat, Mat TransVec, GLfloat GLModelView[16])
{
	memset(GLModelView, 0, 16 * sizeof(GLfloat));

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
	FILE * fp;

	errno_t err = fopen_s(&fp, "CB3DPts.txt", "w");
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			CB3DPts.push_back(Point3f(-50 + 25 * j, 75 - 25 * i, 0));

			fprintf_s(fp, "%f %f %f\n", CB3DPts[5*i+j].x, CB3DPts[5 * i + j].y, CB3DPts[5 * i + j].z);
		}
	}
	fclose(fp);
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

GLuint CreateTexturebyImage(Mat Img)
{
	GLuint	texture;

	for (int i = 0; i < Img.cols; i++)
	{
		for (int j = 0; j < Img.rows; j++)
		{
			int		PixelIdx = Img.cols * j + i;
			uchar	temp;

			temp = Img.data[3 * PixelIdx];
			Img.data[3 * PixelIdx] = Img.data[3 * PixelIdx + 2];
			Img.data[3 * PixelIdx + 2] = temp;
		}
	}

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);

	// Set the texture wrapping parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// Set texture wrapping to GL_REPEAT (usually basic wrapping method)
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// Set texture filtering parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, Img.cols, Img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, Img.data);
	glGenerateMipmap(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);

	return texture;
}

void DrawBGImage(Mat BGImg, kuShaderHandler BGShader)
{
	static const GLfloat BGVertices[]
	= {
		1.0f,  1.0f, 1.0f, 0.0f,
		1.0f, -1.0f, 1.0f, 1.0f,
		-1.0f, -1.0f, 0.0f, 1.0f,
		-1.0f,  1.0f, 0.0f, 0.0f
	};

	static const GLfloat BGTexCoords[]
	= {
		0.0
	};

	GLuint indices[]
	= { 0, 1, 3,
		1, 2, 3 };

	GLuint BGVertexArray = 0;
	glGenVertexArrays(1, &BGVertexArray);
	GLuint BGVertexBuffer = 0;						// Vertex Buffer Object (VBO)
	glGenBuffers(1, &BGVertexBuffer);				// give an ID to vertex buffer
	GLuint BGElementBuffer = 0;						// Element Buffer Object (EBO)
	glGenBuffers(1, &BGElementBuffer);

	glBindVertexArray(BGVertexArray);

	glBindBuffer(GL_ARRAY_BUFFER, BGVertexBuffer);  // Bind buffer as array buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(BGVertices), BGVertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, BGElementBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	// Assign vertex position data
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	// Assign texture coordinates
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (GLvoid*)(2 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);

	GLuint BGImgTextureID = CreateTexturebyImage(BGImg);

	BGShader.Use();

	glBindTexture(GL_TEXTURE_2D, BGImgTextureID);

	glBindVertexArray(BGVertexArray);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
	glBindTexture(GL_TEXTURE_2D, 0);

	glDeleteTextures(1, &BGImgTextureID);

	glDeleteVertexArrays(1, &BGVertexArray);
	glDeleteBuffers(1, &BGVertexBuffer);
	glDeleteBuffers(1, &BGElementBuffer);
}
