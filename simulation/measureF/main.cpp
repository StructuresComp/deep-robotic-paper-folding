/**
 * simDER
 * simDER stands for "[sim]plified [D]iscrete [E]lastic [R]ods"
 * Dec 2017
 * This code is based on previous iterations.
 * */

//This line is for mac
//#include <GLUT/glut.h>

//This is for linux
#include <GL/glut.h>

#include <iostream>
#include <fstream>
#include <string>
#include "environment/eigenIncludes.h"

// Rod and stepper are included in the world
#include "world.h"
#include "setInput.h"

world myWorld;
int NPTS;
ofstream outfile;
ofstream nodefile;

static void Key(unsigned char key, int x, int y)
{
  switch (key) // ESCAPE to quit
  {
	case 27:
		exit(0);
  }
}

/* Initialize OpenGL Graphics */
void initGL()
{
	glClearColor(0.7f, 0.7f, 0.7f, 0.0f); // Set background color to black and opaque
	glClearDepth(10.0f);                   // Set background depth to farthest
	//glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	//glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	//glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections

	glLoadIdentity();
	gluLookAt(0.05, 0.05, 0.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
	glPushMatrix();

	//glMatrixMode(GL_MODELVIEW);
}

void display(void)
{

	double currentTime  = 0;
	while ( myWorld.simulationRunning() > 0)
	{
		//  Clear screen and Z-buffer
		glClear(GL_COLOR_BUFFER_BIT);

		// draw axis
		double axisLen = 1;
		glLineWidth(0.5);

		glBegin(GL_LINES);
			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(axisLen, 0.0, 0.0);

			glColor3f(0.0, 1.0, 0.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, axisLen, 0.0);

			glColor3f(0.0, 0.0, 1.0);
			glVertex3f(0.0, 0.0, 0.0);
			glVertex3f(0.0, 0.0, axisLen);
		glEnd();

		//draw a line
		glColor3f(0.1, 0.1, 0.1);
		glLineWidth(3.0);

		glBegin(GL_LINES);
		for (int i=0; i < NPTS-1; i++)
		{
			glVertex3f( myWorld.getScaledCoordinate(4*i), myWorld.getScaledCoordinate(4*i+1), myWorld.getScaledCoordinate(4*i+2));
			glVertex3f( myWorld.getScaledCoordinate(4*(i+1)), myWorld.getScaledCoordinate(4*(i+1)+1), myWorld.getScaledCoordinate(4*(i+1)+2));
		}
		glEnd();

		glFlush();

		// Update step
		// if (currentTime <= 1000)
		// {
		// 	myWorld.updateTimeStep();
		// }
		// else
		// {
		// 	myWorld.updateTimeStep_data();
		// }
        myWorld.updateTimeStep();
//         if (int(myWorld.getCurrentTime()/1e-4)%10==0)
//		 {
//		 	myWorld.CoutDataC(outfile); // write data to file
//	     }
		// myWorld.CoutDataC(outfile);
		// currentTime  = currentTime + 1e-3;
    myWorld.CoutData(outfile);
    myWorld.CoutDataC(nodefile);
  }
	exit(1);
}

int main(int argc,char *argv[])
{
	setInput inputData;
	inputData = setInput();
	inputData.LoadOptions(argv[1]);
	inputData.LoadOptions(argc,argv);
	//read input parameters from txt file and cmd

	myWorld = world(inputData);
  myWorld.OpenFile(outfile, "force");
  // myWorld.OpenFile(nodefile, "node");
	myWorld.setRodStepper();


	bool render = myWorld.isRender();

	if (render) // if OpenGL visualization is on
	{
		NPTS = myWorld.numPoints();

		glutInit(&argc,argv);
		glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB);
		glutInitWindowSize (1000, 1000);
		glutInitWindowPosition (100, 100);
		glutCreateWindow ("simDER");
		initGL();
		glutKeyboardFunc(Key);
		glutDisplayFunc(display);
		glutMainLoop();
	}
	else
	{
		while ( myWorld.simulationRunning() > 0)
		{
			myWorld.updateTimeStep(); // update time step
      myWorld.CoutData(outfile);
      // myWorld.CoutDataC(nodefile);
		}
	}

	// Close (if necessary) the data file
	myWorld.CloseFile(outfile);

	return 0;
}
