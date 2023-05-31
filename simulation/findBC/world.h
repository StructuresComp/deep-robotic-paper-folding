#ifndef WORLD_H
#define WORLD_H

#include "environment/eigenIncludes.h"

// include elastic rod class
#include "environment/elasticRod.h"

// include force classes
#include "environment/elasticStretchingForce.h"
#include "environment/elasticBendingForce.h"
#include "environment/elasticTwistingForce.h"
#include "environment/externalGravityForce.h"
#include "environment/inertialForce.h"

// include external force
#include "environment/dampingForce.h"

// include time stepper
#include "environment/timeStepper.h"

// include input file and option
#include "setInput.h"

//include collision checker
// #include "collision.h"

class world
{
public:
	world();
	world(setInput &m_inputData);
	~world();
	void setRodStepper();
	void updateTimeStep();
	int simulationRunning();
	int numPoints();
	double getScaledCoordinate(int i);
	double getCurrentTime();

	bool isRender();

	// file output
	void OpenFile(ofstream &outfile);
	void CloseFile(ofstream &outfile);
	void CoutData(ofstream &outfile);

private:

	// Physical parameters
	double RodLength;
	double rodRadius;
	int numVertices;
	double youngM;
	double Poisson;
	double shearM;
	double deltaTime;
	double totalTime;
	double density;
	Vector3d gVector;
	double viscosity;

	double tol, stol;
	int maxIter; // maximum number of iterations
	double characteristicForce;
	double forceTol;

	// Geometry
	MatrixXd vertices;
	VectorXd theta;

	// Rod
	elasticRod *rod;

	// set up the time stepper
	timeStepper *stepper;
	double *totalForce;
	double currentTime;

	// declare the forces
	elasticStretchingForce *m_stretchForce;
	elasticBendingForce *m_bendingForce;
	elasticTwistingForce *m_twistingForce;
	inertialForce *m_inertialForce;
	externalGravityForce *m_gravityForce;
	dampingForce *m_dampingForce;

	int iter;

	bool render; // should the OpenGL rendering be included?
	bool saveData; // should data be written to a file?

	int searchStep;
	double goal0;
	double Lgb;

	double hL;

	bool move2goal(Vector3d temp);
	void updateBoundary();

	void rodGeometry();
	void rodBoundaryCondition();

	void newtonMethod(bool &solved);

	void move2NextP();
	void findUp();
	void findBottom();
	void setUpBound();
	void setUpMiddle();

	Vector3d BC1;
	Vector3d BC2;
	Vector3d Middle;
  double middle;
  Vector3d Up;
  Vector3d Bottom;
	bool searchUp;
	double dgoal;







};

#endif
