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
	double getTotalTime();

	bool isRender();

	// file output
	void OpenFile(ofstream &outfile, string name);
	void CloseFile(ofstream &outfile);
	void CoutData(ofstream &outfile);
	void CoutDataC(ofstream &outfile);

	void updateTimeStep_data();
	bool savetemp;
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

	double pulltime;

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
	double hL;

	string searchFile;
	vector<Vector3d> searchP;
	double Lgb;


	void rodGeometry();
	void rodBoundaryCondition();

	void updateBoundary();

	void updateCons();

	void newtonMethod(bool &solved);

	void calculateForce();
	bool move2goal(Vector3d temp);


};

#endif
