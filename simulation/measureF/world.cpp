#include "world.h"
#include <sstream>
#include <iomanip>

world::world()
{
	;
}

world::world(setInput &m_inputData)
{
	render = m_inputData.GetBoolOpt("render");				// boolean
	saveData = m_inputData.GetBoolOpt("saveData");			// boolean

	// Physical parameters
	hL = m_inputData.GetScalarOpt("hangLength");
	// dl_b = m_inputData.GetScalarOpt("dl_b");

	maxIter = m_inputData.GetIntOpt("maxIter");             // maximum number of iterations
	deltaTime = m_inputData.GetScalarOpt("deltaTime");      // seconds
	tol = m_inputData.GetScalarOpt("tol");                  // small number like 10e-7
	stol = m_inputData.GetScalarOpt("stol");				// small number, e.g. 0.1%
	viscosity = m_inputData.GetScalarOpt("viscosity");      // viscosity in Pa-s
	searchFile = m_inputData.GetStringOpt("filename");

	// default parameters
	gVector = Vector3d(0, 0, -100);
	density = 100;
	youngM = 1.8e6;
	rodRadius = 1.6e-3;
	Poisson = 0.5;
	Lgb = youngM * pow(rodRadius,2)/(8*density * 100);
	Lgb = pow(Lgb, 1.0/3);
	totalTime = 100000;

	shearM = youngM/(2.0*(1.0+Poisson));					// shear modulus
	// decide the real length
	hL = hL * Lgb;

	shearM = youngM/(2.0*(1.0+Poisson));					// shear modulus
}

world::~world()
{
	;
}

bool world::isRender()
{
	return render;
}

void world::OpenFile(ofstream &outfile, string filename)
{
	if (saveData==false) return;

	int systemRet = system("mkdir datafiles"); //make the directory
	if(systemRet == -1)
	{
		cout << "Error in creating directory\n";
	}

	time_t current_time = time(0);

	// Open an input file named after the current time
	ostringstream name;

	name << std::setprecision(4);
	name << "datafiles/"<< filename;
	name << "_hangLength_"<< (hL/Lgb);
	name <<".txt";

//    name << "datafiles/simDER" <<".txt";
	outfile.open(name.str().c_str());
	outfile.precision(10);
	saveData = false;
	// outfile << "# time (sec) x [meter] y [meter] z [meter]\n";
}

void world::CloseFile(ofstream &outfile)
{
	if (saveData==false)
		return;

	outfile.close();
}

void world::CoutData(ofstream &outfile)
{
	if (saveData==false)
		return;

	double zoffset = 0;
	if (savetemp)
	{
		for (int i = 2; i < numVertices; i++)
		{
			Vector3d xLocal = rod->getVertex(i);
			if (xLocal(2) < 0)
			{
				zoffset = zoffset + xLocal(2);
			}
		}
	}

	Vector3d temp = rod->getVertex(numVertices-1);
	Vector3d temp1 = rod->getVertex(numVertices-2);
	Vector3d Fs = stepper->force.segment(0, 3);
	Vector3d Fs1 = stepper->force.segment(4, 3);

	temp = temp/Lgb;
	temp1 = temp1/Lgb;
	temp = temp + Vector3d(hL/Lgb, 0, 0);
	temp1 = temp1 + Vector3d(hL/Lgb, 0, 0);
	Fs = Fs * rodRadius * rodRadius / rod->EI;
	Fs1 = Fs1 * rodRadius * rodRadius / rod->EI;



	outfile <<temp(0)<<" "<<temp(1)<<" "<<temp(2)<<" "<<temp1(0)<<" "<<temp1(1)<<" "<<temp1(2)<<
		" "<<Fs(0)<<" "<<Fs(1)<<" "<<Fs(2)<<" " <<Fs1(0)<<" "<<Fs1(1)<<" "<<Fs1(2)<<" "<<zoffset<<endl;
}


void world::CoutDataC(ofstream &outfile)
{
	if (savetemp==false)
		return;
	for (int i = 0; i < rod->nv; i++)
	{
		if (i<rod->ne)
		{
			outfile  << rod->x(4*i) << " " <<rod->
		x(4*i+1) << " " << rod->x(4*i+2) <<" "<< rod->x(4*i+3)<<endl;
		}
		else
		{
			outfile<< rod->x(4*i) << " " <<rod->
		x(4*i+1) << " " << rod->x(4*i+2) <<" "<< 0<<endl;
		}
	}
}

void world::setRodStepper()
{
	// Set up geometry
	rodGeometry();

	// Create the rod
	rod = new elasticRod(vertices, vertices, density, rodRadius, deltaTime,
		youngM, shearM, RodLength, theta);

	// Find out the tolerance, e.g. how small is enough?
	characteristicForce = M_PI * pow(rodRadius ,4)/4.0 * youngM / pow(RodLength, 2);
	forceTol = tol * characteristicForce;

	// Set up boundary condition
	rodBoundaryCondition();
	// setup the rod so that all the relevant variables are populated
	rod->setup();
	// End of rod setup

	// set up the time stepper
	stepper = new timeStepper(*rod);
	totalForce = stepper->getForce();

	// declare the forces
	m_stretchForce = new elasticStretchingForce(*rod, *stepper);
	m_bendingForce = new elasticBendingForce(*rod, *stepper);
	m_twistingForce = new elasticTwistingForce(*rod, *stepper);
	m_inertialForce = new inertialForce(*rod, *stepper);
	m_gravityForce = new externalGravityForce(*rod, *stepper, gVector);
	m_dampingForce = new dampingForce(*rod, *stepper, viscosity);

	// Allocate every thing to prepare for the first iteration
	rod->updateTimeStep();

	saveData = false;
	searchStep = 0;

	currentTime = 0.0;
}

void world::rodGeometry()
{

	//read data for rodGeometry
	double dl = 1e-3;
	numVertices = hL/dl;
	dl = hL/numVertices;

	numVertices = numVertices + 2;

	//vertices of rod
	vertices = MatrixXd(numVertices, 3);

	for (int i = 0; i<numVertices; i++)
	{
		vertices(i, 0) = -dl+ i*dl;
		vertices(i, 1) = 0;
		vertices(i, 2) = 0;
	}
	RodLength = hL + dl;
	theta = VectorXd::Zero(numVertices-1);


	// read file
	ifstream infile;

	infile.open(searchFile.c_str());
	if (!infile.is_open()) {
			cout << "Unable to open file to read file";
	}

	double a, b;
	searchP.clear();
	while (infile >> a >> b) {
		searchP.push_back(Vector3d(a, 0, b) * Lgb);
	}
	infile.close();
}



void world::rodBoundaryCondition()
{

	rod->setVertexBoundaryCondition(rod->getVertex(0),0);
  rod->setThetaBoundaryCondition(rod->getTheta(0),0);
  rod->setVertexBoundaryCondition(rod->getVertex(1),1);

  rod->setVertexBoundaryCondition(rod->getVertex(numVertices-1) ,numVertices-1);


	for (int i = 0; i < numVertices-1; i++)
	{
		rod->setThetaBoundaryCondition(rod->getTheta(i),i);
		rod->setDofBoudaryCondition(4*i+1);
	}
  int i = numVertices-1;
	rod->setDofBoudaryCondition(4*i+1);
}



void world::updateBoundary()
{
	// Steps
	// 1.move to ori
  // 2.move along vertical direction
	// 3.back to ori
	// saveData = false;
	saveData = false;
	Vector3d goal = searchP[searchStep];
	bool flag = move2goal(goal);

	if (flag)
	{
		bool contact = false;
		for (int i = 2; i < numVertices; i++)
		{
			Vector3d xLocal = rod->getVertex(i);
			if (xLocal(2) < 0)
			{
				contact = true;
				break;
			}
		}
		saveData = true;
		if (contact)
		{
			saveData = false;
		}
		searchStep++;
	}

	if (searchStep >= searchP.size())
	{
		currentTime = totalTime;
	}
}


bool world::move2goal(Vector3d goal)
{
	Vector3d temp = rod->getVertex(numVertices-1);
  temp = goal - temp;
	if (temp.norm() <= 0.01 * RodLength * deltaTime)
	{
		rod->setVertexBoundaryCondition(goal , numVertices-1);

		if (rod->u.norm()<1e-5)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		temp = temp/temp.norm();
		rod->setVertexBoundaryCondition(rod->getVertex(numVertices-1)+0.01 *RodLength*temp*deltaTime
			 , numVertices-1);
	}
	return false;
}

void world::updateTimeStep()
{
	bool solved = false;

	if (currentTime > 10)
	{
		updateBoundary();
	}

	while (!solved)
	{
		rod->updateGuess();
		newtonMethod(solved); //get configuration when no collisoins
		if (!solved)
		{
			deltaTime = 0.1 * deltaTime;
			rod->dt = deltaTime;
		}

		if (deltaTime < 1e-6)
		{
			break;
		}
	}

	rod->updateTimeStep();

	if (render) cout << "time: " << currentTime <<" searchStep: "<<searchStep<<" iter=" << iter <<" velocity: "<<rod->u.norm()<<endl;

	currentTime += deltaTime;

	if (deltaTime < 1e-2 && iter <= 2)
	{
		deltaTime = 10 * deltaTime;
		rod->dt = deltaTime;
	}

	if (solved == false)
	{
		currentTime = totalTime; // we are exiting
	}
}

void world::newtonMethod(bool &solved)
{
	double normf = forceTol * 10.0;
	double normf0 = 0;
	iter = 0;
	while (solved == false)
	{
		rod->prepareForIteration();

		stepper->setZero();

		// Compute the forces and the jacobians
		m_inertialForce->computeFi();
		m_inertialForce->computeJi();

		m_stretchForce->computeFs();
		m_stretchForce->computeJs();

		m_bendingForce->computeFb();
		m_bendingForce->computeJb();

		m_twistingForce->computeFt();
		m_twistingForce->computeJt();

		m_gravityForce->computeFg();
		m_gravityForce->computeJg();

		m_dampingForce->computeFd();
		m_dampingForce->computeJd();

		// Compute norm of the force equations.
		normf = 0;
		for (int i=0; i < rod->uncons; i++)
		{
			normf += totalForce[i] * totalForce[i];
		}
		normf = sqrt(normf);
		if (iter == 0)
		{
			normf0 = normf;
		}

		if (normf <= forceTol )
		{
			solved = true;
		}
		else if(iter > 0 && normf <= normf0 * stol)
		{
			solved = true;
		}

		if (solved == false)
		{
			stepper->integrator(); // Solve equations of motion
			rod->updateNewtonX(totalForce); // new q = old q + Delta q
			iter++;
		}

		if (iter > maxIter)
		{
			break;
		}
	}
}

// void world::calculateForce()
// {
// 	stepper->setZero();
// 	m_inertialForce->computeFi();
//
// 	inertiaF = Vector3d::Zero();
// 	dampF = Vector3d::Zero();
// 	evaluateF = Vector3d::Zero();
//
// 	for (int i = 0; i< rod->nv; i++)
// 	{
// 		inertiaF(0) = inertiaF(0) + stepper->force(4*i);
// 		inertiaF(1) = inertiaF(1) + stepper->force(4*i+1);
// 		inertiaF(2) = inertiaF(2) + stepper->force(4*i);
// 	}
//
// 	stepper->setZero();
// 	m_dampingForce ->computeFd();
// 	for (int i = 0; i< rod->nv; i++)
// 	{
// 		dampF(0) = dampF(0) + stepper->force(4*i);
// 		dampF(1) = dampF(1) + stepper->force(4*i+1);
// 		dampF(2) = dampF(2) + stepper->force(4*i);
// 	}
//
//
// 	stepper->setZero();
// 	m_gravityForce->computeFg();
//
// 	S_gravity = Vector3d::Zero();
//
// 	for (int i = 0; i < rod->nv; i++)
// 	{
// 		S_gravity(0) = S_gravity(0) - stepper->force(4*i);
// 		S_gravity(1) = S_gravity(1) - stepper->force(4*i+1);
// 		S_gravity(2) = S_gravity(2) - stepper->force(4*i+2);
// 	}
//
// 	stepper->setZero();
// 	m_inertialForce->computeFi();
// 	m_stretchForce->computeFs();
// 	m_bendingForce->computeFb();
// 	m_twistingForce->computeFt();
// 	m_gravityForce->computeFg();
// 	m_dampingForce->computeFd();
//
// 	evaluateF(0) = stepper->force(4);
// 	evaluateF(1) = stepper->force(5);
// 	evaluateF(2) = stepper->force(6);
//
// 	//caculate the force from substrate
// 	Fs(0) = stepper->force(0);
// 	Fs(1) = stepper->force(1);
// 	Fs(2) = stepper->force(2);
//
// 	Fs1(0) = stepper->force(4);
// 	Fs1(1) = stepper->force(5);
// 	Fs1(2) = stepper->force(6);
//
//
//
// 	//caculate the force from end effector
//
// 	Fe(0) = stepper->force(4*(numVertices-2)) + stepper->force(4*(numVertices-1));
// 	Fe(1) = stepper->force(4*(numVertices-2)+1) + stepper->force(4*(numVertices-1)+1);
// 	Fe(2) = stepper->force(4*(numVertices-2)+2) + stepper->force(4*(numVertices-1)+2);
//
//
//
//
// }

int world::simulationRunning()
{
	if (currentTime<totalTime)
		return 1;
	else
	{
		return -1;
	}
}

int world::numPoints()
{
	return rod->nv;
}

double world::getScaledCoordinate(int i)
{
	return rod->x[i] /RodLength ;
}

double world::getCurrentTime()
{
	return currentTime;
}

double world::getTotalTime()
{
	return totalTime;
}
