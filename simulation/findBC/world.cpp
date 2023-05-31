#include "world.h"
#include <sstream>
#include <iomanip>


// double angleBetweenVectors(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2) {
//     double cosTheta = vec1.dot(vec2) / (vec1.norm() * vec2.norm());
//     return std::acos(cosTheta);
// }


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
	dgoal = M_PI/180;
}

world::~world()
{
	;
}

bool world::isRender()
{
	return render;
}

void world::OpenFile(ofstream &outfile)
{
	if (saveData==false) return;

	int systemRet = system("mkdir bcfiles"); //make the directory
	if(systemRet == -1)
	{
		cout << "Error in creating directory\n";
	}

	time_t current_time = time(0);

	// Open an input file named after the current time
	ostringstream name;
  ostringstream num;

	num << std::setprecision(4);
	// num << std::fixed;
	num << (hL/Lgb);
  name << "bcfiles/simDER" <<num.str()<<".txt";
	outfile.open(name.str().c_str());
	outfile.precision(10);
	saveData = false;
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

	Up = Up/Lgb;
	Bottom = Bottom/Lgb;

	outfile << goal0<<" "<<Up(0)<<" "<<Up(1)<<" "<<Up(2)<<
		        " "<< Bottom(0)<<" "<< Bottom(1)<<" "<<Bottom(2)<<endl;
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

	goal0 = M_PI/18;
	searchStep = 1;

	currentTime = 0.0;
}



void world::rodGeometry()
{
	//vertices number for hanging lengths
	// 0.1 meter dense discretization, 0.9 meter appropriate discretization
	double dl = 1e-3;
	numVertices = hL/dl;
	dl = hL/numVertices;
	numVertices = numVertices + 2;

	//vertices of rod
	vertices = MatrixXd(numVertices, 3);
	//here rod length is hanging length;
	//edgenumber

	for (int i = 0; i<numVertices; i++)
	{
		vertices(i, 0) = -dl+ i*dl;
		vertices(i, 1) = 0;
		vertices(i, 2) = 0;
	}

	RodLength = hL + dl;
	theta = VectorXd::Zero(numVertices-1);

}



void world::rodBoundaryCondition()
{

	rod->setVertexBoundaryCondition(rod->getVertex(0),0);
  rod->setThetaBoundaryCondition(rod->getTheta(0),0);
  rod->setVertexBoundaryCondition(rod->getVertex(1),1);

  rod->setVertexBoundaryCondition(rod->getVertex(numVertices-1),numVertices-1);

	for (int i = 0; i < numVertices-1; i++)
	{
		rod->setDofBoudaryCondition(4*i+1);
	}
}

void world::updateBoundary()
{
	saveData = false;
	if (goal0 <= M_PI && currentTime > 10)
	{
		switch (searchStep){
			case 1:{
				move2NextP();
				break;
			}
			case 2:{
				// find upper
				setUpMiddle();
				break;
			}
			case 3:{
				// find upper
				setUpBound();
				break;
			}
			case 4:{
				// find upper
				findUp();
				break;
			}
			case 5:{
				findBottom();
				break;
			}
		}
	}

	if (goal0 > M_PI)
	{
		exit(0);
	}
}

void world::move2NextP()
{
	bool flag = move2goal(Vector3d(hL*cos(goal0),
	0, hL*sin(goal0)));
	if (flag)
	{
		//check x local
		searchUp = false;
		for (int i = 2; i < numVertices; i++)
		{
			Vector3d xLocal = rod->getVertex(i);
			if (xLocal(2) < 0)
			{
				searchUp = true;
				break;
			}
		}
		Middle = rod->getVertex(numVertices-1);
		searchStep = 2;
	}
}

void world::setUpMiddle()
{
	if (searchUp)
	{
		Vector3d dir = rod->getVertex(numVertices-1);
		dir = dir/dir.norm();
		bool flag = move2goal(Middle + 1e-5 * hL * dir);
		if (flag)
		{
			Middle = Middle + 1e-5 * hL * dir;
			// check status;
      bool flag = false;
			for (int i = 2; i < numVertices-1; i++)
			{
				Vector3d xLocal = rod->getVertex(i);
				if (xLocal(2) < 0)
				{
					flag = true;
					break;
				}
			}
			if (!flag)
			{
				BC1 = Middle;
				searchStep = 3;
			}
		}
	}
	else
	{
		Vector3d dir = rod->getVertex(numVertices-1);
		dir = dir/dir.norm();
		bool flag = move2goal(Middle);
		if (flag)
		{
			// check status;
			Vector3d Fs1 = stepper->force.segment(4, 3);

			bool flag = false;
			for (int i = 2; i < numVertices; i++)
			{
				Vector3d xLocal = rod->getVertex(i);
				if (xLocal(2) < 0)
				{
					flag = true;
					break;
				}
			}

			if (Fs1(2) >= 0 || flag)
			{
				BC1 = Middle;
				searchStep = 3;

				if (flag && goal0 > M_PI/2)
				{
					if (dgoal >= M_PI/1800)
					{
						goal0 = goal0 - dgoal;
						dgoal = 0.1 * dgoal;
						goal0 = goal0 + dgoal;
						searchStep = 1;
					}
					else
					{
						cout <<"Completed one task"<<endl;
						exit(0);
					}
				}

				if (Fs1(2) < 0)
				{
					if (dgoal >= M_PI/1800)
					{
						goal0 = goal0 - dgoal;
						dgoal = 0.1 * dgoal;
						goal0 = goal0 + dgoal;
						searchStep = 1;
					}
					else
					{
						exit(0);
					}
				}
			}
			else
			{
				Middle = Middle - 1e-3 * hL * dir;
			}
		}
	}
}

void world::setUpBound()
{
	// x > 0, F < 0 is the Up bound
	bool flag = move2goal(BC1);
	if (flag)
	{
		// check status;
		Vector3d Fs1 = stepper->force.segment(4, 3);

		if (Fs1(2) >= 0)
		{
			BC1 = 1.1*BC1;
		}
		else
		{
			BC2 = Middle;
			searchStep = 4;
		}
	}
}


void world::findUp()
{
	Vector3d BC = (BC1 + BC2)/2.0;

	bool flag = move2goal(BC);
	if (flag)
	{
		// check status
		double tol = 0;
		Vector3d Fs1 = stepper->force.segment(4, 3);

		if (Fs1(2) < 0) // down
		{
			BC1 = BC;
		}
		else // upper
		{
			BC2 = BC;
		}

		if ((BC1-BC2).norm() < 1e-5)
		{
			Up = BC1;
			BC1 = Up;
			BC2 = Vector3d(0, 0, 0);
			searchStep = 5;
		}
	}
}

// searching along the ray
void world::findBottom()
{
	Vector3d BC = (BC1 + BC2)/2.0;

	bool flag = move2goal(BC);
	if (flag)
	{
		if ((BC1-BC2).norm() < 1e-5)
		{
			Bottom = BC2;
			goal0 = goal0 + dgoal;
			saveData = true;
			searchStep = 1;
		}
		// all the cases unsatisfied the requirement of the available region x > 0 && Fs > 0
		// check location
		bool contactflag = true;
		for (int i = 2; i < numVertices; i++)
		{
			Vector3d xLocal = rod->getVertex(i);
			if (xLocal(2) < 0) // upper
			{
				contactflag = false;
				break;
			}
		}
		// check forces
		Vector3d Fs1 = stepper->force.segment(4, 3);
		Vector3d Fs2 = stepper->force.segment(0, 3);
		if (Fs1(2) < 0 || Fs2(2) < 0) contactflag = false;

		if (contactflag)
		{
			BC1 = BC;
		}
		else
		{
			BC2 = BC;
		}
	}
}

bool world::move2goal(Vector3d goal)
{
	Vector3d temp = rod->getVertex(numVertices-1);
  temp = goal - temp;
	if (temp.norm() <= 0.05 * RodLength * deltaTime)
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
		rod->setVertexBoundaryCondition(rod->getVertex(numVertices-1)+0.05 *RodLength*temp*deltaTime
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

	if (render) cout << "time: " << currentTime <<" searchStep: "<<searchStep<<endl;

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
			// cout << "Error. Could not converge. Exiting.\n";
			break;
		}
	}
}


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
