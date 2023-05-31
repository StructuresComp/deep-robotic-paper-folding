#ifndef TIMESTEPPER_H
#define TIMESTEPPER_H

#include "elasticRod.h"

extern "C" void dgbsv_( int* n, int* kl, int* ku, int* nrhs, double* ab, int* ldab, int* ipiv, double* b, int* ldb, int* info );

class timeStepper
{
public:
	timeStepper(elasticRod &m_rod);
	~timeStepper();
	double* getForce();
	double* getJacobian();
	void setZero();
	void addForce(int ind, double p);
	void addJacobian(int ind1, int ind2, double p);
	void integrator();
	void update();

	void pardisoSolver();

	VectorXd force;

	void monitor();

private:
	elasticRod *rod;
	int kl, ku, freeDOF;

	double *totalForce;
	double *jacobian;

	// utility variables
	int mappedInd, mappedInd1, mappedInd2;
	int row, col, offset;
	int NUMROWS;
	int jacobianLen;
	int nrhs;

	int *ipiv;
  int info;
  int ldb;

  MatrixXd Jacobian;
  VectorXd Force;
};

#endif
