#ifndef __CNonlinearEnergyOperator_
#define __CNonlinearEnergyOperator_

#include "CMatrix.h"
#include "CBandpassFilters.h"

class CNonlinearEnergyOperator
{
private:
	CMatrix* D; //lag matrix
	CMatrix* SDelayed;
	CMatrix* phi_y_kD;
	CMatrix* phi_y_k2D;
	CMatrix* phi_s_k2D;

	CMatrix* s_k;
	CMatrix* s_kD;
	CMatrix* s_k2D;
	CMatrix* s_k3D;
	CMatrix* s_k4D;

	CMatrix* y_k;
	CMatrix* y_kD;
	CMatrix* y_k2D;
	CMatrix* y_k3D;

	CMatrix* PrevFreq;
	CMatrix* PrevAmp;

	double T; //SamplingPeriod
	//int MaxDelay;
public:
	CMatrix* Freq;
	CMatrix* Amp;

public:
	CNonlinearEnergyOperator(double samplingPeriod, double TPE, CBandpassFilters& bandpassFilters);
	~CNonlinearEnergyOperator();
	void RunNEO(CMatrix* input);

private:
	CMatrix CalculateLag(double samplingPeriod, double TPE, CBandpassFilters& bandpassFilters);
};

#endif
