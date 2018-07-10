#ifndef __CNonlinearEnergyOperator_
#define __CNonlinearEnergyOperator_

#include "CMatrix.h"
#include "CBandpassFilters.h"

/// <summary>
/// Class of NEO(Nonlinear Energy Operator) to separate chatter frequency and amplitude for each individual band
/// Refer to NEO Simulink block for detailed algorithm
/// </summary>
class CNonlinearEnergyOperator
{
private:
	CMatrix* D; //lag matrix: [1 x m]
	CMatrix* SDelayed; //Delayed s: [m x 4*max(D)]
	CMatrix* phi_y_kD; // [m x 1]
	CMatrix* phi_y_k2D; // [m x 1]
	CMatrix* phi_s_k2D; // [m x 1]

	CMatrix* s_k; // [m x 1]
	CMatrix* s_kD; // [m x 1]
	CMatrix* s_k2D; // [m x 1]
	CMatrix* s_k3D; // [m x 1]
	CMatrix* s_k4D; // [m x 1]
	
	// difference operators
	CMatrix* y_k; // [m x 1]
	CMatrix* y_kD; // [m x 1]
	CMatrix* y_k2D; // [m x 1]
	CMatrix* y_k3D; // [m x 1]

	CMatrix* PrevFreq; // store previous frequency [rad/s]
	CMatrix* PrevAmp;

	double T; //SamplingPeriod

public:
	CMatrix* Freq; //Output frequency [rad/s]
	CMatrix* Amp; //Output amplitude

public:
	CNonlinearEnergyOperator(double samplingPeriod, double TPE, CBandpassFilters& bandpassFilters);
	~CNonlinearEnergyOperator();
	void RunNEO(CMatrix* input);

private:
	void CalculateLag(double TPE, CBandpassFilters& bandpassFilters);
};

#endif
