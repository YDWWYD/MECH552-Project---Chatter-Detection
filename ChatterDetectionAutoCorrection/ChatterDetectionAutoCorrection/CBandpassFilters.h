#ifndef __CBandpassFilters_
#define __CBandpassFilters_

#include "CMatrix.h"
//#include <cmath>
//#define PI acos(-1.0)

class CBandpassFilters
{
private:
	CMatrix* E;
	CMatrix* EDelayed;
	CMatrix* EDelayedSingle;
	CMatrix* ASingle;
	CMatrix* BSingle;
	CMatrix* ESingleTrans;
	double SamplingPeriod;

public:
	CMatrix* Numerator;
	CMatrix* Denominator;
	CMatrix* BandpassOutputs;
	int Order;
	int NumberOfFilters;
	int StartBand;

public:
	//CBandpassFilters(double StartFreq, double StopFreq, double TPE, double dt, double StopBandMag, double RippleMag, double Ts);
	CBandpassFilters(int numOfBand, double TPE, double Ts);
	~CBandpassFilters();
	double CalculateInput(double measurement, double estimation);
	void RunBandpassFilters(double input);
	void CalculateCoefficients(double TPE);
	void UpdateBandpass(double newSpindleSpeed);
	//void DenominatorCoeffi(double Q, double w0, double a1, double a2, double Ts);
};

#endif
