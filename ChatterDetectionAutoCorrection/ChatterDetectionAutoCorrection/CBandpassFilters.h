#ifndef __CBandpassFilters_
#define __CBandpassFilters_

#include "CMatrix.h"

/// <summary>
/// Class of bandpass filter bank. Refer to Bandpass Filter Bank Simulink block for detailed information on properties and algorithm 
/// </summary>
class CBandpassFilters
{
// m: number of band
// N: order of filter
private:
	CMatrix* E; //[m x 2N+1]
	CMatrix* EDelayed; //[m x 2N]
	CMatrix* EDelayedSingle; //Single row of EDelayed:[2N x 1]
	CMatrix* ASingle; //denominator of a single bandpass filter:[1 x 2N]
	CMatrix* BSingle; //numerator of a single bandpass filter:[1 x 2N+1]
	CMatrix* ESingleTrans; // [2N+1 x 1]
	double SamplingPeriod; // in [s]
	int Order; // Order of filter. Order is 4 in this project

public:
	CMatrix* Numerator; //[m x 2N+1]
	CMatrix* Denominator; //[m x 2N]
	CMatrix* BandpassOutputs; //[m x 1]
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
};

#endif
