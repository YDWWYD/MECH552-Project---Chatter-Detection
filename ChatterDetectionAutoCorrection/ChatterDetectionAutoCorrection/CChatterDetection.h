#ifndef __CChatterDetection_
#define __CChatterDetection_

#include "CMatrix.h"
#include <cmath>
#include "CMeanFilter.h"
#include "CEnergyMeanFilter.h"
#define PI acos(-1.0)

class CChatterDetection
{
public:
	int ChatterDetected;
	double PeriodicEnergy;
	double ChatterEnergy;
	double EnergyRatio;
	double EnergyThreshold;
	double EnergyRatioLimit;
	CMatrix* DelayedPeriodicEnergy;

private:
	int PrevChatterDetected;
	int DetectionDelay;
	double IntegrationFactor;


public:
	CChatterDetection(double ndMean, double energyThreshold, double energyRatioLimit);
	~CChatterDetection();
	void RunChatterDetection(/*CMatrix* periodicAmplitude, double spindleSpeed, MeanFilterOutput* meanFilterOutputs, int numOfBand*/);
};
//double CalculateChatterEnergy(CMatrix* amplitude, CMatrix* frequency);
#endif
