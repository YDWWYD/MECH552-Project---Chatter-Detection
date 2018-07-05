#ifndef __CChatterDetection_
#define __CChatterDetection_

#include "CMatrix.h"

class CChatterDetection
{
public:
	int ChatterDetected;
	double PeriodicEnergy;
	double ChatterEnergy;
	double EnergyRatio;
	double EnergyThreshold;
	double EnergyRatioLimit;

private:
	int PrevChatterDetected;
	int DetectionDelay;
	double IntegrationFactor;
	CMatrix* DelayedPeriodicEnergy;

public:
	CChatterDetection(double ndMean, double energyThreshold, double energyRatioLimit);
	~CChatterDetection();
	void RunChatterDetection(/*CMatrix* periodicAmplitude, double spindleSpeed, MeanFilterOutput* meanFilterOutputs, int numOfBand*/);
};

#endif
