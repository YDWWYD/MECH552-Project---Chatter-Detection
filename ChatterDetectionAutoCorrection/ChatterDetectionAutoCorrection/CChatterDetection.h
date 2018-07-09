#ifndef __CChatterDetection_
#define __CChatterDetection_

#include "CMatrix.h"
/// <summary>
/// ChatterDetection class which contains energy information and detection results
/// </summary>
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
	CMatrix* DelayedPeriodicEnergy;

public:
	CChatterDetection(double ndMean, double energyThreshold, double energyRatioLimit);
	~CChatterDetection();
	void RunChatterDetection();
};

#endif
