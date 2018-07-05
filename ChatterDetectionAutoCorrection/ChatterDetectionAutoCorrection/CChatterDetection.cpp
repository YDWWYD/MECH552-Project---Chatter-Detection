#include "CChatterDetection.h"

CChatterDetection::CChatterDetection(double ndMean, double energyThreshold, double energyRatioLimit)
{
	ChatterDetected = 0;
	//PeriodicEnergy = new double[1];
	PeriodicEnergy = 0;
	//ChatterEnergy = new double[1];
	ChatterEnergy = 0;

	EnergyRatio = 0;
	EnergyThreshold = energyThreshold;
	EnergyRatioLimit = energyRatioLimit;

	PrevChatterDetected = 0;
	DetectionDelay = (int)round((ndMean + 1) / 2);
	DelayedPeriodicEnergy = new CMatrix(DetectionDelay, 1);
}

CChatterDetection::~CChatterDetection(void)
{
	delete DelayedPeriodicEnergy;
	//delete PeriodicEnergy;
	//delete ChatterEnergy;
}

void CChatterDetection::RunChatterDetection(/*CMatrix* periodicAmplitude, double spindleSpeed, MeanFilterOutput* meanFilterOutputs, int numOfBand*/)
{
	EnergyRatio = ChatterEnergy / (ChatterEnergy + DelayedPeriodicEnergy->Content[0]);

	if (PrevChatterDetected == 0 && EnergyRatio > 0.5) // entry
		ChatterDetected = 0;
	else if (PrevChatterDetected == 1 && EnergyRatio > 0.1) // chatter continues
		ChatterDetected = 1;
	else if (EnergyRatio > EnergyRatioLimit && PeriodicEnergy > EnergyThreshold) // chatter just detected
		ChatterDetected = 1;
	else if (EnergyRatio < 0.1) //no chatter
		ChatterDetected = 0;
	else
		ChatterDetected = 0;

	// Update if chatter is detected by previous data
	PrevChatterDetected = ChatterDetected;

	// Update delayed periodic energy
	for (int i = 0; i < DetectionDelay - 1; i++)
	{
		DelayedPeriodicEnergy->Content[i] = DelayedPeriodicEnergy->Content[i + 1];
	}
	DelayedPeriodicEnergy->Content[DetectionDelay - 1] = PeriodicEnergy;
}