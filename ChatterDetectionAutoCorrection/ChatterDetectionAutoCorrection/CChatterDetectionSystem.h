#ifndef __CChatterDetectionSystem_
#define __CChatterDetectionSystem_

#include "CKalmanFilter.h"
#include "CBandpassFilters.h"
#include "CNonlinearEnergyOperator.h"
#include "CMeanFilter.h"
#include "CChatterDetection.h"

/// <summary>
/// Stucture consists of chatter frequency[rad/s], chatter amplitude, chatter energy and bandnumber for each band
/// The purpose of this structure is select chatter frequency by sorting
/// </summary>
struct SChatterOutput
{
	double Freq;//[rad/s]
	double Amp;
	double ChatterEnergy;
	int BandNumber;
};

/// <summary>
/// Class of a chatter detection system which includes all components
/// </summary>
class CChatterDetectionSystem
{
public:
	CKalmanFilter* KalmanFilter;
	CBandpassFilters* BandpassFilters;
	CNonlinearEnergyOperator* NEO;
	CMeanFilter* FreqMeanFilter;
	CMeanFilter* AmpMeanFilter;
	CMeanFilter* ChatterMeanFilter;
	CMeanFilter* PeriodicMeanFilter;
	CChatterDetection* ChatterDetection;

	CMatrix* ChatterFreq; // selected chattered frequency [rad/s]
	CMatrix* PrevChatterFreq;
	CMatrix* ChatterFreqVariation;
	double ChatterEnergyThreshold; // for selecting chatter frequency
	double SpindleSpeed; // current spindle speed
	int SpindleRateSet;

private:
	int N;
	int NumberOfFlute;
	double SamplingPeriod;
	double Lamda;
	double R;
	CMatrix* ChatterEnergy;
	double ThresholdEng; // Threshold chatter energy to compare with for chatter freq selection
	double EnergyThreshold; double EnergyRatioLimit; double IntegrationFactor;
	SChatterOutput* ChatterOutput;

public:
	CChatterDetectionSystem(int N, int numOfFlute, double spindleSpeed, double samplingPeriod, double lamda, double R, int numOfBand, double energyThreshold, double energyRatioLimit, double integrationFactor, double chatterEnergyThreshold, int ndMean, int delay);
	~CChatterDetectionSystem();
	void Run(double measuremnt);
	void SelectChatterFreq();
	double CalculateNewSpindleSpeed(double dominantChatterFreq);
	void UpdateSystem(double newSpindleSpeed);
	double FindDominantFreq(void);
	double CalculateChatterEnergy(CMatrix* freqIn, CMatrix* ampIn);
	double CalculatePeriodicEnergy(CMatrix* amplitude);
};

#endif