#ifndef __CChatterDetectionSystem_
#define __CChatterDetectionSystem_

#include "CKalmanFilter.h"
#include "CBandpassFilters.h"
#include "CNonlinearEnergyOperator.h"
#include "CMeanFilter.h"
#include "CChatterDetection.h"

struct SChatterOutput
{
	double Freq;
	double Amp;
	double ChatterEnergy;
	int BandNumber;
};

class CChatterDetectionSystem
{
public:
	CKalmanFilter* KalmanFilter;
	CBandpassFilters* BandpassFilters;
	CNonlinearEnergyOperator* NEO;
	CEnergyMeanFilter* FreqMeanFilter;
	CEnergyMeanFilter* AmpMeanFilter;

	CEnergyMeanFilter* ChatterMeanFilter;
	CEnergyMeanFilter* PeriodicMeanFilter;

	CChatterDetection* ChatterDetection;

	CMatrix* ChatterFreq;
	CMatrix* PrevChatterFreq;
	CMatrix* ChatterFreqVariation;
	double ChatterEnergyThreshold;
	double SpindleSpeed;
	int SpindleRateSet;

private:
	int N;
	int NumberOfFlute;
	double SamplingPeriod;
	double Lamda;
	double R;
	CMatrix* ChatterEnergy;
	double ThresholdEng;
	//double StartFreq; double StopFreq;  
	//double TPE; 
	//double Dt; 
	//double StopBandMag; double RippleMag;
	double EnergyThreshold; double EnergyRatioLimit; double IntegrationFactor;
	SChatterOutput* ChatterOutput;

public:
	CChatterDetectionSystem(int N, int numOfFlute, double spindleSpeed, double samplingPeriod, double lamda, double R, int numOfBand, double energyThreshold, double energyRatioLimit, double integrationFactor, double chatterEnergyThreshold, int ndMean, int delay);
	~CChatterDetectionSystem();
	void Run(double measuremnt);
	void CalculateChatterFreq(/*MeanFilterOutput* meanFilterOutputs, double totalChatterEnergy,*/ int numOfInput/*, double threshold*/);
	double CalculateNewSpindleSpeed(double dominantChatterFreq, double currentSpindleSpeed);
	void UpdateSystem(double newSpindleSpeed);
	double FindDominantFreq(void);
	double CalculateChatterEnergy(CMatrix* freqIn, CMatrix* ampIn);
	double CalculatePeriodicEnergy(CMatrix* amplitude);
};

#endif