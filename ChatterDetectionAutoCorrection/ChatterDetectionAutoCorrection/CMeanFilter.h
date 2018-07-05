#ifndef __CMeanFilter_
#define __CMeanFilter_

#include "CMatrix.h"
#include "CEnergyMeanFilter.h"
//#include <cmath>
//#define PI acos(-1.0)

struct MeanFilterOutput
{
	double Freq;
	double Amp;
	double ChatterEnergy;
	int BandNumber;
};

class CMeanFilter
{
public:
	CMatrix* FilteredFreq;
	CMatrix* FilteredAmp;
	double ndMean; //Must use double or explicitly cast to double in calculation to avoid converting to int
	MeanFilterOutput* Output;
	//CEnergyMeanFilter* FreqFilter;
	//CEnergyMeanFilter* AmpFilter;

private:
	CMatrix* PrevAveFreq;
	CMatrix* PrevAveAmp;
	double Counter; // explicit double or cast to double to avoid being converted to int in RunMeanFilter()
	int NumberOfInput;
	double IntegrationFactor;

public:
	CMeanFilter(int numOfInput, double integrationFactor, int _ndMean);
	~CMeanFilter();
	void RunMeanFilter(CMatrix* freqInput, CMatrix* ampInput);
};

#endif
