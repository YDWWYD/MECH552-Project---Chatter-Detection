#include "CMeanFilter.h"

CMeanFilter::CMeanFilter(int numOfInput, double integrationFactor, int _ndMean) 
{
	Counter = 0;
	//ndMean = round(1 / (spindleSpeed / 2 / PI) / samplingPeriod);
	ndMean = (double)_ndMean;

	FilteredFreq = new CMatrix(numOfInput, 1);
	FilteredAmp = new CMatrix(numOfInput, 1);
	PrevAveAmp = new CMatrix(numOfInput, 1);
	PrevAveFreq = new CMatrix(numOfInput, 1);
	
	Output = new MeanFilterOutput[numOfInput];
	NumberOfInput = numOfInput;
	IntegrationFactor = integrationFactor;

	//FreqFilter = new CEnergyMeanFilter(numOfInput, ndMean);
	//AmpFilter = new CEnergyMeanFilter(numOfInput, ndMean);
}

CMeanFilter::~CMeanFilter(void)
{
	delete FilteredFreq;
	delete FilteredAmp;
	delete PrevAveFreq;
	delete PrevAveAmp;
	delete Output;
}

void CMeanFilter::RunMeanFilter(CMatrix* freqIn, CMatrix* ampIn)
{
	//FilteredFreq = FreqFilter->RunMeanFilter(freqIn);
	//FilteredAmp = AmpFilter->RunMeanFilter(ampIn);
	for (int i = 0; i < NumberOfInput; i++)
	{
		Output[i].Freq = FilteredFreq->Content[i];
		Output[i].Amp = FilteredAmp->Content[i];
		Output[i].BandNumber = i + 1;
		double currentFreq = Output[i].Freq;
		double currentAmp = Output[i].Amp;
		if (currentFreq != 0)
			Output[i].ChatterEnergy = pow(currentAmp, 2)*pow(currentFreq, 2 * IntegrationFactor);
		else
			Output[i].ChatterEnergy = 0;
	}

	//if (Counter < ndMean)
	//{
	//	if (Counter != 0)
	//	{
	//		for (int i = 0; i < NumberOfInput; i++)
	//		{
	//			FilteredFreq->Content[i] = (Counter - 1) / (Counter)*PrevAveFreq->Content[i] + 1 / (Counter)*freqIn->Content[i];
	//			FilteredAmp->Content[i] = (Counter - 1) / (Counter)*PrevAveAmp->Content[i] + 1 / (Counter)*ampIn->Content[i];

	//			Output[i].Freq = FilteredFreq->Content[i];
	//			Output[i].Amp = FilteredAmp->Content[i];
	//			Output[i].BandNumber = i + 1;
	//			double currentFreq = Output[i].Freq;
	//			double currentAmp = Output[i].Amp;
	//			if (currentFreq != 0)
	//				Output[i].ChatterEnergy = pow(currentAmp, 2)*pow(currentFreq, 2 * IntegrationFactor);
	//			else
	//				Output[i].ChatterEnergy = 0;
	//		}
	//	}
	//}

	//else
	//{
	//	for (int i = 0; i < NumberOfInput; i++)
	//	{
	//		FilteredFreq->Content[i] = (ndMean - 1) / ndMean*PrevAveFreq->Content[i] + 1 / ndMean*freqIn->Content[i]; // In [rad/s]
	//		FilteredAmp->Content[i] = (ndMean - 1) / ndMean*PrevAveAmp->Content[i] + 1 / ndMean*ampIn->Content[i];

	//		Output[i].Freq = FilteredFreq->Content[i];
	//		Output[i].Amp = FilteredAmp->Content[i];
	//		Output[i].BandNumber = i + 1;
	//		double currentFreq = Output[i].Freq;
	//		double currentAmp = Output[i].Amp;
	//		if (currentFreq != 0)
	//			Output[i].ChatterEnergy = pow(currentAmp, 2)*pow(currentFreq, 2 * IntegrationFactor);
	//		else
	//			Output[i].ChatterEnergy = 0;
	//	}
	//}

	// //Update previous average frequency and amplitude
	//for (int i = 0; i < NumberOfInput; i++)
	//{
	//	PrevAveFreq->Content[i] = FilteredFreq->Content[i]; // in [rad/s]
	//	PrevAveAmp->Content[i] = FilteredAmp->Content[i];
	//}

	////Update counter;
	//Counter++;
}