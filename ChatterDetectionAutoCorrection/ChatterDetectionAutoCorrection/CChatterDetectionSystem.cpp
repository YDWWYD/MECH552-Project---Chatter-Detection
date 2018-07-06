#include "CChatterDetectionSystem.h"

bool sortByChatterEnergy(SChatterOutput& lhs, SChatterOutput& rhs);

CChatterDetectionSystem::CChatterDetectionSystem(int _N, int numOfFlute, double spindleSpeed, double samplingPeriod, double lamda, double _R, 
												 int numOfBand, double energyThreshold, double energyRatioLimit, double integrationFactor, double chatterEnergyThreshold, int ndMean, int delay)
{
	ChatterEnergyThreshold = chatterEnergyThreshold;
	SpindleSpeed = spindleSpeed;
	N = _N;
	NumberOfFlute = numOfFlute;
	SamplingPeriod = samplingPeriod;
	Lamda = lamda; 
	R = _R;
	//StartFreq = startFreq;
	//StopFreq = stopFreq; 
	//TPE = _TPE;
	//StopBandMag = stopBandMag;
	//RippleMag = rippleMag;
	EnergyThreshold = energyThreshold;
	EnergyRatioLimit = energyRatioLimit;
	IntegrationFactor = integrationFactor;
	SpindleRateSet = 100;
	ThresholdEng = 0;

	KalmanFilter = new CKalmanFilter(_N, spindleSpeed, samplingPeriod, lamda, _R);
	BandpassFilters = new CBandpassFilters(numOfBand, numOfFlute*spindleSpeed/(2 * PI), samplingPeriod);
	NEO = new CNonlinearEnergyOperator(samplingPeriod, numOfFlute*spindleSpeed / (2 * PI), *BandpassFilters);

	ChatterMeanFilter = new CMeanFilter(1, delay);
	PeriodicMeanFilter = new CMeanFilter(1, delay);
	FreqMeanFilter = new CMeanFilter(BandpassFilters->NumberOfFilters, ndMean);
	AmpMeanFilter = new CMeanFilter(BandpassFilters->NumberOfFilters, ndMean);

	ChatterDetection = new CChatterDetection(ndMean, energyThreshold, energyRatioLimit);

	ChatterFreq = new CMatrix(BandpassFilters->NumberOfFilters, 1);
	PrevChatterFreq = new CMatrix(BandpassFilters->NumberOfFilters, 1);
	ChatterFreqVariation = new CMatrix(BandpassFilters->NumberOfFilters, 1);
	ChatterEnergy =  new CMatrix(BandpassFilters->NumberOfFilters, 1);
	ChatterOutput = new SChatterOutput[BandpassFilters->NumberOfFilters];
}

CChatterDetectionSystem::~CChatterDetectionSystem()
{
	//cout << "CChatterDetectionSystem destructor is called..." << endl;
	delete KalmanFilter;
	delete BandpassFilters;
	delete NEO;
	delete ChatterMeanFilter;
	delete PeriodicMeanFilter;
	delete AmpMeanFilter;
	delete FreqMeanFilter;
	delete ChatterDetection;
	delete ChatterFreq;
	delete PrevChatterFreq;
	delete ChatterFreqVariation;
	delete[] ChatterOutput;
	delete ChatterEnergy;
}

void CChatterDetectionSystem::Run(double measurement)
{
	double estimation = KalmanFilter->RunKalman(measurement);
	//BandpassFilters->RunBandpassFilters(BandpassFilters->CalculateInput(measurement, estimation));
	BandpassFilters->RunBandpassFilters(measurement-estimation);
	NEO->RunNEO(BandpassFilters->BandpassOutputs);

	FreqMeanFilter->RunMeanFilter(NEO->Freq);
	AmpMeanFilter->RunMeanFilter(NEO->Amp);

	ChatterDetection->ChatterEnergy = ChatterMeanFilter->RunMeanFilter(CalculateChatterEnergy(FreqMeanFilter->MOutput, AmpMeanFilter->MOutput));
	ChatterDetection->PeriodicEnergy = PeriodicMeanFilter->RunMeanFilter(CalculatePeriodicEnergy(KalmanFilter->PeriodicAmp));

	ChatterDetection->RunChatterDetection();

	if (ChatterDetection->ChatterDetected == 1)
	{
		CalculateChatterFreq();
		double newSpindleSpeed = CalculateNewSpindleSpeed(ChatterFreq->Content[0]);
	}
}

double CChatterDetectionSystem::CalculateChatterEnergy(CMatrix* freqIn, CMatrix* ampIn)
{
	double totalChatterEnergy = 0;
	for (int i = 0; i < freqIn->Row; i++)
	{
		double currentFreq = freqIn->Content[i];
		double currentAmp = ampIn->Content[i];
		if (currentFreq != 0)
		{
			double chatterEng = pow(currentAmp, 2)*pow(currentFreq, 2 * IntegrationFactor);
			ChatterEnergy->Content[i] = chatterEng;
			totalChatterEnergy += chatterEng;
		}

		else
			ChatterEnergy->Content[i] = 0;
	}

	ThresholdEng = totalChatterEnergy*ChatterEnergyThreshold;
	return totalChatterEnergy;
}

double CChatterDetectionSystem::CalculatePeriodicEnergy(CMatrix* amplitude)
{
	double periodicEnergy = 0;

	for (int i = 0; i < amplitude->Row; i++)
	{
		double frequency = (i + 1)*SpindleSpeed;
		if (frequency != 0)
			periodicEnergy = periodicEnergy + pow(amplitude->Content[i], 2)*pow(frequency, 2 * IntegrationFactor);
	}

	return periodicEnergy;
}

void CChatterDetectionSystem::CalculateChatterFreq(/*MeanFilterOutput* meanFilterOutputs, double totalChatterEnergy, int numOfInput, double threshold*/)
{
	//sort(meanFilterOutputs, meanFilterOutputs + numOfInput, sortByChatterEnergy);
	int numOfInput = BandpassFilters->NumberOfFilters;
	double totalEng = 0;
	for (int i = 0; i < numOfInput; i++)
	{
		ChatterOutput[i].Freq = FreqMeanFilter->MOutput->Content[i];
		ChatterOutput[i].Amp = AmpMeanFilter->MOutput->Content[i];
		ChatterOutput[i].BandNumber = i + 1;
		ChatterOutput[i].ChatterEnergy = ChatterEnergy->Content[i];
		totalEng += ChatterEnergy->Content[i];
 	}

	//for (int i = 0; i < numOfInput; i++)
	//{
	//	cout << "Freq is " << ChatterOutput[i].Freq << endl;
	//	//cout << "Amp is " << ChatterOutput[i].Amp << endl;
	//	cout << "Chatter energy ratio is " << ChatterOutput[i].ChatterEnergy / totalEng << endl;
	//}
	//cout << "\n\n";

	double tempTotalEnergy = 0;

	for (int i = 0; i < numOfInput; i++)
	{
		SChatterOutput* maxChatterEnergy = max_element(ChatterOutput, ChatterOutput + numOfInput - i, sortByChatterEnergy);
		tempTotalEnergy += maxChatterEnergy->ChatterEnergy;

		ChatterFreq->Content[i] = maxChatterEnergy->Freq;
		//cout << "Chatter Frequency " << i << " is " << ChatterFreq->Content[i]  << "rad/s" << endl;

		if (tempTotalEnergy >= ThresholdEng)
		{
			for (int i = 0; i < numOfInput; i++)
			{
				//Calculate how much each chatter freq changes.
				ChatterFreqVariation->Content[i] = ChatterFreq->Content[i] - PrevChatterFreq->Content[i];
				//Update prevsious chatter frequencies
				PrevChatterFreq->Content[i] = ChatterFreq->Content[i];
			}
			return;
		}

		else
		{
			if (maxChatterEnergy->BandNumber != numOfInput - i)
			{
				ChatterOutput[maxChatterEnergy->BandNumber - 1].Amp = ChatterOutput[numOfInput - 1 - i].Amp;
				ChatterOutput[maxChatterEnergy->BandNumber - 1].Freq = ChatterOutput[numOfInput - 1 - i].Freq;
				ChatterOutput[maxChatterEnergy->BandNumber - 1].ChatterEnergy = ChatterOutput[numOfInput - 1 - i].ChatterEnergy;
			}
		}
	}
}

double CChatterDetectionSystem::CalculateNewSpindleSpeed(double dominantChatterFreq)
{
	double highSpindleSpeed = dominantChatterFreq / floor(dominantChatterFreq / SpindleSpeed);
	double lowSpindleSpeed = dominantChatterFreq / ceil(dominantChatterFreq / SpindleSpeed);
	double newSpindleSpeed;
	
	if (highSpindleSpeed / SpindleSpeed >= 1.5) //first check if override is bigger than 150% which is the limit
		newSpindleSpeed = lowSpindleSpeed;
	else if (highSpindleSpeed / (2 * PI) * 60 >= 12000) //then if high frequency > 12000 rpm which is the limit
		newSpindleSpeed = lowSpindleSpeed;
	else if ((BandpassFilters->NumberOfFilters + 1)* highSpindleSpeed / (2 * PI) >= 1/SamplingPeriod/2) //then if high frequency > Nyquist freq, choose the lower one
		newSpindleSpeed = lowSpindleSpeed;
	else
		newSpindleSpeed = highSpindleSpeed;

	SpindleRateSet = (int)(newSpindleSpeed / SpindleSpeed * 100);
	return newSpindleSpeed;
}

void CChatterDetectionSystem::UpdateSystem(double newSpindleSpeed)
{
	SpindleSpeed = newSpindleSpeed;
	KalmanFilter->UpdateKalman(newSpindleSpeed);
	BandpassFilters->UpdateBandpass(NumberOfFlute*newSpindleSpeed );
}

double CChatterDetectionSystem::FindDominantFreq(void)
{
	if (PrevChatterFreq->Content[0] == 0)
		return ChatterFreq->Content[0]; // first detected
	else
	{
		double minChange = ChatterFreqVariation->Content[0];
		int index = 0;
		for (int i = 1; i < ChatterFreq->Row; i++)
		{
			if (ChatterFreqVariation->Content[i] == 0)
			{
				break;
			}

			if (ChatterFreqVariation->Content[i] < minChange)
			{
				minChange = ChatterFreqVariation->Content[i];
				index = i;
			}
		}
		return ChatterFreq->Content[index];
	}
}

bool sortByChatterEnergy(SChatterOutput& lhs, SChatterOutput& rhs)
{
	return lhs.ChatterEnergy < rhs.ChatterEnergy;
}