#include "CChatterDetectionSystem.h"

bool sortByChatterEnergy(SChatterOutput& lhs, SChatterOutput& rhs);

/// <summary>
/// Constructor
/// </summary>
/// <param name="_N">Number of harnomics</param>
/// <param name="numOfFlute">Number of flute</param>
/// <param name="spindleSpeed">Initial spindle speed [rad/s]</param>
/// <param name="samplingPeriod"> Sampling period [s]</param>
/// <param name="lamda">tuned value</param>
/// <param name="_R">covariance</param>
/// <param name="numOfBand">number of bandpass filters</param>
/// <param name="energyThreshold">threshold for periodic energy</param>
/// <param name="energyRatioLimit">limit of energy ratio</param>
/// <param name="integrationFactor">integration factor for energy calculation</param>
/// <param name="chatterEnergyThreshold">for chatter freq selection</param>
/// <param name="ndMean">delay of freq. and amp. mean filters</param>
/// <param name="delay">delay of energy mean filters</param>
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

/// <summary>
/// Run chatter detection system.
/// </summary>
/// <param name="measurement">input to the system: measurement from sensor</param>
void CChatterDetectionSystem::Run(double measurement)
{
	//Run Kalman filter, bandpass filter and NEO
	double estimation = KalmanFilter->RunKalman(measurement);
	BandpassFilters->RunBandpassFilters(measurement-estimation);
	NEO->RunNEO(BandpassFilters->BandpassOutputs);

	//Mean filter the frequency and amplitude output from NEO
	FreqMeanFilter->RunMeanFilter(NEO->Freq);
	AmpMeanFilter->RunMeanFilter(NEO->Amp);

	//Calculate periodic energy and chatter energy and mean filter the output
	ChatterDetection->ChatterEnergy = ChatterMeanFilter->RunMeanFilter(CalculateChatterEnergy(FreqMeanFilter->MOutput, AmpMeanFilter->MOutput));
	ChatterDetection->PeriodicEnergy = PeriodicMeanFilter->RunMeanFilter(CalculatePeriodicEnergy(KalmanFilter->PeriodicAmp));

	ChatterDetection->RunChatterDetection();

	// if chatter is detected, select chatter freuqency and calculate new spindle speed
	if (ChatterDetection->ChatterDetected == 1)
	{
		SelectChatterFreq();
		double newSpindleSpeed = CalculateNewSpindleSpeed(ChatterFreq->Content[0]);
	}
}

/// <summary>
/// Calculate the total chatter energy.
/// </summary>
/// <param name="freqIn">input frequency matrix: output freq matrix from NEO [rad/s]</param>
/// <param name="ampIn">input amplitude matrix: output amp matrix from NEO</param>
/// <returns>total chatter energy</returns>
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

		// if frequency is 0, outputs are initial conditions: 0. Do nothing.
		else
			ChatterEnergy->Content[i] = 0;
	}

	ThresholdEng = totalChatterEnergy*ChatterEnergyThreshold;
	return totalChatterEnergy;
}

/// <summary>
/// Calculate the total periodic energy.
/// </summary>
/// <param name="amplitude">periodic vibration amp matrix from Kalman filter</param>
/// <returns>total periodic energy</returns>
double CChatterDetectionSystem::CalculatePeriodicEnergy(CMatrix* amplitude)
{
	double periodicEnergy = 0;

	for (int i = 0; i < amplitude->Row; i++)
	{
		double frequency = (i + 1)*SpindleSpeed;

		// if frequency is 0, outputs are initial conditions: 0. Do nothing.
		if (frequency != 0)
			periodicEnergy = periodicEnergy + pow(amplitude->Content[i], 2)*pow(frequency, 2 * IntegrationFactor);
	}

	return periodicEnergy;
}

/// <summary>
/// Select chatter frequency using the criterion that minimum number of chatter frequencise whose total chatter energy
/// is bigger a threshold total chatter energy should be selected.
/// </summary>
void CChatterDetectionSystem::SelectChatterFreq()
{
	//sort(meanFilterOutputs, meanFilterOutputs + numOfInput, sortByChatterEnergy);
	int numOfInput = BandpassFilters->NumberOfFilters;
	//double totalEng = 0;
	// store chatter information into the structure array for selection later
	for (int i = 0; i < numOfInput; i++)
	{
		ChatterOutput[i].Freq = FreqMeanFilter->MOutput->Content[i];
		ChatterOutput[i].Amp = AmpMeanFilter->MOutput->Content[i];
		ChatterOutput[i].BandNumber = i + 1;
		ChatterOutput[i].ChatterEnergy = ChatterEnergy->Content[i];
		//totalEng += ChatterEnergy->Content[i];
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
		// output the band with max chatter energy from the 1st to the (numOfInput - i)th element in the structure
		// To improve performance by not searching the whole structure array
		SChatterOutput* maxChatterEnergy = max_element(ChatterOutput, ChatterOutput + numOfInput - i, sortByChatterEnergy);

		tempTotalEnergy += maxChatterEnergy->ChatterEnergy;

		//Select one and save it in chatter frequency matrix
		ChatterFreq->Content[i] = maxChatterEnergy->Freq;

		//cout << "Chatter Frequency " << i << " is " << ChatterFreq->Content[i]  << "rad/s" << endl;

		// if the selected total chatter energy is bigger than the threshold, stop selection
		if (tempTotalEnergy >= ThresholdEng)
		{
			for (int i = 0; i < numOfInput; i++)
			{
				//Calculate how much each chatter freq changes.
				ChatterFreqVariation->Content[i] = ChatterFreq->Content[i] - PrevChatterFreq->Content[i];
				//Update previous chatter frequencies
				PrevChatterFreq->Content[i] = ChatterFreq->Content[i];
			}
			return;
		}

		// if the selected total chatter energy is smaller than the threshold, move the selected element to the current end of array.
		// The last element will be ignored in the next loop just to improve performace.
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

/// <summary>
/// Calculate the new spindle speed using the most dominant chatter frequency
/// </summary>
/// <param name="dominantChatterFreq">selected chatter frequency [rad/s]</param>
/// <returns>new spindle speed [rad/s]</returns>
double CChatterDetectionSystem::CalculateNewSpindleSpeed(double dominantChatterFreq)
{
	double highSpindleSpeed = dominantChatterFreq / floor(dominantChatterFreq / SpindleSpeed);
	double lowSpindleSpeed = dominantChatterFreq / ceil(dominantChatterFreq / SpindleSpeed);
	double newSpindleSpeed;
	
	if (highSpindleSpeed / SpindleSpeed >= 1.5) //first check if spindle speed override is bigger than 150% which is the limit of Heidenhain CNC
		newSpindleSpeed = lowSpindleSpeed;
	else if (highSpindleSpeed / (2 * PI) * 60 >= 12000) //then check if high frequency > 12000 rpm which is the limit of Heidenhain CNC
		newSpindleSpeed = lowSpindleSpeed;
	else if ((BandpassFilters->NumberOfFilters + 1)* highSpindleSpeed / (2 * PI) >= 1/SamplingPeriod/2) //then check if high frequency > Nyquist freq, choose the lower one
		newSpindleSpeed = lowSpindleSpeed;
	else
		newSpindleSpeed = highSpindleSpeed;

	SpindleRateSet = (int)(newSpindleSpeed / SpindleSpeed * 100); // spindle speed override to CNC in percentage
	return newSpindleSpeed;
}

/// <summary>
/// Update the detection system when spindle speed changes. Only Kalman and bandpass filters are influenced.
/// </summary>
/// <param name="newSpindleSpeed">new spindle speed [rad/s]</param>
void CChatterDetectionSystem::UpdateSystem(double newSpindleSpeed)
{
	SpindleSpeed = newSpindleSpeed;
	KalmanFilter->UpdateKalman(newSpindleSpeed);
	BandpassFilters->UpdateBandpass(NumberOfFlute*newSpindleSpeed / 2 / PI);
}

/// <summary>
/// Select the most dominant chatter frequency which is the one that varies the least amount
/// </summary>
/// <returns>the most dominant chatter frequency [rad/s]</returns>
double CChatterDetectionSystem::FindDominantFreq(void)
{
	// if chatter is first detected, select the 1st one with highest chatter energy
	if (PrevChatterFreq->Content[0] == 0)
		return ChatterFreq->Content[0]; 
	else // find the one with the smallest varivation
	{
		double minChange = ChatterFreqVariation->Content[0];
		int index = 0;
		for (int i = 1; i < ChatterFreq->Row; i++)
		{
			if (ChatterFreqVariation->Content[i] == 0) // if reaches the 0 element, break from the loop
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

/// <summary>
/// sort a structure array by customized rule. Here by chatter energy.
/// </summary>
/// <param name="lhs">reference of the left hand side structure</param>
/// <param name="rhs">reference of the right hand side structure</param>
/// <returns>boolean</returns>
bool sortByChatterEnergy(SChatterOutput& lhs, SChatterOutput& rhs)
{
	return lhs.ChatterEnergy < rhs.ChatterEnergy;
}