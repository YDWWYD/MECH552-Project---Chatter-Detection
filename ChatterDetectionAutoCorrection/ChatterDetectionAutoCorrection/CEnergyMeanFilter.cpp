#include "CEnergyMeanFilter.h"

CEnergyMeanFilter::CEnergyMeanFilter(int numOfInput, int numOfDelay)
{
	Counter = 0;
	NumOfInput = numOfInput;
	NumDelay = (double)numOfDelay;
	Output = 0;
	MOutput = new CMatrix(numOfInput, 1);
	MPrevOutput = new CMatrix(numOfInput, 1);
	//PrevOutput = new double[numOfInput];
	PrevOutput = 0;
}

CEnergyMeanFilter::~CEnergyMeanFilter()
{
	delete MOutput;
	delete MPrevOutput;
}

CMatrix* CEnergyMeanFilter::RunMeanFilter(CMatrix* input)
{
	for (int i = 0; i < NumOfInput; i++)
	{
		if (Counter<NumDelay)
		{
			if (Counter != 0)
			{
				MOutput->Content[i] = (Counter - 1) / Counter*MPrevOutput->Content[i] + 1 / Counter*input->Content[i];
			}
		}
		else
		{
			MOutput->Content[i] = (NumDelay - 1) / NumDelay*MPrevOutput->Content[i] + 1 / NumDelay*input->Content[i];
		}
	}

	// Update previous average chatter and periodic energy
	//for (int i = 0; i < NumOfInput; i++)
	//{
	//	MPrevOutput->Content[i] = MOutput->Content[i]; // in [rad/s]
	//}
	MPrevOutput = MOutput;

	Counter++;

	return MOutput;
}

double CEnergyMeanFilter::RunMeanFilter(double input)
{
	if (Counter<NumDelay)
	{
		if (Counter != 0)
		{
			Output = (Counter - 1) / Counter*PrevOutput + 1 / Counter*input;
		}
	}
	else
	{
		Output = (NumDelay - 1) / NumDelay*PrevOutput + 1 / NumDelay*input;
	}

	// Update previous average chatter and periodic energy
	PrevOutput = Output;

	Counter++;

	return Output;
}