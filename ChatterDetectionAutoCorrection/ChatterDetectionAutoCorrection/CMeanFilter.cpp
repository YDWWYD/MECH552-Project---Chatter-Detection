#include "CMeanFilter.h"

/// <summary>
/// Constructor
/// </summary>
/// <param name="numOfInput">number of input</param>
/// <param name="numOfDelay">number of previous data to take the average</param>
CMeanFilter::CMeanFilter(int numOfInput, int numOfDelay)
{
	Counter = 0;
	NumOfInput = numOfInput;
	NumDelay = (double)numOfDelay;
	Output = 0;
	MOutput = new CMatrix(numOfInput, 1); //Initial condition is 0
	MPrevOutput = new CMatrix(numOfInput, 1); //Initial condition is 0
	PrevOutput = 0;
}

CMeanFilter::~CMeanFilter()
{
	delete MOutput;
	delete MPrevOutput;
}

/// <summary>
/// Run recursive mean algorithm for matrix input
/// </summary>
/// <param name="input">matrix input: pointer to a matrix</param>
/// <returns>matrix output: pointer to a matrix</returns>
CMatrix* CMeanFilter::RunMeanFilter(CMatrix* input)
{
	for (int i = 0; i < NumOfInput; i++)
	{
		if (Counter<NumDelay)
		{
			// if counter = 0, output is 0
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

	// Update previous output
	for (int i = 0; i < NumOfInput; i++)
	{
		MPrevOutput->Content[i] = MOutput->Content[i];
	}

	// Increase counter
	Counter++;

	return MOutput;
}

/// <summary>
/// Run recursive mean algorithm for scalar input.
/// </summary>
/// <param name="input"> single scalar input</param>
/// <returns> single scalar ouput</returns>
double CMeanFilter::RunMeanFilter(double input)
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

	// Update previous output
	PrevOutput = Output;

	// Increase counter
	Counter++;

	return Output;
}