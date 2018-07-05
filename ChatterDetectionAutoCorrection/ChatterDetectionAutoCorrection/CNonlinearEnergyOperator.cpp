#include "CNonlinearEnergyOperator.h"

CNonlinearEnergyOperator::CNonlinearEnergyOperator(double samplingPeriod, double TPE, CBandpassFilters& bandpassFilters)
{
	T = samplingPeriod;
	D = new CMatrix(1, bandpassFilters.NumberOfFilters);
	*D = CalculateLag(samplingPeriod, TPE, bandpassFilters);
	//D->PrintMatrix();
	SDelayed = new CMatrix(bandpassFilters.NumberOfFilters, 4 * (int)D->matrixMax());
	phi_y_kD = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	phi_y_k2D = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	phi_s_k2D = new CMatrix(bandpassFilters.NumberOfFilters, 1);

	s_k = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	s_kD = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	s_k2D = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	s_k3D = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	s_k4D = new CMatrix(bandpassFilters.NumberOfFilters, 1);

	y_k = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	y_kD = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	y_k2D = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	y_k3D = new CMatrix(bandpassFilters.NumberOfFilters, 1);

	Freq = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	Amp = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	PrevFreq = new CMatrix(bandpassFilters.NumberOfFilters, 1);
	PrevAmp = new CMatrix(bandpassFilters.NumberOfFilters, 1);
}

CMatrix CNonlinearEnergyOperator:: CalculateLag(double samplingPeriod, double TPE, CBandpassFilters& bandpassFilters)
{
	CMatrix D(1, bandpassFilters.NumberOfFilters); // Initialize lag matrix 

	for (int i = 0; i < bandpassFilters.NumberOfFilters; i++)
	{
		D.Content[i] = round((1 / samplingPeriod) / (4 * (bandpassFilters.StartBand + i + 1)*TPE + TPE / 2) + 0.5);
	}

	return D;
}

CNonlinearEnergyOperator::~CNonlinearEnergyOperator(void)
{
	delete D; //lag matrix
	delete SDelayed;
	delete phi_y_kD;
	delete phi_y_k2D;
	delete phi_s_k2D;

	delete s_k;
	delete s_kD;
	delete s_k2D;
	delete s_k3D;
	delete s_k4D;

	delete y_k;
	delete y_kD;
	delete y_k2D;
	delete y_k3D;

	delete Freq;
	delete Amp;
	delete PrevFreq;
	delete PrevAmp;
}

void CNonlinearEnergyOperator::RunNEO(CMatrix* input)
{
	// Update input from bandpass filter
	for (int i = 0; i < input->Row; i++)
	{
		int delay = (int)D->Content[i];
		int index = (i + 1)*SDelayed->Column;
		s_k->Content[i] = input->Content[i];
		s_kD->Content[i] = SDelayed->Content[index - delay];
		s_k2D->Content[i] = SDelayed->Content[index - 2 * delay];
		s_k3D->Content[i] = SDelayed->Content[index - 3 * delay];
		s_k4D->Content[i] = SDelayed->Content[index - 4 * delay];
	}

	// Calculate y_k, y_k-D, y_k-2D and y_k-3D 
	*y_k = *s_k - *s_kD;
	*y_kD = *s_kD - *s_k2D;
	*y_k2D = *s_k2D - *s_k3D;
	*y_k3D = *s_k3D - *s_k4D;

	// Calculate Phi[y_(k-D)], Phi[y_(k-2D)], Phi[s_(k-2D)]
	for (int i = 0; i < input->Row; i++)
	{
		//phi_y_kD.content[i][0] = 1 / pow(T, 2)*(pow(y_kD.content[i][0], 2) - y_k2D.content[i][0] * y_k.content[i][0]);
		//phi_y_k2D.content[i][0] = 1 / pow(T, 2)*(pow(y_k2D.content[i][0], 2) - y_k3D.content[i][0] * y_kD.content[i][0]);
		//phi_s_k2D.content[i][0] = 1 / pow(T, 2)*(pow(s_k2D.content[i][0], 2) - s_k3D.content[i][0] * s_kD.content[i][0]);
		phi_y_kD->Content[i] = pow(y_kD->Content[i], 2) - y_k2D->Content[i] * y_k->Content[i];
		phi_y_k2D->Content[i] = pow(y_k2D->Content[i], 2) - y_k3D->Content[i] * y_kD->Content[i];
		phi_s_k2D->Content[i] = pow(s_k2D->Content[i], 2) - s_k3D->Content[i] * s_kD->Content[i];
	}

	// Calculate frequency and amplitude
	for (int i = 0; i < input->Row; i++)
	{
		double phi_xd = phi_y_k2D->Content[i] + phi_y_kD->Content[i];
		if (phi_s_k2D->Content[i] > 0 && phi_xd > 0 && phi_xd < 8 * phi_s_k2D->Content[i])
		{
			Freq->Content[i] = acos(1 - phi_xd / (4 * phi_s_k2D->Content[i])) / T / D->Content[i]; // in rad/s
			Amp->Content[i] = pow(phi_s_k2D->Content[i] / (1 - pow(1 - phi_xd / (4 * phi_s_k2D->Content[i]), 2)), 0.5);
		}
		else
		{
			Freq->Content[i] = PrevFreq->Content[i]; // in rad/s
			Amp->Content[i] = PrevAmp->Content[i];
		}
	}

	// Update delayed input from bandpass filter
	for (int i = 0; i < input->Row; i++)
	{
		for (int j = 0; j < SDelayed->Column - 1; j++)
		{
			int index = i*SDelayed->Column + j;
			SDelayed->Content[index] = SDelayed->Content[index + 1];
		}
		SDelayed->Content[(i+1)*SDelayed->Column - 1] = input->Content[i];
	}

	// Update previous frequency and amplitude
	for (int i = 0; i < input->Row; i++)
	{
		PrevFreq->Content[i] = Freq->Content[i];
		PrevAmp->Content[i] = Amp->Content[i];
	}
}