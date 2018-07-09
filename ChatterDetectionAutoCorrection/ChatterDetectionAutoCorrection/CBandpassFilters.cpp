#include "CBandpassFilters.h"

/// <summary>
/// Constructor
/// </summary>
/// <param name="numOfBand">number of bandpass filter</param>
/// <param name="TPE">tooth-passing frequency:[Hz]</param>
/// <param name="Ts">sampling period:[s]</param>
CBandpassFilters::CBandpassFilters(int numOfBand, double TPE, double Ts)
{
	SamplingPeriod = Ts;
	Order = 4; // always 4 in this project
	StartBand = 1; // always 1 in this project: ceil(startFreq / TPE)
	NumberOfFilters = numOfBand;

	Numerator = new CMatrix(NumberOfFilters, 2 * Order + 1);
	Denominator = new CMatrix(NumberOfFilters, 2 * Order);
	E = new CMatrix(NumberOfFilters, 2 * Order + 1);
	EDelayed = new CMatrix(NumberOfFilters, 2 * Order);
	EDelayedSingle = new CMatrix(2 * Order, 1);
	ASingle = new CMatrix(1, 2 * Order);
	BSingle = new CMatrix(1, 2 * Order + 1); 
	ESingleTrans = new CMatrix(2 * Order + 1, 1);
	BandpassOutputs = new CMatrix(NumberOfFilters, 1);

	CalculateCoefficients(TPE);
}

CBandpassFilters:: ~CBandpassFilters()
{
	delete Numerator;
	delete Denominator;
	delete E;
	delete EDelayed;
	delete EDelayedSingle;
	delete ASingle;
	delete BSingle;
	delete ESingleTrans;
	delete BandpassOutputs;
}

//d8 = (2 / Ts*Q / w0) ^ 4;
//d7 = (a1 + a2)*(2 / Ts*Q / w0) ^ 3;
//d6 = (2 / Ts*Q / w0) ^ 2 * (a1*a2 + 2 * (2 * Q ^ 2 + 1));
//d5 = (2 / Ts) ^ 1 * (a1 + a2)*Q / w0*(3 * Q ^ 2 + 1);
//d4 = (6 * Q ^ 4 + (2 * a1*a2 + 4)*Q ^ 2 + 1);
//d3 = (2 / Ts) ^ -1 * (a1 + a2)*w0*Q*(1 + 3 * Q ^ 2);
//d2 = (Ts / 2 * Q*w0) ^ 2 * (4 * Q ^ 2 + 2 + a1*a2);
//d1 = (a1 + a2)*(Ts / 2 * Q*w0) ^ 3;
//d0 = (Q*w0*Ts / 2) ^ 4;
//
//dd8 = d0 - d1 + d2 - d3 + d4 - d5 + d6 - d7 + d8;
//dd7 = 8 * d0 - 6 * d1 + 4 * d2 - 2 * d3 + 2 * d5 - 4 * d6 + 6 * d7 - 8 * d8;
//dd6 = 28 * d0 - 14 * d1 + 4 * d2 + 2 * d3 - 4 * d4 + 2 * d5 + 4 * d6 - 14 * d7 + 28 * d8;
//dd5 = 56 * d0 - 14 * d1 - 4 * d2 + 6 * d3 - 6 * d5 + 4 * d6 + 14 * d7 - 56 * d8;
//dd4 = 70 * d0 - 10 * d2 + 6 * d4 - 10 * d6 + 70 * d8;
//dd3 = 56 * d0 + 14 * d1 - 4 * d2 - 6 * d3 + 6 * d5 + 4 * d6 - 14 * d7 - 56 * d8;
//dd2 = 28 * d0 + 14 * d1 + 4 * d2 - 2 * d3 - 4 * d4 - 2 * d5 + 4 * d6 + 14 * d7 + 28 * d8;
//dd1 = 8 * d0 + 6 * d1 + 4 * d2 + 2 * d3 - 2 * d5 - 4 * d6 - 6 * d7 - 8 * d8;
//dd0 = d0 + d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8;
//numCoeffs = (2 / Ts) ^ 4 * [1, 0, -4, 0, 6, 0, -4, 0, 1];
/// <summary>
/// Calculate coefficients of numerator and denominator of transfer function for each bandpass filter.
/// Only final expressions are coded here. Refer to MIT notes: continusous_discrete_transformation.pdf for detailed explanations.
/// </summary>
/// <param name="TPE">tooth-passing frequency [Hz]</param>
void CBandpassFilters::CalculateCoefficients(double TPE)
{
	double a1 = -0.7654; //this is constant parameter
	double a2 = -1.8478; //this is constant parameter
	double stopBand[2]; //wc_c [Hz]
	double wrappedStopBand[2]; //wc_d [Hz]
	double w0 = 0;
	double dw = 0;
	double Q = 0;

	for (int i = 0; i < NumberOfFilters; i++)
	{
		stopBand[0] = (1*TPE + i * TPE);
		stopBand[1] = stopBand[0] + TPE;

		wrappedStopBand[0] = 2.0 / SamplingPeriod*tan(stopBand[0] * (2 * PI) * SamplingPeriod / 2); //[Hz]
		wrappedStopBand[1] = 2.0 / SamplingPeriod*tan(stopBand[1] * (2 * PI) * SamplingPeriod / 2); //[Hz]

		w0 = pow(wrappedStopBand[0] * wrappedStopBand[1], 0.5); // [Hz]
		dw = wrappedStopBand[1] - wrappedStopBand[0]; // [Hz]
		Q = w0 / dw; //unitless

		double d8 = pow(2 / SamplingPeriod*Q / w0, 4);
		double d7 = (a1 + a2)*pow(2 / SamplingPeriod*Q / w0, 3);
		double d6 = pow(2 / SamplingPeriod*Q / w0, 2) * (a1*a2 + 2 * (2 * pow(Q, 2) + 1));
		double d5 = (2 / SamplingPeriod) * (a1 + a2)*Q / w0*(3 * pow(Q, 2) + 1);
		double d4 = (6 * pow(Q, 4) + (2 * a1*a2 + 4)* pow(Q, 2) + 1);
		double d3 = pow(2 / SamplingPeriod, -1) * (a1 + a2)*w0*Q*(1 + 3 * pow(Q, 2));
		double d2 = pow(SamplingPeriod / 2 * Q*w0, 2) * (4 * pow(Q, 2) + 2 + a1*a2);
		double d1 = (a1 + a2)*pow(SamplingPeriod / 2 * Q*w0, 3);
		double d0 = pow(Q*w0*SamplingPeriod / 2, 4);

		double dd8 = d0 - d1 + d2 - d3 + d4 - d5 + d6 - d7 + d8;
		double dd7 = 8 * d0 - 6 * d1 + 4 * d2 - 2 * d3 + 2 * d5 - 4 * d6 + 6 * d7 - 8 * d8;
		double dd6 = 28 * d0 - 14 * d1 + 4 * d2 + 2 * d3 - 4 * d4 + 2 * d5 + 4 * d6 - 14 * d7 + 28 * d8;
		double dd5 = 56 * d0 - 14 * d1 - 4 * d2 + 6 * d3 - 6 * d5 + 4 * d6 + 14 * d7 - 56 * d8;
		double dd4 = 70 * d0 - 10 * d2 + 6 * d4 - 10 * d6 + 70 * d8;
		double dd3 = 56 * d0 + 14 * d1 - 4 * d2 - 6 * d3 + 6 * d5 + 4 * d6 - 14 * d7 - 56 * d8;
		double dd2 = 28 * d0 + 14 * d1 + 4 * d2 - 2 * d3 - 4 * d4 - 2 * d5 + 4 * d6 + 14 * d7 + 28 * d8;
		double dd1 = 8 * d0 + 6 * d1 + 4 * d2 + 2 * d3 - 2 * d5 - 4 * d6 - 6 * d7 - 8 * d8;
		double dd0 = d0 + d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8;

		// Numerator
		Numerator->Content[i*Numerator->Column + 0] = 1.0 / dd8;
		Numerator->Content[i*Numerator->Column + 1] = 0;
		Numerator->Content[i*Numerator->Column + 2] = (-4.0) / dd8;
		Numerator->Content[i*Numerator->Column + 3] = 0 / dd8;
		Numerator->Content[i*Numerator->Column + 4] = 6.0 / dd8;
		Numerator->Content[i*Numerator->Column + 5] = 0 / dd8;
		Numerator->Content[i*Numerator->Column + 6] = (-4.0) / dd8;
		Numerator->Content[i*Numerator->Column + 7] = 0 / dd8;
		Numerator->Content[i*Numerator->Column + 8] = 1.0 / dd8;

		// Denominator
		Denominator->Content[i*Denominator->Column + 0] = dd7 / dd8;
		Denominator->Content[i*Denominator->Column + 1] = dd6 / dd8;
		Denominator->Content[i*Denominator->Column + 2] = dd5 / dd8;
		Denominator->Content[i*Denominator->Column + 3] = dd4 / dd8;
		Denominator->Content[i*Denominator->Column + 4] = dd3 / dd8;
		Denominator->Content[i*Denominator->Column + 5] = dd2 / dd8;
		Denominator->Content[i*Denominator->Column + 6] = dd1 / dd8;
		Denominator->Content[i*Denominator->Column + 7] = dd0 / dd8;
	}
}

double CBandpassFilters::CalculateInput(double measurement, double estimation)
{
	return measurement - estimation;
}

/// <summary>
/// Update bandpass filters when spindle speed changes. Transfer function changes so calculate coefficient again using new spidnle speed.
/// </summary>
/// <param name="newTPE">new tooth-passing frequency [Hz]</param>
void CBandpassFilters::UpdateBandpass(double newTPE) // # of filters is actually fixed
{
	CalculateCoefficients(newTPE);
}

/// <summary>
/// Run bandpss filter's algorithm. Refer to Bandpass Filter Bank Simulink block for detailed information.
/// </summary>
/// <param name="input">input to the bandpass filter</param>
void CBandpassFilters::RunBandpassFilters(double input)
{
	// ----- For loop to calculate A*(E_delayed)' of each filter and update 1st column of E signal
	for (int i = 0; i < NumberOfFilters; i++)
	{
		EDelayedSingle->ExtractRowToColumn(EDelayed, i); // Ed': [2n x 1]
		ASingle->ExtractRow(Denominator, i); // Extract A of #(i+1) filter: [1 x 2n]
		E->Content[i*E->Column] = input - (*ASingle * *EDelayedSingle).Content[0];// (INPUT - A*Ed') of #i filter [1x1]
	}

	// Update E signal of the remaining columns: copy from EDelayed signal
	for (int i = 0; i < NumberOfFilters; i++) // # of filters
	{
		for (int j = 0; j < EDelayed->Column; j++)
		{
			E->Content[i*E->Column + j + 1] = EDelayed->Content[i*EDelayed->Column + j];
		}
	}

	// ---- for loop to calculate each filter's output---------
	for (int i = 0; i < NumberOfFilters; i++)
	{
		ESingleTrans->ExtractRowToColumn(E, i); // E': [2n+1 x 1]
		BSingle->ExtractRow(Numerator, i); // B of #(i+1) filter: [1 x 2n+1]
		BandpassOutputs->Content[i] = (*BSingle**ESingleTrans).Content[0];// Output of #(i+1) filter: [1 x 1]
	}

	// Update delayed E
	for (int i = 0; i < NumberOfFilters; i++)
	{
		for (int j = 0; j < EDelayed->Column - 1; j++)
		{
			int index1 = i*EDelayed->Column + EDelayed->Column - 1 - j;
			EDelayed->Content[index1] = EDelayed->Content[index1 - 1];
		}
		EDelayed->Content[i*EDelayed->Column] = E->Content[i*E->Column];
	}
}