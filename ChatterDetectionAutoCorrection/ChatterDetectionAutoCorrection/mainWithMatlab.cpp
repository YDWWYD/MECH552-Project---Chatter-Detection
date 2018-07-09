//#include <iostream>
//#include <iomanip>
//#include "CMatrix.h"
//#include "CKalmanFilter.h"
//#include "CBandpassFilters.h"
//#include "CNonlinearEnergyOperator.h"
//#include "CMeanFilter.h"
//#include "CChatterDetection.h"
//#include "CChatterDetectionSystem.h"
//#include <fstream>
//#include <string>
//#include <time.h> // for clock
//#include <chrono> // for measuring accurate time, eg microsecond
//#include "engine.h"
//
//using namespace std;
//double* ReadMeasurements(char* filePath);
//
//int main(void)
//{
//	int N;
//	char* filePath;
//	int numOfData;
//	double Ts;
//
//	int testNum = 1;
//	switch (testNum)
//	{
//	case(1): N = 20;
//		filePath = "..\\CNCData.txt";
//		numOfData = 100032;
//		//numOfData = 35701;
//		Ts = (double)1 / 10000;
//		break;
//	case(2): N = 50;
//		filePath = "..\\testData.txt";
//		numOfData = 150016;
//		Ts = (double)1 / 10000;
//		break;
//	default:
//		N = 20;
//		filePath = "..\\Acc2.txt";
//		numOfData = 152788;
//		Ts = (double)1 / 25600;
//		break;
//	}
//
//	int numOfFlute = 2;
//	double Fs = 1.0 / Ts;
//	double spindleSpeed = 9000 * ((double)1 / 60 * 2 * PI); //  RPM->[rad/s] !!!!!!!! Initial Spindle Speed: Must manually change before each test
//
//	if (N*spindleSpeed / (2 * PI) >= Fs / 2) // check if max freq is bigger than Nyquist frequency
//		N = (int)floor((Fs / 2) / (spindleSpeed / (2 * PI)));
//
//	double lamda = 1E-6;
//	double R = 8.539750000056574E-6;
//	//double R = 0.15;
//
//	double* measurements = ReadMeasurements(filePath);
//	double TPE = spindleSpeed / (2 * PI)*numOfFlute; // [Hz] Original: 400Hz
//													 //double stopBandMag = 5; //dB Original: 5
//													 //double rippleMag = 2; //dB Original: 2
//	double integrationFactor = 0; // factor is 0 or 1 or -1, depending on the type of signal
//	double energyThreshold = 1E-8;
//	double energyRatioLimit = 0.2;
//	int energyMeanFilterDelay = (int)round(1 / (spindleSpeed / 2 / PI) / Ts / 2) * 5; // delays of chatter and periodic energy filters
//	int ndMean = (int)round(1 / (spindleSpeed / 2 / PI) / Ts); // delay of amp and freq mean filters
//
//	double startFreq = spindleSpeed / (2 * PI); // [Hz]
//	double stopFreq = N*startFreq; // [Hz]
//
//	int numOfBand;
//	if (N <= numOfFlute)
//		numOfBand = 1;
//	else
//		numOfBand = (int)ceil(stopFreq / TPE) - 1;
//
//	double dt = 0.1; // Original 0.1
//
//	double chatterEnergyThreshold = 0.6; //Used for selecting chatter freq.
//	clock_t clockBegin, clockEnd;
//
//	CChatterDetectionSystem myChatterDetectionSys(N, numOfFlute, spindleSpeed, Ts, lamda, R, numOfBand,
//		energyThreshold, energyRatioLimit, integrationFactor, chatterEnergyThreshold, ndMean, energyMeanFilterDelay);
//
//	//--------------------------------Matlab Engine------------------
//	Engine *ep;
//	mxArray *Chatter = NULL; 
//	mxArray *T = NULL;
//	mxArray *Bandpass = NULL;
//	double *ChatterArray = new double[numOfData];
//	double *time = new double[numOfData];
//	double *bandpassArray = new double[numOfData*numOfBand];
//
//	//FILE* OutFilePointer = fopen("..\\Kalman Output.txt", "w");
//	FILE* BandpassPointer = fopen("..\\Bandpass Filter Output.txt", "w");
//	FILE* FrequencyPointer = fopen("..\\Frequency Output.txt", "w");
//	FILE* AmplitudePointer = fopen("..\\Amplitude Output.txt", "w");
//	FILE* FilteredFreqPointer = fopen("..\\Filtered Freq Output.txt", "w");
//	FILE* FilteredAmpPointer = fopen("..\\Filtered Amp Output.txt", "w");
//	FILE* ChatterDetectionPointer = fopen("..\\Chatter Detection Output.txt", "w");
//	FILE* TimerPointer = fopen("..\\Timer Output.txt", "w");
//	FILE* FilteredChatterEng = fopen("..\\Filter Chatter Eng. Output.txt", "w");
//	FILE* FilteredPeriodicEng = fopen("..\\Filter Periodic Eng. Output.txt", "w");
//
//	bool flag = true;
//
//	//clockBegin = clock();		
//	std::cout << "System started, wait for it to finish...." << endl;
//	for (int i = 0; i < numOfData; i++)
//	{
//		myChatterDetectionSys.Run(measurements[i]);
//
//		//for (int j = 0; j < myChatterDetectionSys.BandpassFilters->NumberOfFilters; j++)
//		//{
//		//	fprintf(BandpassPointer, "%.8f	", myChatterDetectionSys.BandpassFilters->BandpassOutputs->Content[j]);
//		//	fprintf(FrequencyPointer, "%.8f	", myChatterDetectionSys.NEO->Freq->Content[j] / 2 / PI); //[Hz]
//		//	fprintf(AmplitudePointer, "%.8f	", myChatterDetectionSys.NEO->Amp->Content[j]);
//		//	fprintf(FilteredFreqPointer, "%.8f	", myChatterDetectionSys.FreqMeanFilter->MOutput->Content[j]);//[rad/s]
//		//	fprintf(FilteredAmpPointer, "%.8f	", myChatterDetectionSys.AmpMeanFilter->MOutput->Content[j]);
//		//}
//		//fprintf(BandpassPointer, "\n");
//		//fprintf(FrequencyPointer, "\n");
//		//fprintf(AmplitudePointer, "\n");
//		//fprintf(FilteredFreqPointer, "\n");
//		//fprintf(FilteredAmpPointer, "\n");
//		fprintf(ChatterDetectionPointer, "%d\n", myChatterDetectionSys.ChatterDetection->ChatterDetected);
//		//fprintf(FilteredChatterEng, "%.6f\n", myChatterDetectionSys.ChatterMeanFilter->Output);
//		//fprintf(FilteredPeriodicEng, "%.6f\n", myChatterDetectionSys.PeriodicMeanFilter->Output);
//		//fprintf(TimerPointer, "%d\n", chrono::duration_cast<chrono::microseconds>(time).count());
//
//
//		for (int j = 0; j < myChatterDetectionSys.BandpassFilters->NumberOfFilters; j++)
//		{
//			bandpassArray[i*numOfBand + j] = myChatterDetectionSys.BandpassFilters->BandpassOutputs->Content[j];
//		}
//		ChatterArray[i] = myChatterDetectionSys.ChatterDetection->ChatterDetected;
//		time[i] = Ts*i;
//	}
//
//	cout << "done!\n";
//
//	fclose(BandpassPointer);
//	fclose(FrequencyPointer);
//	fclose(AmplitudePointer);
//	fclose(FilteredFreqPointer);
//	fclose(FilteredAmpPointer);
//	fclose(ChatterDetectionPointer);
//	fclose(FilteredChatterEng);
//	fclose(FilteredPeriodicEng);
//	fclose(TimerPointer);
//
//	cout << "Processing data" << endl;
//	if (!(ep = engOpen(""))) {
//		fprintf(stderr, "\nCan't start MATLAB engine\n");
//		return EXIT_FAILURE;
//	}
//
//	engEvalString(ep, "clear, clc, close all");
//	engEvalString(ep, "CompareRealTime; plot(ChatterDetected.time, ChatterDetected.signals.values)");
//
//	Chatter = mxCreateDoubleMatrix(numOfData, 1, mxREAL);
//	memcpy((void*)mxGetPr(Chatter), (void *)ChatterArray, sizeof(double)*numOfData);
//	engPutVariable(ep, "CChatterDetected", Chatter);
//
//	T = mxCreateDoubleMatrix(numOfData, 1, mxREAL);
//	memcpy((void*)mxGetPr(T), (void *)time, sizeof(double)*numOfData);
//	engPutVariable(ep, "T", T);
//
//	Bandpass = mxCreateDoubleMatrix(numOfBand, numOfData, mxREAL);
//	memcpy((void*)mxGetPr(Bandpass), (void *)bandpassArray, sizeof(double)*numOfData*numOfBand);
//	engPutVariable(ep, "CBandpass", Bandpass);
//
//	//engEvalString(ep, "figure; plot(T,CChatterDetected)");
//	engEvalString(ep, "hold on; plot(T,CChatterDetected); legend('C','matlab')");
//	//engEvalString(ep, "legend('C','matlab')");
//	//engEvalString(ep, "figure; plot(T,CBandpass)");
//	cout << "Processing data done." << endl;
//
//	/// destroy all 
//	engClose(ep);
//
//	system("pause");
//	return 0;
//}
//
//double* ReadMeasurements(char* filePath)
//{
//	char data;
//	int dataNumber = 0;
//	int i;
//	FILE* fp;
//	fp = fopen(filePath, "r");
//
//	if (fp == NULL)
//	{
//		printf("Could not open file!\n");
//		return NULL;
//	}
//
//	while ((data = fgetc(fp)) != EOF)
//	{
//		if (data == '\n')
//			dataNumber++;
//	}
//
//	dataNumber++; // ++1 since no '\n'(return line character) for the last data
//
//	fclose(fp);
//	//FILE* fp2 = fopen(filePath, "r");
//	fp = fopen(filePath, "r");
//	double* measurement = (double*)calloc(dataNumber, sizeof(double));
//
//	char* test = (char *)malloc(100 * sizeof(char)); // allocate memory for a string of 100 characters
//
//	for (i = 0; i < dataNumber; i++)
//	{
//		fscanf(fp, "%s", test);
//		measurement[i] = atof(test);
//	}
//
//	fclose(fp);
//	return measurement;
//}