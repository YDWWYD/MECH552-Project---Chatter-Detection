#ifndef __CKalmanFilter_
#define __CKalmanFilter_

#include "CMatrix.h"

class CKalmanFilter
{
private:
	CMatrix *Phi;
	int N;
	double SamplingPeriod;
	double Q;
	double R; //covariance
	CMatrix *q; //q_hat_k-1 [2N x 1]
	CMatrix *P; //P_har_k-1 [2N x 2N]
	CMatrix *qPrior; // q_hat_k_prior [2N x 1]
	CMatrix *PPrior; // P_hat_k_prior [2N x 2N]
	CMatrix *K; // K_k [2N x 1]

public:
	//double SpEst; // Estimation
	CMatrix* PeriodicAmp; //[N x 1]

public:
	CKalmanFilter(int N, double spindleSpeed, double samplingPeriod, double lamda, double covariance);
	~CKalmanFilter();
	void UpdateKalman(double newSpindleSpeed);
	double RunKalman(double measurement);
};

#endif
