#ifndef __CKalmanFilter_
#define __CKalmanFilter_

#include "CMatrix.h"

/// <summary>
/// Class of Kalman filter for the estimation of periodic vibration signal
/// </summary>

class CKalmanFilter
{
private:
	CMatrix *Phi; // [2N x 1]: A matrix of state state equation
	int N; // number of harmonics
	double SamplingPeriod;
	double Q; //element of the diagonal process noise covariance matrix
	double R; //covariance
	CMatrix *q; //q_hat_k-1 [2N x 1]: posteriori estimation of state matrix
	CMatrix *P; //P_har_k-1 [2N x 2N]: posteriori estimation error covariance matrix
	CMatrix *qPrior; // q_hat_k_prior [2N x 1]: priori estimation of state matrix
	CMatrix *PPrior; // P_hat_k_prior [2N x 2N]: priori estimation error covariance matrix
	CMatrix *K; // K_k [2N x 1]: Kalman gain matrix

public:
	//double SpEst; // Estimation
	CMatrix* PeriodicAmp; // [N x 1]: Amplitude of vibration of each harmonic

public:
	CKalmanFilter(int N, double spindleSpeed, double samplingPeriod, double lamda, double covariance);
	~CKalmanFilter();
	void UpdateKalman(double newSpindleSpeed);
	double RunKalman(double measurement);
};

#endif
