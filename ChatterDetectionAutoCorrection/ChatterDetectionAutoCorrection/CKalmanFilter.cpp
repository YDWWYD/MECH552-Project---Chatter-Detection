#include "CKalmanFilter.h"

/// <summary>
/// Constructor
/// </summary>
/// <param name="N_TPE">Number of harmonics</param>
/// <param name="spindleSpeed">spindle speed [rad/s]</param>
/// <param name="samplingPeriod">sampling period [s]</param>
/// <param name="lamda">lambda, tuned value</param>
/// <param name="covariance"> error covariance</param>
CKalmanFilter::CKalmanFilter(int N_TPE, double spindleSpeed, double samplingPeriod, double lamda, double covariance)
{
	Phi = new CMatrix(2 * N_TPE, 1);

	for (int i = 0; i <  N_TPE; i++)
	{
		Phi->Content[Phi->Index(2 * i, 0)] = cos((i + 1)*spindleSpeed*samplingPeriod);
		Phi->Content[Phi->Index(2 * i + 1, 0)] = sin((i + 1)*spindleSpeed*samplingPeriod);
	}

	N = N_TPE;
	R = covariance;
	SamplingPeriod = samplingPeriod;
	Q = lamda*R;
	q = new CMatrix(2 * N, 1);
	P = new CMatrix(2 * N, 2 * N);
	
	for (int i = 0; i < P->Row; i++)
	{
		P->Content[P->Index(i, i)] = 10;
	}

	qPrior = new CMatrix(2 * N, 1);
	PPrior = new CMatrix(2 * N, 2 * N);
	K = new CMatrix(2 * N, 1);
	PeriodicAmp = new CMatrix(N, 1);
}

CKalmanFilter::~CKalmanFilter()
{
	delete Phi;
	delete q;
	delete P;
	delete qPrior;
	delete PPrior;
	delete K;
	delete PeriodicAmp;
}

/// <summary>
/// Update the Kalman filter when spindle speed changes. Only phi matrix is affacted by spindle speed so only phi is updated.
/// </summary>
/// <param name="newSpindleSpeed"> new spindle speed [rad/s]</param>
void CKalmanFilter::UpdateKalman(double newSpindleSpeed)
{
	for (int i = 0; i < N; i++)
	{
		Phi->Content[Phi->Index(2 * i, 0)] = cos((i + 1)*newSpindleSpeed*SamplingPeriod);
		Phi->Content[Phi->Index(2 * i + 1, 0)] = sin((i + 1)*newSpindleSpeed*SamplingPeriod);
	}
}

/// <summary>
/// Run Kalman algorithm. Each kalman equations is explicitly expanded so that patterns are found to save storage and avoid unnecessary
/// calculations, for example, zero matrix multiplication, inverse of a constant etc.
/// </summary>
/// <param name="measurement">direct measurement signal</param>
/// <returns>estimation of periodic vibration</returns>
double CKalmanFilter::RunKalman(double measurement)
{
	// 1: priori estimation of state q_k_priori = Phi*q_k-1
	for (int i = 0; i < N; i++)
	{
		int index1 = 2 * i;
		int index2 = 2 * i + 1;
		qPrior->Content[index1] = Phi->Content[index1] * q->Content[index1] - Phi->Content[index2] * q->Content[index2];
		qPrior->Content[index2] = Phi->Content[index2] * q->Content[index1] + Phi->Content[index1] * q->Content[index2];
	}

	// 2: priori estimation error: P_k_prior = Phi*P_k-1*Phi' + Q
	//Phi*P_k - 1 * Phi'
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			int indexPHI11 = 2 * i; //Phi->Index(2 * i, 0)
			int indexPHI21 = 2 * j; //Phi->Index(2 * j, 0)
			int indexPHI12 = 2 * i + 1; //Phi->Index(2 * i + 1, 0)
			int indexPHI22 = 2 * j + 1; //Phi->Index(2 * j + 1, 0)

			int indexP11 = 2 * i * PPrior->Column + 2 * j; //PPrior->Index(2 * i, 2 * j)
			int indexP21 = (2 * i + 1) * PPrior->Column + 2 * j; //PPrior->Index(2 * i + 1, 2 * j)
			int indexP12 = 2 * i * PPrior->Column + (2 * j + 1); //PPrior->Index(2 * i, 2 * j + 1)
			int indexP22 = (2 * i + 1) * PPrior->Column + (2 * j + 1); //PPrior->Index(2 * i + 1, 2 * j + 1)

			PPrior->Content[indexP11] = Phi->Content[indexPHI21] * (P->Content[indexP11] * Phi->Content[indexPHI11] - P->Content[indexP21] * Phi->Content[indexPHI12]) -
				                        Phi->Content[indexPHI22] * (P->Content[indexP12] * Phi->Content[indexPHI11] - P->Content[indexP22] * Phi->Content[indexPHI12]);

			PPrior->Content[indexP12] = Phi->Content[indexPHI22] * (P->Content[indexP11] * Phi->Content[indexPHI11] - P->Content[indexP21] * Phi->Content[indexPHI12]) +
										Phi->Content[indexPHI21] * (P->Content[indexP12] * Phi->Content[indexPHI11] - P->Content[indexP22] * Phi->Content[indexPHI12]);

			PPrior->Content[indexP21] = Phi->Content[indexPHI21] * (P->Content[indexP11] * Phi->Content[indexPHI12] + P->Content[indexP21] * Phi->Content[indexPHI11]) -
										Phi->Content[indexPHI22] * (P->Content[indexP12] * Phi->Content[indexPHI12] + P->Content[indexP22] * Phi->Content[indexPHI11]);

			PPrior->Content[indexP22] = Phi->Content[indexPHI22] * (P->Content[indexP11] * Phi->Content[indexPHI12] + P->Content[indexP21] * Phi->Content[indexPHI11]) +
										Phi->Content[indexPHI21] * (P->Content[indexP12] * Phi->Content[indexPHI12] + P->Content[indexP22] * Phi->Content[indexPHI11]);
		}
	}

	// Phi*P_K-1*Phi' + Q
	for (int i = 0; i < 2 * N; i++)
	{
		PPrior->Content[i*PPrior->Column + i] += Q;
	}

	// 3. Kalman gain matrix: K_k = P_k_prior*H'*(H*P_k_prior*H'+R)^(-1)
	// Calculate inverse
	double invSum = 0;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			invSum += PPrior->Content[2 * i * PPrior->Column + 2 * j];
		}
	}
	double inverse = 1.0 / (invSum + R);

	for (int i = 0; i < 2 * N; i++)
	{
		double tempSum = 0;
		for (int j = 0; j < N; j++)
			tempSum += PPrior->Content[i*PPrior->Column + 2 * j];
		K->Content[i] = inverse*tempSum;
	}

	// 4. q_k = q_k_priori + K_k(s_k-H*q_k-1_priori)
	double sMinusHqk = measurement;
	for (int i = 0; i < N; i++)
	{
		sMinusHqk -= qPrior->Content[2 * i];
	}

	for (int i = 0; i < 2 * N; i++)
	{
		q->Content[i] = qPrior->Content[i] + K->Content[i] * sMinusHqk;
	}

	// 5. P_k = (I-K_k*H)P_k_priori
	for (int j = 0; j < 2 * N; j++) // each row second
	{
		double tempSum = 0;
		for (int k = 0; k < N; k++)
			tempSum += PPrior->Content[2 * k*PPrior->Column + j];

		for (int i = 0; i < 2 * N; i++) // each column  first
		{
			int indexPij = i*P->Column + j;
			P->Content[indexPij] = -K->Content[i] * tempSum + PPrior->Content[indexPij];
		}
	}

	// Estimation output Sp_est = H*q_k
	double spEst = 0;
	for (int i = 0; i < N; i++)
	{
		spEst += q->Content[2 * i];
	}

	// periodic vibration amplitude = sqrt(q_k_2n-1^2 + q_k_2n^2)
	for (int i = 0; i < N; i++)
	{
		PeriodicAmp->Content[i] = pow(pow(q->Content[2 * i], 2) + pow(q->Content[2 * i + 1], 2), 0.5);
	}

	return spEst;
}