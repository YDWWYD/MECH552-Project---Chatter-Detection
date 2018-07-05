#ifndef __CEnergyMeanFilter_
#define __CEnergyMeanFilter_

#include "CMatrix.h"
//#include <cmath>
//#define PI acos(-1.0)

class CEnergyMeanFilter
{
public:
	double Output;
	CMatrix* MOutput;
	double NumDelay; //Must use double or explicitly cast to double in calculations to avoid converting to int
	int NumOfInput;

private:
	double PrevOutput;
	CMatrix* MPrevOutput;
	double Counter; // double type or cast to double to avoid being converted to int in RunMeanFilter()

public:
	CEnergyMeanFilter(int numOfInput, int numOfDelay);
	~CEnergyMeanFilter();
	CMatrix* RunMeanFilter(CMatrix* input);
	double RunMeanFilter(double input);
};
#endif
