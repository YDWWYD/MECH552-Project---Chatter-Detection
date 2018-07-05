#ifndef __CMeanFilter_
#define __CMeanFilter_

#include "CMatrix.h"
//#include <cmath>
//#define PI acos(-1.0)

class CMeanFilter
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
	CMeanFilter(int numOfInput, int numOfDelay);
	~CMeanFilter();
	CMatrix* RunMeanFilter(CMatrix* input);
	double RunMeanFilter(double input);
};
#endif
