#ifndef __CMeanFilter_
#define __CMeanFilter_

#include "CMatrix.h"
/// <summary>
/// Class of recursive mean filter
/// </summary>
class CMeanFilter
{
public:
	double Output; //scalar input
	CMatrix* MOutput; // Matrix input
	double NumDelay; //Must use double or explicitly cast to double in calculations to avoid converting to int
	int NumOfInput;

private:
	double PrevOutput; // scalar output
	CMatrix* MPrevOutput; // matrix output
	double Counter; // must use double type or cast to double to avoid being converted to int in RunMeanFilter()

public:
	CMeanFilter(int numOfInput, int numOfDelay);
	~CMeanFilter();
	CMatrix* RunMeanFilter(CMatrix* input);
	double RunMeanFilter(double input);
};
#endif
