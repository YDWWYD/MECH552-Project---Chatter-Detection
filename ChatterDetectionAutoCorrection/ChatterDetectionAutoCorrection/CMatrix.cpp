#include "CMatrix.h"

CMatrix::CMatrix(void)
{
	Row = 0;
	Column = 0;

	Content = new double[Row * Column];

	for (int i = 0; i < Row*Column; i++)
	{
		Content[i] = 0;
	}
}

CMatrix::CMatrix(int row, int column)
{
	Row = row;
	Column = column;

	Content = new double[Row * Column];

	for (int i = 0; i < Row*Column; i++)
	{
		Content[i] = 0;
	}
}

CMatrix::CMatrix(CMatrix& sourceMatrix)
{
	Row = sourceMatrix.Row;
	Column = sourceMatrix.Column;
	
	int size = Row*Column;

	Content = new double[size];
	for (int i = 0; i < size; i++)
	{
		Content[i] = sourceMatrix.Content[i];
	}
}

CMatrix::~CMatrix()
{
	//cout << "Destructor being called---" << endl;
	delete[] Content;
}

int CMatrix::Index(int rowNum, int columnNum)
{
	return rowNum*Column + columnNum;
}

void CMatrix::PrintMatrix(void)
{
	for (int i = 0; i < Row; i++)
	{
		for (int j = 0; j < Column; j++)
			cout << setprecision(16) << Content[Index(i, j)] << "   ";
			//printf("%.15f  ", matrix.content[i][j]);
		cout << "\n";
	}
	cout << "\n";
}

CMatrix* CMatrix::ExtractRow(CMatrix* SourceMatrix, int row)
{
	if (this->Row != 1 || this->Column != SourceMatrix->Column)
	{
		cout << "Invalid matrix sizes!\n";
		return NULL;
	}

	if (row > SourceMatrix->Row)
	{
		cout << "Row number exceeds max row!\n";
		return NULL;
	}

	for (int i = 0; i < SourceMatrix->Column; i++)
	{
		this->Content[i] = SourceMatrix->Content[row*SourceMatrix->Column + i];
	}

	return this;
}

CMatrix* CMatrix::ExtractRowToColumn(CMatrix* SourceMatrix, int row)
{
	if (this->Column != 1 || this->Row != SourceMatrix->Column)
	{
		cout << "Invalid matrix sizes!\n";
		return NULL;
	}

	if (row > SourceMatrix->Row)
	{
		cout << "Row number exceeds max row!\n";
		return NULL;
	}

	for (int i = 0; i < SourceMatrix->Column; i++)
	{
		this->Content[i] = SourceMatrix->Content[row*SourceMatrix->Column + i];
	}

	return this;
}

CMatrix CMatrix:: operator*(const CMatrix& M1)
{
	CMatrix result(Row, M1.Column);	
	if (Column != M1.Row)
	{
		cout << "Dimension Invalid!" << endl;
		exit(1);
	}

	else
	{	
		for (int i = 0; i < result.Row; i++)
		{
			for (int j = 0; j < result.Column; j++)
			{
				//result[i, j] = 0; //Default values are 0
				
				for (int k = 0; k < Column; k++)
				{
					int index = i*result.Column + j;
					//cout << Content[i*Column + k] << endl;
					result.Content[index] = result.Content[index] + Content[i*Column + k] * M1.Content[k*M1.Column + j];
				}
			}
		}
	}

	return result;
}

CMatrix CMatrix:: operator+(const CMatrix& M1)
{
	CMatrix result(Row, Column);
	if (Column != M1.Column || Row != M1.Row)
	{
		cout << "Dimension Invalid!" << endl;
		exit(1);
	}

	else
	{
		int size = Row*Column;
		for (int i = 0; i < size; i++)
		{
			result.Content[i] = Content[i] + M1.Content[i];
		}
	}

	return result;
}

CMatrix CMatrix:: operator-(const CMatrix& M1)
{
	CMatrix result(Row, Column);
	if (Column != M1.Column || Row != M1.Row)
	{
		cout << "Dimension Invalid!" << endl;
		exit(1);
	}

	else
	{
		int size = Row*Column;
		for (int i = 0; i < size; i++)
		{
			result.Content[i] = Content[i] - M1.Content[i];
		}
	}

	return result;
}

CMatrix& CMatrix:: operator=(const CMatrix& M1)
{
	if (&M1 == this)
		return *this;

	delete[] Content;
	Row = M1.Row;
	Column = M1.Column;

	int size = Row*Column;
	Content = new double[size];
	for (int i = 0; i < size; i++)
	{
		Content[i] = M1.Content[i];
	}

	return *this;
}

//void CMatrix::operator=(const CMatrix &other)   // overloading operator =
//{
//	if (Content != other.Content && Column == other.Column && Row == other.Row)
//	{
//		int size = Row*Column;
//		for (int i = 0; i < size; i++)
//		{
//			Content[size] = other.Content[size];
//		}
//	}
//}

double CMatrix::matrixMax(void)
{
	double max = this->Content[0];

	for (int i = 0; i < this->Row; i++)
	{
		for (int j = 0; j < this->Column; j++)
		{
			int index = i*this->Column + j;
			if (this->Content[index] > max)
				max = Content[index];
		}
	}

	return max;
}

void CMatrix::ResetToZero(void)
{
	int size = Row*Column;
	for (int i = 0; i < size; i++)
	{
		Content[i] = 0;
	}
}