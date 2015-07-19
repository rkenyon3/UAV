#include "util.h"

uint16_t calcCheckSum(char * data, uint8_t DataLength)
{
	uint16_t sum = 0;
	uint8_t count = 0;
	while (count < DataLength)
	{
		sum += *(data + count);
		count++;
	}

	return sum;
}

int delaymsToTicks(unsigned int NumberOfMilliseconds)
{
	float seconds = NumberOfMilliseconds / 1000.0;
	float sysTickTime = 1.0/CFG_SYSTICK_FREQ;
	int ticks = (int)(seconds / sysTickTime);
	return ticks;
}

void MultiplyMatrices(float Matrix1[3][3], float Matrix2[3][3], float OutMatrix[3][3])
{
	// Note: Assume that the first index is the row index.
	int row, col, i;

	for(row = 0; row < 3; row++)
	{
		for(col = 0; col < 3; col++)
		{
			float elementTotal = 0;
			for(i = 0; i < 3; i++)
			{
				elementTotal += Matrix1[row][i] * Matrix2[i][col];
			}
			OutMatrix[row][col] = elementTotal;
		}
	}
}

void MultiplyMatrixVector(float Matrix[3][3], float Vector[3], float OutVector[3])
{
	// Note: Assume that the first index is the row index.
	int row, i;

	for(row = 0; row < 3; row++)
	{
		float elementTotal = 0;
		for(i = 0; i < 3; i++)
		{
			elementTotal += Matrix[row][i] * Vector[i];
		}
		OutVector[row] = elementTotal;
	}
}

void AddVectors(float VectorA[3], float VectorB[3], float OutVector[3])
{
	int i;
	for(i = 0; i < 3; i++)
	{
		OutVector[i] = VectorA[i] + VectorB[i];
	}
}

int mapi(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
long mapl(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
