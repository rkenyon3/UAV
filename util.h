#ifndef UTIL_H
#define UTIL_H

#include "stm32f0xx.h"
#include "OsConfig.h"

int delaymsToTicks(unsigned int NumberOfMilliseconds);

// Calculates checksum for data packets
uint16_t calcCheckSum(char * data, uint8_t DataLength);

/* Matrix multiplication functions. Assumes a 3x3 matrix
 * or a 3x1 vector.
 */
void MultiplyMatrices(float Matrix1[3][3], float Matrix2[3][3], float OutMatrix[3][3]);
void MultiplyMatrixVector(float Matrix[3][3], float Vector[3], float OutVector[3]);
void AddVectors(float VectorA[3], float VectorB[3], float OutVector[3]);

/* Mapping functions. Map a number from one range to another.
 * Shamelessly ripped off from the arduino website:
 * http://arduino.cc/en/Reference/Map */
int mapi(int x, int in_min, int in_max, int out_min, int out_max);
long mapl(long x, long in_min, long in_max, long out_min, long out_max);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);
double mapd(double x, double in_min, double in_max, double out_min, double out_max);

#endif
