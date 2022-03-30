#ifndef __DATASAMPLING_H__
#define __DATASAMPLING_H__

#include "main.h"

extern double GetTheta(int32_t Theta);
extern double GetCur(int32_t Cur);
extern double GetThetaE(double ThetaE, uint8_t Np);
extern void GetSpd(double Theta, double* Theta_Pre, uint8_t Spd_Tick, double* Speed, double SpdTs);

#endif
