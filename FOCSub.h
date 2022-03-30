#ifndef __FOCSUB_H__
#define __FOCSUB_H__

#include "main.h"

extern void Cordic(double ThetaE, double* SinTheta, double* CosTheta);
extern void InvPark(double Ud, double Uq, double SinTheta, double CosTheta, double* Ux, double* Uy);
extern void InvClarke(double Ux, double Uy, double* U1, double* U2, double* U3);
extern uint8_t GetSector(double U1, double U2, double U3);
extern void GetCCR(double U1, double U2, double U3, uint8_t Sector, double Udc, double* CCRa, double* CCRb, double* CCRc);
extern void Clarke(double Ia, double Ic, double* Ix, double* Iy);
extern void Park(double Ix, double Iy, double SinTheta, double CosTheta, double* Id, double* Iq);
extern void Spd_Timer(uint8_t* Spd_Tick);

#endif
