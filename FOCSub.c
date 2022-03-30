#include "FOCSub.h"

void Cordic(double ThetaE, double* SinTheta, double* CosTheta){
    *SinTheta = sin(ThetaE);
    *CosTheta = cos(ThetaE);
}

void InvPark(double Ud, double Uq, double SinTheta, double CosTheta, double* Ux, double* Uy){
    *Ux = CosTheta * Ud - SinTheta * Uq;
    *Uy = SinTheta * Ud + CosTheta * Uq;
}

void InvClarke(double Ux, double Uy, double* U1, double* U2, double* U3){
    *U1 =  Uy;
    *U2 =  Ux * 0.866 - Uy * 0.5;
    *U3 = -Ux * 0.866 - Uy * 0.5;
}

uint8_t GetSector(double U1, double U2, double U3){
    uint8_t N = ((U3 >= 0) << 2) | ((U2 >= 0) << 1) | ((U1 >= 0) << 0);
    uint8_t Sector;

    switch (N){
    case 3: Sector = 1; break;
    case 1: Sector = 2; break;
    case 5: Sector = 3; break;
    case 4: Sector = 4; break;
    case 6: Sector = 5; break;
    case 2: Sector = 6; break;}

    return Sector;
}

void GetCCR(double U1, double U2, double U3, uint8_t Sector, double Udc, double* CCRa, double* CCRb, double* CCRc){
    double Tx, Ty;
    double Ta, Tb, Tc;
    double K = Udc / 1.732;
    switch(Sector){
    case 1: Tx =  U2 / K; Ty =  U1 / K; break;
    case 2: Tx = -U2 / K; Ty = -U3 / K; break;
    case 3: Tx =  U1 / K; Ty =  U3 / K; break;
    case 4: Tx = -U1 / K; Ty = -U2 / K; break;
    case 5: Tx =  U3 / K; Ty =  U2 / K; break;
    case 6: Tx = -U3 / K; Ty = -U1 / K; break;}

    Ta = (1 - Tx - Ty) / 2;
    Tb = (1 + Tx - Ty) / 2;
    Tc = (1 + Tx + Ty) / 2;

    switch(Sector){
    case 1: *CCRa = Ta;   *CCRb = Tb;   *CCRc = Tc;   break;
    case 2: *CCRa = Tb;   *CCRb = Ta;   *CCRc = Tc;   break;
    case 3: *CCRa = Tc;   *CCRb = Ta;   *CCRc = Tb;   break;
    case 4: *CCRa = Tc;   *CCRb = Tb;   *CCRc = Ta;   break;
    case 5: *CCRa = Tb;   *CCRb = Tc;   *CCRc = Ta;   break;
    case 6: *CCRa = Ta;   *CCRb = Tc;   *CCRc = Tb;   break;
    }
}

void Clarke(double Ia, double Ic, double* Ix, double* Iy){
    *Ix = Ia;
    *Iy = -((Ia + Ic *2)*0.57735);
}

void Park(double Ix, double Iy, double SinTheta, double CosTheta, double* Id, double* Iq){
    *Id = SinTheta * Iy + CosTheta * Ix;
    *Iq = CosTheta * Iy - SinTheta * Ix;
}

void Spd_Timer(uint8_t* Spd_Tick){
    if(*Spd_Tick < 9)
        *Spd_Tick += 1;
    else
        *Spd_Tick = 0;
}
