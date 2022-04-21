#ifndef __MAIN_H__
#define __MAIN_H__

#include <math.h>
#include <stdint.h>

typedef struct{
    float Kp;
    float Ki;
    float Max;
    float up;
    float ui;
    float Error;
    float Out_temp;
}PI_str;

typedef struct{
    uint8_t Spd_Tick;
    float Theta_Pre;
    
    float Spd;

    float Id;
    float Iq;

    float CurTs;
    float SpdTs;
    float CurFs;
    float SpdFs;

    float Ud;
    float Uq;

    uint8_t Mode;
}ControlCommand_str;

typedef struct{
    float Ls;
    float Rs;
    float Kt;
    float J;
    float Flux;
    uint8_t Np;
}MotorParameter_str;

typedef struct{
    float Theta;
    float Spd;

    float Udc;

    float SinTheta;
    float CosTheta; 

    float Ux;       
    float Uy;

    float U1;    
    float U2;   
    float U3;       

    uint8_t Sector;

    float CCRa;
    float CCRb; 
    float CCRc;

    float Ia;
    float Ic;

    float Ix;       
    float Iy;

    float Id;
    float Iq;

    float Ud;
    float Uq;

    float EMF;

    float Ud_qCoupling;
    float Uq_dCoupling;

    float Ud_Electrical;
    float Uq_Electrical;

    float Ud_ElectricalMaxUp;
    float Ud_ElectricalMaxDown;
    float Uq_ElectricalMaxUp;
    float Uq_ElectricalMaxDown;

    float Ex;
    float Ey;

    float ThetaE;
}MotorRealTimeInformation_str;

typedef struct{
    float Te;
    float TL;
    float Acc;
    float Spd;
    float Spd_Temp;
    float Spd_Bef;
    float Spd_Pre;
    float Theta;
    float Theta_Pre;
    PI_str Spd_PI;
}MotorObserver_str;

typedef struct{
    float Ix_Bef;
    float Iy_Bef;
    float Ex;
    float Ey;
    float Ix;
    float Iy;
    float Vx;
    float Vy;
    float h1;
    float h2;
    float de;
    PI_str SpdE_PI;
    float SpdE;
    float ThetaE;
    float SinTheta;
    float CosTheta;
    float Flag;
    float E1;
    float E2;
    float EMF_LPF_wc;
    float Theta_PLL_wn;
    float Theta_PLL_we;
    float Theta_PLL_zeta;
    float Spd_LPF_wc;
    float Switch_Spd;
    float ThetaE2;
    float EMF_Flag;
}SlidingModeObserver_str;

#define PI acos(-1)
#define TRUE 1
#define FALSE 0

#endif

