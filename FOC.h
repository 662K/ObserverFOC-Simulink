#ifndef __FOC_H__
#define __FOC_H__

#include <math.h>
#include <stdint.h>

typedef struct{
    double Kp;
    double Ki;
    double Max;
    double up;
    double ui;
    double Error;
    double Out_temp;
}PI_str;

typedef struct{
    uint8_t Spd_Tick;
    double Theta_Pre;
    
    double Spd;

    double Id;
    double Iq;

    double CurTs;
    double SpdTs;

    double Ud;
    double Uq;

    uint8_t Mode;
}ControlCommand_str;

typedef struct{
    double Ls;
    double Rs;
    double Kt;
    double J;
    uint8_t Np;
}MotorParameter_str;

typedef struct{
    double Theta;
    double Spd;

    double Udc;

    double SinTheta;
    double CosTheta; 

    double Ux;       
    double Uy;

    double U1;    
    double U2;   
    double U3;       

    uint8_t Sector;

    double CCRa;
    double CCRb; 
    double CCRc;

    double Ia;
    double Ic;

    double Ix;       
    double Iy;

    double Id;
    double Iq;

    double Ud;
    double Uq;
}MotorRealTimeInformation_str;

typedef struct{
    double Te;
    double TL;
    double Acc;
    double Spd;
    double Spd_Temp;
    double Spd_Bef;
    double Spd_Pre;
    double Theta;
    double Theta_Pre;
    PI_str Spd_PI;
}MotorObserver_str;

#define PI acos(-1)
#define TRUE 1
#define FALSE 0

extern double GetTheta(int32_t Theta);
extern double GetCur(int32_t Cur);
extern void FOC(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, MotorRealTimeInformation_str* MRT_Inf);

#endif
