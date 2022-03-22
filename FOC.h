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
    double Target;
    double Present;
    double Out_temp;
}PI_str;

typedef struct{
    uint8_t Spd_Tick;
    double Theta_Pre;
    double PresentSpd;
    double TargetSpd;

    double Theta;

    double SinTheta;
    double CosTheta; 

    double TargetId;
    double TargetIq;

    double Ux;       
    double Uy;

    double U1;    
    double U2;   
    double U3;       

    uint8_t Sector;

    double Udc;
    double CurTs;
    double SpdTs;

    double CCRa;
    double CCRb; 
    double CCRc;

    double Ia;
    double Ic;

    double Ix;       
    double Iy;

    double PresentId;
    double PresentIq;

    double PresentUd;
    double PresentUq;

    double TargetUd;
    double TargetUq;

    uint8_t Mode;
}DataIO_str;

typedef struct{
    double Ls;
    double Rs;
    double Kt;
    double J;
    uint8_t Np;
}MotorParameter_str;

typedef struct{
    double Id;
    double Iq;
    double Id_delay;
    double Iq_delay;
}MotorObserver_str;

#define PI acos(-1)

extern double GetTheta(int32_t Theta);
extern double GetCur(int32_t Cur);
extern void FOC(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, DataIO_str* DataIO, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, PI_str* ObserverD_PI, PI_str* ObserverQ_PI);

#endif
