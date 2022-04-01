#include "DataProcessing.h"

double PID_Control(PI_str* pPI, double Target, double Present){
    double Error = Target - Present;
    uint8_t ui_flag = !(((pPI->Out_temp > pPI->Max) || (pPI->Out_temp < -pPI->Max)) && (pPI->Out_temp * Error >= 0));
    
    pPI->up = pPI->Kp * Error;
    pPI->ui = pPI->ui + pPI->Ki * Error * ui_flag;
    
    pPI->Out_temp = pPI->up + pPI->ui;
    
    double PIout = 0;
    
    if(pPI->Out_temp > pPI->Max)
        PIout = pPI->Max;
    else if(pPI->Out_temp < -pPI->Max)
        PIout = -pPI->Max;
    else 
        PIout = pPI->Out_temp;
    
    return PIout;
}

double PID_Control_Err(PI_str* pPI, double Error){
    uint8_t ui_flag = !(((pPI->Out_temp > pPI->Max) || (pPI->Out_temp < -pPI->Max)) && (pPI->Out_temp * Error >= 0));
    
    pPI->up = pPI->Kp * Error;
    pPI->ui = pPI->ui + pPI->Ki * Error * ui_flag;
    
    pPI->Out_temp = pPI->up + pPI->ui;
    
    double PIout = 0;
    
    if(pPI->Out_temp > pPI->Max)
        PIout = pPI->Max;
    else if(pPI->Out_temp < -pPI->Max)
        PIout = -pPI->Max;
    else 
        PIout = pPI->Out_temp;
    
    return PIout;
}

double ObsPID_Control(PI_str* pPI, double Target, double Present){
    pPI->Error = Target-Present;

    uint8_t ui_flag = !(((pPI->Out_temp > pPI->Max) || (pPI->Out_temp < -pPI->Max)) && (pPI->Out_temp * pPI->Error >= 0));
    
    pPI->up = pPI->Kp * pPI->Error;
    pPI->ui = pPI->ui + pPI->Ki * pPI->Error * ui_flag;
    
    pPI->Out_temp = pPI->up + pPI->ui;
    
    double PIout = 0;
    
    if(pPI->Out_temp > pPI->Max)
        PIout = pPI->Max;
    else if(pPI->Out_temp < -pPI->Max)
        PIout = -pPI->Max;
    else 
        PIout = pPI->Out_temp;
    
    return PIout;
}

double PIMAX_Control(PI_str* pPI, double Target, double Present, double MaxUp, double MaxDown){
    double Error = Target - Present;
    uint8_t ui_flag = !(((pPI->Out_temp > MaxUp) || (pPI->Out_temp < MaxDown)) && (pPI->Out_temp * Error >= 0));
    
    pPI->up = pPI->Kp * Error;
    pPI->ui = pPI->ui + pPI->Ki * Error * ui_flag;
    
    pPI->Out_temp = pPI->up + pPI->ui;
    
    double PIout = 0;
    
    if(pPI->Out_temp > MaxUp)
        PIout = MaxUp;
    else if(pPI->Out_temp < MaxDown)
        PIout = MaxDown;
    else 
        PIout = pPI->Out_temp;
    
    return PIout;
}

void LPF(double* Uo, double Ui, double Fs, double Fc){
    *Uo = *Uo + Fc / Fs * (Ui - *Uo);
}

void SlidingModeObserver(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    MRT_Inf->EMF = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->Ex = -MRT_Inf->EMF * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF * MRT_Inf->CosTheta;

    SMO->h = 20;

    SMO->Vx = SMO->h * ((SMO->Ix > MRT_Inf->Ix)?(1):(-1));
    SMO->Vy = SMO->h * ((SMO->Iy > MRT_Inf->Iy)?(1):(-1));

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Ix + MRT_Inf->Ux - SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Iy + MRT_Inf->Uy - SMO->Vy) / MotorParameter->Ls;

    LPF(&(SMO->Ex), SMO->Vx, CtrlCom->CurFs, 250 * 2 * PI);
    LPF(&(SMO->Ey), SMO->Vy, CtrlCom->CurFs, 250 * 2 * PI);

    double wn = 200 * 2 * PI;
    double we = 250 * 2 * PI;
    double zeta = 1;

    Cordic(SMO->ThetaE, &(SMO->SinTheta), &(SMO->CosTheta));

    SMO->SpdE_PI.Kp = 2 * zeta * wn / MotorParameter->Flux / we;
    SMO->SpdE_PI.Ki = wn * wn / MotorParameter->Flux / we * CtrlCom->CurTs;
    SMO->SpdE_PI.Max = 2 * PI * 200 * MotorParameter->Np;

    SMO->de = -SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta;

    double SpdE = PID_Control(&(SMO->SpdE_PI), -SMO->Ex * SMO->CosTheta, SMO->Ey * SMO->SinTheta);
    LPF(&(SMO->SpdE), SpdE, CtrlCom->CurFs, 250 * 2 * PI);
    SMO->ThetaE = SMO->ThetaE + SMO->SpdE * CtrlCom->CurTs;
}

void SlidingModeObserver2(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    MRT_Inf->EMF = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->Ex = -MRT_Inf->EMF * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF * MRT_Inf->CosTheta;

    SMO->E0 = 2;
    SMO->h = 60;

    if((SMO->Ix - MRT_Inf->Ix) > SMO->E0)
        SMO->Vx = SMO->h;
    else if((SMO->Ix - MRT_Inf->Ix) < -SMO->E0)
        SMO->Vx = -SMO->h;
    else
        SMO->Vx = SMO->h / SMO->E0 * (SMO->Ix - MRT_Inf->Ix);

    if((SMO->Iy - MRT_Inf->Iy) > SMO->E0)
        SMO->Vy = SMO->h;
    else if((SMO->Iy - MRT_Inf->Iy) < -SMO->E0)
        SMO->Vy = -SMO->h;
    else
        SMO->Vy = SMO->h / SMO->E0 * (SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Ix + MRT_Inf->Ux + 0 * SMO->Ex - SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Iy + MRT_Inf->Uy + 0 * SMO->Ey - SMO->Vy) / MotorParameter->Ls;

    LPF(&(SMO->Ex), SMO->Vx, CtrlCom->CurFs, 1500 * 2 * PI);
    LPF(&(SMO->Ey), SMO->Vy, CtrlCom->CurFs, 1500 * 2 * PI);

    double wn = 500 * 2 * PI;
    double we = 250 * 2 * PI;
    double zeta = 1;

    Cordic(SMO->ThetaE, &(SMO->SinTheta), &(SMO->CosTheta));

    if(((-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta) < 2) && ((-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta) > -2)){
        SMO->SpdE_PI.Kp = 2 * zeta * wn / MotorParameter->Flux / we;
        SMO->SpdE_PI.Ki = wn * wn / MotorParameter->Flux / we * CtrlCom->CurTs;
        SMO->SpdE_PI.Max = 2 * PI * 200 * MotorParameter->Np;
        SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta);
    }
    else{
        SMO->SpdE_PI.Kp = 2 * zeta * wn;
        SMO->SpdE_PI.Ki = wn * wn * CtrlCom->CurTs;
        SMO->SpdE_PI.Max = 2 * PI * 200 * MotorParameter->Np;
        SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / (-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta) *2;
    }

    double SpdE = PID_Control_Err(&(SMO->SpdE_PI), SMO->de);
    LPF(&(SMO->SpdE), SpdE, CtrlCom->CurFs, 250 * 2 * PI);
    SMO->ThetaE = SMO->ThetaE + SpdE * CtrlCom->CurTs;
}
