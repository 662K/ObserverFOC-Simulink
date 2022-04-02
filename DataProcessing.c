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

void LPF(double* Uo, double Ui, double Fs, double Wc){
    *Uo = *Uo + Wc / Fs * (Ui - *Uo);
}

double SMOSwitchFunction1(double E, double Error){
    double SF_Out;

    if(Error > E)
        SF_Out = 1.0;
    else if(Error < -E)
        SF_Out = -1.0;
    else
        SF_Out = 1.0 / E * Error;

    return SF_Out;
}

void SlidingModeObserver(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    MRT_Inf->EMF = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->Ex = -MRT_Inf->EMF * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF * MRT_Inf->CosTheta;

    SMO->Vx = SMO->h1 * ((SMO->Ix > MRT_Inf->Ix)?(1):(-1));
    SMO->Vy = SMO->h1 * ((SMO->Iy > MRT_Inf->Iy)?(1):(-1));

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Ix + MRT_Inf->Ux - SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Iy + MRT_Inf->Uy - SMO->Vy) / MotorParameter->Ls;

    LPF(&(SMO->Ex), SMO->Vx, CtrlCom->CurFs, 250 * 2 * PI);
    LPF(&(SMO->Ey), SMO->Vy, CtrlCom->CurFs, 250 * 2 * PI);

    Cordic(SMO->ThetaE, &(SMO->SinTheta), &(SMO->CosTheta));

    SMO->SpdE_PI.Kp = 2 * SMO->Theta_PLL_zeta * SMO->Theta_PLL_wn / MotorParameter->Flux / SMO->Theta_PLL_we;
    SMO->SpdE_PI.Ki = SMO->Theta_PLL_wn * SMO->Theta_PLL_wn / MotorParameter->Flux / SMO->Theta_PLL_we * CtrlCom->CurTs;
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

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Ix + MRT_Inf->Ux + 0 * SMO->Ex - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (-MotorParameter->Rs * SMO->Iy + MRT_Inf->Uy + 0 * SMO->Ey - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    LPF(&(SMO->Ex), SMO->Vx, CtrlCom->CurFs, SMO->EMF_LPF_wc);
    LPF(&(SMO->Ey), SMO->Vy, CtrlCom->CurFs, SMO->EMF_LPF_wc);

    Cordic(SMO->ThetaE, &(SMO->SinTheta), &(SMO->CosTheta));

    if(((-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta) < 2) && ((-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta) > -2)){
        SMO->SpdE_PI.Kp = 2 * SMO->Theta_PLL_zeta * SMO->Theta_PLL_wn / MotorParameter->Flux / SMO->Theta_PLL_we;
        SMO->SpdE_PI.Ki = SMO->Theta_PLL_wn * SMO->Theta_PLL_wn / MotorParameter->Flux / SMO->Theta_PLL_we * CtrlCom->CurTs;
        SMO->SpdE_PI.Max = 2 * PI * 200 * MotorParameter->Np;
        SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta);
    }
    else{
        SMO->SpdE_PI.Kp = 2 * SMO->Theta_PLL_zeta * SMO->Theta_PLL_wn;
        SMO->SpdE_PI.Ki = SMO->Theta_PLL_wn * SMO->Theta_PLL_wn * CtrlCom->CurTs;
        SMO->SpdE_PI.Max = 2 * PI * 200 * MotorParameter->Np;
        SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / (-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta) *2;
    }

    double SpdE = PID_Control_Err(&(SMO->SpdE_PI), SMO->de);
    LPF(&(SMO->SpdE), SpdE, CtrlCom->CurFs, SMO->Spd_LPF_wc);
    SMO->ThetaE = SMO->ThetaE + SpdE * CtrlCom->CurTs;
}

void SlidingModeObserver3(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    MRT_Inf->EMF = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->Ex = -MRT_Inf->EMF * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - SMO->Ex - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - SMO->Ey - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    SMO->Ex = SMO->Ex + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey + SMO->h2 * SMO->Vx / MotorParameter->Ls);
    SMO->Ey = SMO->Ey + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex + SMO->h2 * SMO->Vy / MotorParameter->Ls);

    Cordic(SMO->ThetaE, &(SMO->SinTheta), &(SMO->CosTheta));

    SMO->de = -SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta;

    // if((SMO->de < SMO->Switch_Spd * MotorParameter->Np * MotorParameter->Flux) && (SMO->de > -SMO->Switch_Spd * MotorParameter->Np * MotorParameter->Flux)){
    //     SMO->SpdE_PI.Kp = 2 * SMO->Theta_PLL_zeta * SMO->Theta_PLL_wn / MotorParameter->Flux / SMO->Theta_PLL_we;
    //     SMO->SpdE_PI.Ki = SMO->Theta_PLL_wn * SMO->Theta_PLL_wn / MotorParameter->Flux / SMO->Theta_PLL_we * CtrlCom->CurTs;
    //     SMO->SpdE_PI.Max = 2 * PI * 200 * MotorParameter->Np;
    //     SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta);
    // }
    {
        SMO->SpdE_PI.Kp = 2 * SMO->Theta_PLL_zeta * SMO->Theta_PLL_wn;
        SMO->SpdE_PI.Ki = SMO->Theta_PLL_wn * SMO->Theta_PLL_wn * CtrlCom->CurTs;
        SMO->SpdE_PI.Max = 2 * PI * 200 * MotorParameter->Np;
        SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / (-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta);
    }

    double SpdE = PID_Control_Err(&(SMO->SpdE_PI), SMO->de);
    LPF(&(SMO->SpdE), SpdE, CtrlCom->CurFs, SMO->Spd_LPF_wc);

    double ThetaE_temp = SMO->ThetaE + SpdE * CtrlCom->CurTs;
    if(ThetaE_temp < 0)
        ThetaE_temp += 2 * PI;
    SMO->ThetaE = fmod(ThetaE_temp, 2 * PI);
    SMO->de = (-SMO->Ex * SMO->SinTheta + SMO->Ey * SMO->CosTheta);
}
