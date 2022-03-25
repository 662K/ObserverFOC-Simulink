#include "FOC.h"

double GetTheta(int32_t Theta){
    return 2 * PI * Theta / (1 << 20);
}

double GetCur(int32_t Cur){
    return 5.0 * (Cur - 2048) / (1 << 11);
}

double GetThetaE(double ThetaE, uint8_t Np){
    return fmod((ThetaE * (double)Np), (2.0 * PI));
}

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

void Spd_Timer(uint8_t* Spd_Tick){
    if(*Spd_Tick < 9)
        *Spd_Tick += 1;
    else
        *Spd_Tick = 0;
}

void GetSpd(double Theta, double* Theta_Pre, uint8_t Spd_Tick, double* Speed, double SpdTs){
    if(Spd_Tick == 0){
        double Speed_temp = Theta - *Theta_Pre;
        if(Speed_temp > 0.314)
            Speed_temp = Speed_temp - 2*PI;
        else if(Speed_temp < -0.314)
            Speed_temp = Speed_temp + 2*PI;
        else
            Speed_temp = Speed_temp;
        
        *Speed = Speed_temp / SpdTs;
        *Theta_Pre = Theta;
    }
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

void SpeedLoopOri_Mode2(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);
}

void SpeedLoopLuenbergerObeserver_Mode3(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, MotorRealTimeInformation_str* MRT_Inf){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;
    MotorObserver->Spd_PI.Ki *= CtrlCom->SpdTs;
    
    CtrlCom->Id = 0;
    CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MotorObserver->Spd);
    MotorObserver->Te = CtrlCom->Iq * MotorParameter->Kt;
    MotorObserver->Acc = (MotorObserver->Te + MotorObserver->TL) / MotorParameter->J;
    if(CtrlCom->Spd_Tick == 0){
        MotorObserver->Spd_Bef = MotorObserver->Spd_Temp;
        MotorObserver->TL = ObsPID_Control(&(MotorObserver->Spd_PI), MRT_Inf->Spd, MotorObserver->Spd_Bef);
    }
    if(CtrlCom->Spd_Tick == 5)
        MotorObserver->Spd_Temp = MotorObserver->Spd;
    MotorObserver->Spd = MotorObserver->Spd_Pre;
    MotorObserver->Spd_Pre = MotorObserver->Spd + MotorObserver->Acc * CtrlCom->CurTs;

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);
}

void LoadTorqueObeserver_Mode4(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, MotorRealTimeInformation_str* MRT_Inf){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;
    MotorObserver->Spd_PI.Ki *= CtrlCom->SpdTs;
    
    CtrlCom->Id = 0;
    CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MotorObserver->Spd);
    MotorObserver->Te = CtrlCom->Iq * MotorParameter->Kt;
    MotorObserver->Acc = (MotorObserver->Te) / MotorParameter->J;
    if(CtrlCom->Spd_Tick == 0){
        MotorObserver->Spd_Bef = MotorObserver->Spd_Temp;
        MotorObserver->TL = ObsPID_Control(&(MotorObserver->Spd_PI), MRT_Inf->Spd, MotorObserver->Spd_Bef);
    }
    if(CtrlCom->Spd_Tick == 5)
        MotorObserver->Spd_Temp = MotorObserver->Spd;
    MotorObserver->Spd = MotorObserver->Spd_Pre;
    MotorObserver->Spd_Pre = MotorObserver->Spd + MotorObserver->Acc * CtrlCom->CurTs;

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq - MotorObserver->TL / MotorParameter->Kt, MRT_Inf->Iq);
}

void FOC(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, MotorRealTimeInformation_str* MRT_Inf){
    Spd_Timer(&(CtrlCom->Spd_Tick));

    double ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs);
    Cordic(ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);
    
    if(CtrlCom->Mode == 2){
        SpeedLoopOri_Mode2(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf);
    }
    else if(CtrlCom->Mode == 3){
        SpeedLoopLuenbergerObeserver_Mode3(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MotorObserver, MRT_Inf);
    }
    else if(CtrlCom->Mode == 4){
        LoadTorqueObeserver_Mode4(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MotorObserver, MRT_Inf);
    }

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}
