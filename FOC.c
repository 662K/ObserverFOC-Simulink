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

void FOC(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, DataIO_str* DataIO, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, PI_str* ObserverD_PI, PI_str* ObserverQ_PI){
    Spd_Timer(&(DataIO->Spd_Tick));

    double ThetaE = GetThetaE(DataIO->Theta, MotorParameter->Np);
    
    GetSpd(DataIO->Theta, &DataIO->Theta_Pre, DataIO->Spd_Tick, &DataIO->PresentSpd, DataIO->SpdTs);
    Cordic(ThetaE, &DataIO->SinTheta, &DataIO->CosTheta);
    Clarke(DataIO->Ia, DataIO->Ic, &DataIO->Ix, &DataIO->Iy);
    Park(DataIO->Ix, DataIO->Iy, DataIO->SinTheta, DataIO->CosTheta, &DataIO->PresentId, &DataIO->PresentIq);
    
    if(DataIO->Mode == 2){
        if(DataIO->Spd_Tick == 0){
            DataIO->TargetId = 0;
            DataIO->TargetIq = PID_Control(Spd_PI, DataIO->TargetSpd, DataIO->PresentSpd);
        }
    }

    if((DataIO->Mode == 1) || (DataIO->Mode == 2)){
        DataIO->PresentUd = PID_Control(D_PI, DataIO->TargetId, MotorObserver->Id); //DataIO->PresentId
        DataIO->PresentUq = PID_Control(Q_PI, DataIO->TargetIq, MotorObserver->Iq); //DataIO->PresentIq
    }

    double Ud = DataIO->PresentUd + DataIO->TargetUd;
    double Uq = DataIO->PresentUq + DataIO->TargetUq;

    /**********观测电流补偿************/
    ObserverD_PI->Kp = 0.5;
    ObserverD_PI->Ki = ObserverD_PI->Kp * 2 * PI / DataIO->CurTs / 5 * DataIO->CurTs;
    ObserverD_PI->Max = 27;

    ObserverQ_PI->Kp = 0.5;
    ObserverQ_PI->Ki = ObserverQ_PI->Kp * 2 * PI / DataIO->CurTs / 5 * DataIO->CurTs;
    ObserverQ_PI->Max = 27;
    
    Ud = Ud + PID_Control(ObserverD_PI, DataIO->PresentId, MotorObserver->Id_delay);
    Uq = Uq + PID_Control(ObserverQ_PI, DataIO->PresentIq, MotorObserver->Iq_delay);

    /**********速度前馈解耦************/
    double SpeedE = DataIO->TargetSpd * MotorParameter->Np;
    double Flux = MotorParameter->Kt / MotorParameter->Np / 1.5;

    Ud = Ud - SpeedE * MotorParameter->Ls * MotorObserver->Iq;
    Uq = Uq + (Flux + MotorParameter->Ls * MotorObserver->Id) * SpeedE;

    /**********dq轴电流观测************/
    MotorObserver->Id_delay = MotorObserver->Id;
    MotorObserver->Iq_delay = MotorObserver->Iq;
    MotorObserver->Id = MotorObserver->Id + (DataIO->CurTs / MotorParameter->Ls) * (Ud - MotorParameter->Rs * MotorObserver->Id);
    MotorObserver->Iq = MotorObserver->Iq + (DataIO->CurTs / MotorParameter->Ls) * (Uq - MotorParameter->Rs * MotorObserver->Iq);

    InvPark(DataIO->PresentUd, DataIO->PresentUq, DataIO->SinTheta, DataIO->CosTheta, &DataIO->Ux, &DataIO->Uy);
    InvClarke(DataIO->Ux, DataIO->Uy, &DataIO->U1, &DataIO->U2, &DataIO->U3);
    DataIO->Sector = GetSector(DataIO->U1, DataIO->U2, DataIO->U3);
    GetCCR(DataIO->U1, DataIO->U2, DataIO->U3, DataIO->Sector, DataIO->Udc, &DataIO->CCRa, &DataIO->CCRb, &DataIO->CCRc);
}
