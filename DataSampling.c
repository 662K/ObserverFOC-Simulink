#include "DataSampling.h"

double GetTheta(int32_t Theta){
    return 2 * PI * Theta / (1 << 20);
}

double GetCur(int32_t Cur){
    return 5.0 * (Cur - 2048) / (1 << 11);
}

double GetThetaE(double ThetaE, uint8_t Np){
    return fmod((ThetaE * (double)Np), (2.0 * PI));
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

void CtrlComFilter(double *Target, double CtrlCom, double TickAdd){
    if(*Target < CtrlCom){
        if(*Target + TickAdd > CtrlCom){
            *Target = CtrlCom;
        }
        else{
            *Target += TickAdd;
        }
    }
    else if(*Target > CtrlCom){
        if(*Target - TickAdd < CtrlCom){
            *Target = CtrlCom;
        }
        else{
            *Target -= TickAdd;
        }
    }          
}
