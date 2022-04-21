#include "FOC.h"
#include "FOCSub.h"
#include "DataSampling.h"
#include "DataProcessing.h"

void SpeedLoopOri_Mode2(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void SpeedLoopLuenbergerObeserver_Mode3(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, MotorRealTimeInformation_str* MRT_Inf){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->CurTs;
    MotorObserver->Spd_PI.Ki *= CtrlCom->SpdTs;
    
    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

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

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void LoadTorqueObeserver_Mode4(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, MotorRealTimeInformation_str* MRT_Inf){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->CurTs;
    MotorObserver->Spd_PI.Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

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

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void SpdTLObserverWithVolDecoupling_Mode5(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, MotorRealTimeInformation_str* MRT_Inf){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->CurTs;
    MotorObserver->Spd_PI.Ki *= CtrlCom->SpdTs;

    MotorObserver->Theta = MRT_Inf->Theta + 2.5 * MotorObserver->Spd * CtrlCom->CurTs;
    if(MotorObserver->Theta > 2 * PI)
        MotorObserver->Theta -= 2 * PI;
    else if(MotorObserver->Theta < 0)
        MotorObserver->Theta += 2 * PI;

    MRT_Inf->ThetaE = GetThetaE(MotorObserver->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    CtrlCom->Id = 0;
    CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MotorObserver->Spd);

    MotorObserver->Te = CtrlCom->Iq * MotorParameter->Kt;
    MotorObserver->Acc = (MotorObserver->Te) / MotorParameter->J;
    if(CtrlCom->Spd_Tick == 0){
        MotorObserver->Spd_Bef = MotorObserver->Spd_Temp;
        MotorObserver->TL = ObsPID_Control(&(MotorObserver->Spd_PI), MRT_Inf->Spd, MotorObserver->Spd_Bef);
    }
    if(CtrlCom->Spd_Tick == 5){
        MotorObserver->Spd_Temp = MotorObserver->Spd;
    }
    
    MotorObserver->Spd = MotorObserver->Spd_Pre;
    MotorObserver->Spd_Pre = MotorObserver->Spd + MotorObserver->Acc * CtrlCom->CurTs;
    
    MRT_Inf->Ud_qCoupling = -MRT_Inf->Spd * MotorParameter->Np * MRT_Inf->Iq * MotorParameter->Ls;
    MRT_Inf->Uq_dCoupling =  MRT_Inf->Spd * MotorParameter->Np * MRT_Inf->Id * MotorParameter->Ls;

    MRT_Inf->EMF = CtrlCom->Spd * MotorParameter->Np * MotorParameter->Flux;

    MRT_Inf->Ud_ElectricalMaxUp   =  D_PI->Max - MRT_Inf->Ud_qCoupling;
    MRT_Inf->Ud_ElectricalMaxDown = -D_PI->Max - MRT_Inf->Ud_qCoupling;
    MRT_Inf->Uq_ElectricalMaxUp   =  Q_PI->Max - MRT_Inf->Uq_dCoupling - MRT_Inf->EMF;
    MRT_Inf->Uq_ElectricalMaxDown = -Q_PI->Max - MRT_Inf->Uq_dCoupling - MRT_Inf->EMF;

    float TargetId = CtrlCom->Id;
    float TargetIq = CtrlCom->Iq - MotorObserver->TL / MotorParameter->Kt;

    MRT_Inf->Ud_Electrical = PIMAX_Control(D_PI, TargetId, MRT_Inf->Id, MRT_Inf->Ud_ElectricalMaxUp, MRT_Inf->Ud_ElectricalMaxDown);
    MRT_Inf->Uq_Electrical = PIMAX_Control(Q_PI, TargetIq, MRT_Inf->Iq, MRT_Inf->Uq_ElectricalMaxUp, MRT_Inf->Uq_ElectricalMaxDown);

    MRT_Inf->Ud = MRT_Inf->Ud_Electrical + MRT_Inf->Ud_qCoupling;
    MRT_Inf->Uq = MRT_Inf->Uq_Electrical + MRT_Inf->Uq_dCoupling + MRT_Inf->EMF;

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void SlidingModeObserver_Mode6(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);

    SMO->EMF_Flag = (CtrlCom->Spd < 0);

    if(SMO->Flag == 0){
        if((MRT_Inf->Spd > SMO->Switch_Spd) || (MRT_Inf->Spd < -SMO->Switch_Spd)){
            SMO->Flag = 1;
        }
    }
    else if(SMO->Flag == 1){
        if((MRT_Inf->Spd < SMO->Switch_Spd * 0.1) && (MRT_Inf->Spd > -SMO->Switch_Spd * 0.1)){
            SMO->Flag = 0;
        }
    }

    // if((MRT_Inf->Spd > SMO->Switch_Spd * 0.1) || (MRT_Inf->Spd < -SMO->Switch_Spd * 0.1))
        SlidingModeObserver3(CtrlCom, MotorParameter, MRT_Inf, SMO);

    if(SMO->Flag == 0)
        Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);
    else
        Park(MRT_Inf->Ix, MRT_Inf->Iy, SMO->SinTheta, SMO->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    CtrlCom->Id = 0;

    if(SMO->Flag == 0){
        if(CtrlCom->Spd_Tick == 0){
            Spd_PI->Ki *= CtrlCom->SpdTs;
            CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
        }
    }
    else{
        Spd_PI->Ki *= CtrlCom->CurTs;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, SMO->SpdE / MotorParameter->Np);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    if(SMO->Flag == 0)
        InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    else
        InvPark(MRT_Inf->Ud, MRT_Inf->Uq, SMO->SinTheta, SMO->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);

    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void NewTest_Mode7(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    SlidingModeObserver(CtrlCom, MotorParameter, MRT_Inf, SMO);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void FOC(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    Spd_Timer(&(CtrlCom->Spd_Tick));
    
    if(CtrlCom->Mode == 2){
        SpeedLoopOri_Mode2(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf);
    }
    else if(CtrlCom->Mode == 3){
        SpeedLoopLuenbergerObeserver_Mode3(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MotorObserver, MRT_Inf);
    }
    else if(CtrlCom->Mode == 4){
        LoadTorqueObeserver_Mode4(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MotorObserver, MRT_Inf);
    }
    else if(CtrlCom->Mode == 5){
        SpdTLObserverWithVolDecoupling_Mode5(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MotorObserver, MRT_Inf);
    }
    else if(CtrlCom->Mode == 6){
        SlidingModeObserver_Mode6(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
    else if(CtrlCom->Mode == 7){
        NewTest_Mode7(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
}
