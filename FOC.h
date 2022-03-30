#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"

extern double GetTheta(int32_t Theta);
extern double GetCur(int32_t Cur);
extern void FOC(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorObserver_str* MotorObserver, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO);

#endif
