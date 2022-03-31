#ifndef __DATAPROCESSING_H__
#define __DATAPROCESSING_H__

#include "main.h"

extern double PID_Control(PI_str* pPI, double Target, double Present);
extern double ObsPID_Control(PI_str* pPI, double Target, double Present);
extern double PIMAX_Control(PI_str* pPI, double Target, double Present, double MaxUp, double MaxDown);
extern void LPF(double* Uo, double Ui, double Fs, double Fc);
extern void SlidingModeObserver(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO);

#endif
