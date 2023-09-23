#ifndef _Deal_ectrical_H_
#define _Deal_ectrical_H_

#include "common.h"
void GetADCValue(void);
void GuiYi(void);
void Island(void);
void Out_Of_Left(void);
void Out_Of_Right(void);
float KalmanFilter_Elect(float curr_elect_val,float last_elect_val);
float CBH(void);
extern int8 flag_CK;
extern int8 flag_HD;
extern int16 AD[7];//存放初值
extern uint8 ADS[4];
extern int G_AD[7];//归一化后值


extern int16 g_adValue[4];		
extern int16 g_adValueFilter[4];
#endif