#ifndef __DIRECTION_H__
#define __DIRECTION_H__

#include  "headfile.h"



/**********全局变量外部申明********/
extern float g_dirControl_P;
extern float g_dirControl_D;
extern float dir_P;
extern float dir_D;

extern float g_dirControl_Z;
extern float g_fDirectionError[2];
extern float g_fDirectionError_dot[2];
extern float g_fDirectionControlOut;
extern float T_Z;


/***********函数声明***********/

void DirectionControl(void);

#endif