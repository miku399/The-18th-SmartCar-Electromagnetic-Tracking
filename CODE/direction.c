#include "headfile.h"

float g_dirControl_P = 110;				//方向控制P  70
float g_dirControl_D = 180; 			//方向控制D    100
float dir_P = 0;
float dir_D = 0;


float g_dirControl_Z = 0.25;         //陀螺仪参数
float g_dirControl_A = 0;       //差比和差系数
float g_B = 0;
float g_C = 0;
float g_dirControl_L = 0;

float g_fDirectionError[2];				//方向偏差    （g_fDirectionError[0]为一对水平电感的差比和偏差）
																	//            （g_fDirectionError[1]为一对垂直电感的差比和偏差）
float g_fDirectionError_dot[2];		//方向偏差微分（g_fDirectionError_dot[0]为一对水平电感的差比和偏差微分）
																	//			      （g_fDirectionError_dot[1]为一对垂直电感的差比和偏差微分）
float g_fDirectionControlOut;			//方向控制输出

float T_Z = 0;

int16 GyroOffset_Zdata = 11;

uint8 huandao=0;
uint8 huandao_count=0;
//int16 ADFilter[4]={0};		//阶梯滤波的电感值（未使用）


/**
 * @file		方向控制
 *				一般情况下：用两水平电感的差比和作为偏差
 *				在环岛时中：用两垂直电感的差比和作为偏差
 *
 *									电感值对应变量
 *
 *				    AD[1]				    AD[2]						         AD[3]             AD[4]
 *				(水平左电感)		（垂直左电感）					（垂直右电感）		（水平右电感）
 *
 * @date		2022 by AceTaffy gzc
 */
static float g_fDirectionErrorTemp[2][5];



void DirectionControl(void)
{	
	g_dirControl_P = dir_P;
  g_dirControl_D = dir_D;
	
	icm20602_get_gyro();
	
//	g_adValueFilter[1] = (g_adValueFilter[1] < 10? 10:g_adValueFilter[1]);	//四个电感值限幅
//	g_adValueFilter[2] = (g_adValueFilter[2] < 10? 10:g_adValueFilter[2]);
//	g_adValueFilter[3] = (g_adValueFilter[3] < 10? 10:g_adValueFilter[3]);
//	g_adValueFilter[4] = (g_adValueFilter[4] < 10? 10:g_adValueFilter[4]);

	g_fDirectionError[0] = (float)KP*( sqrt(g_adValue[1]) - sqrt(g_adValue[4]) ) / ( sqrt(g_adValue[1]) + sqrt(g_adValue[4]) );//水平电感的差比和作为偏差
	
//	g_fDirectionError[0]=KalmanFilter_Elect(g_fDirectionError[0],g_fDirectionError[2]);//卡尔曼滤波
	
	g_fDirectionError[0] = (g_fDirectionError[0]>= 1? 1:g_fDirectionError[0]);	//偏差限幅
	g_fDirectionError[0] = (g_fDirectionError[0]<=-1?-1:g_fDirectionError[0]);
	
//	g_fDirectionError[1] = (float)(g_adValueFilter[2] - g_adValueFilter[3])/(g_adValueFilter[2] + g_adValueFilter[3]);//垂直电感的差比和作为偏差
//	g_fDirectionError[1] = (g_fDirectionError[1]>= 1? 1:g_fDirectionError[1]);	//偏差限幅
//	g_fDirectionError[1] = (g_fDirectionError[1]<=-1?-1:g_fDirectionError[1]);	
	  
//	g_fDirectionErrorTemp[0][4] = g_fDirectionErrorTemp[0][3];
	g_fDirectionErrorTemp[0][3] = g_fDirectionErrorTemp[0][2];
	g_fDirectionErrorTemp[0][2] = g_fDirectionErrorTemp[0][1];
	g_fDirectionErrorTemp[0][1] = g_fDirectionErrorTemp[0][0];
	g_fDirectionErrorTemp[0][0] = g_fDirectionError[0];
	g_fDirectionError_dot[0] = 5*(float)(g_fDirectionErrorTemp[0][0]-g_fDirectionErrorTemp[0][3]);    //水平电感的偏差微分
	g_fDirectionError_dot[0] = (g_fDirectionError_dot[0]> 1.0? 1.0:g_fDirectionError_dot[0]);  //偏差微分限幅
	g_fDirectionError_dot[0] = (g_fDirectionError_dot[0]<-1.0?-1.0:g_fDirectionError_dot[0]);

//	g_fDirectionErrorTemp[1][4] = g_fDirectionErrorTemp[1][3];
//	g_fDirectionErrorTemp[1][3] = g_fDirectionErrorTemp[1][2];
//	g_fDirectionErrorTemp[1][2] = g_fDirectionErrorTemp[1][1];
//	g_fDirectionErrorTemp[1][1] = g_fDirectionErrorTemp[1][0];
//	g_fDirectionErrorTemp[1][0] = g_fDirectionError[1];
//	g_fDirectionError_dot[1] = 5*(g_fDirectionErrorTemp[1][0]-g_fDirectionErrorTemp[1][3]);    //垂直电感的偏差微分
//	g_fDirectionError_dot[1] = (g_fDirectionError_dot[1]> 0.7? 0.7:g_fDirectionError_dot[1]);  //偏差微分限幅
//	g_fDirectionError_dot[1] = (g_fDirectionError_dot[1]<-0.7?-0.7:g_fDirectionError_dot[1]);
	
////	g_fDirectionErrorTemp[0][3] = g_fDirectionErrorTemp[0][2];
////	g_fDirectionErrorTemp[0][2] = g_fDirectionErrorTemp[0][1];
////	g_fDirectionErrorTemp[0][1] = g_fDirectionErrorTemp[0][0];
////	g_fDirectionErrorTemp[0][0] = g_fDirectionError[0];
////	
////	g_fDirectionError_dot[0] = (g_fDirectionErrorTemp[0][0]-g_fDirectionErrorTemp[0][1])-2
////	(g_fDirectionErrorTemp[0][1]-g_fDirectionErrorTemp[0][2])+(g_fDirectionErrorTemp[0][2]-g_fDirectionErrorTemp[0][3]);    //水平电感的偏差微分

////	g_fDirectionError_dot[0] = (g_fDirectionError_dot[0]> 0.7? 0.7:g_fDirectionError_dot[0]);  //偏差微分限幅
////	g_fDirectionError_dot[0] = (g_fDirectionError_dot[0]<-0.7?-0.7:g_fDirectionError_dot[0]);
	

//	g_fDirectionErrorTemp[1][3] = g_fDirectionErrorTemp[1][2];
//	g_fDirectionErrorTemp[1][2] = g_fDirectionErrorTemp[1][1];
//	g_fDirectionErrorTemp[1][1] = g_fDirectionErrorTemp[1][0];
//	g_fDirectionErrorTemp[1][0] = g_fDirectionError[1];
//	
//  g_fDirectionError_dot[1] = (g_fDirectionErrorTemp[1][0]-g_fDirectionErrorTemp[1][1])-2*
// (g_fDirectionErrorTemp[1][1]-g_fDirectionErrorTemp[1][2])+(g_fDirectionErrorTemp[1][2]-g_fDirectionErrorTemp[1][3]);    //垂直电感的偏差微分

//	g_fDirectionError_dot[1] = (g_fDirectionError_dot[1]> 0.7? 0.7:g_fDirectionError_dot[1]);  //偏差微分限幅
//	g_fDirectionError_dot[1] = (g_fDirectionError_dot[1]<-0.7?-0.7:g_fDirectionError_dot[1]);
   	T_Z = (icm20602_gyro_z - 11)/500;
	
	//方向算法（位置式PD）
//	g_fDirectionControlOut = g_fDirectionError[0]*g_dirControl_P +(g_fDirectionError_dot[0])*g_dirControl_D - g_dirControl_G*G_Z;
	g_fDirectionControlOut = g_fDirectionError[0]*g_dirControl_P +(g_fDirectionError_dot[0])*g_dirControl_D;
	
//	g_fDirectionControlOut = g_fDirectionControlOut > 60 ? 60 :g_fDirectionControlOut;
//	g_fDirectionControlOut = g_fDirectionControlOut < -60 ? -60:g_fDirectionControlOut;

g_fDirectionError[2]=g_fDirectionError[0];    //更新差比和
		
}