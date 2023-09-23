#include "headfile.h"

float g_dirControl_P = 110;				//�������P  70
float g_dirControl_D = 180; 			//�������D    100
float dir_P = 0;
float dir_D = 0;


float g_dirControl_Z = 0.25;         //�����ǲ���
float g_dirControl_A = 0;       //��ȺͲ�ϵ��
float g_B = 0;
float g_C = 0;
float g_dirControl_L = 0;

float g_fDirectionError[2];				//����ƫ��    ��g_fDirectionError[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ�
																	//            ��g_fDirectionError[1]Ϊһ�Դ�ֱ��еĲ�Ⱥ�ƫ�
float g_fDirectionError_dot[2];		//����ƫ��΢�֣�g_fDirectionError_dot[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ��΢�֣�
																	//			      ��g_fDirectionError_dot[1]Ϊһ�Դ�ֱ��еĲ�Ⱥ�ƫ��΢�֣�
float g_fDirectionControlOut;			//����������

float T_Z = 0;

int16 GyroOffset_Zdata = 11;

uint8 huandao=0;
uint8 huandao_count=0;
//int16 ADFilter[4]={0};		//�����˲��ĵ��ֵ��δʹ�ã�


/**
 * @file		�������
 *				һ������£�����ˮƽ��еĲ�Ⱥ���Ϊƫ��
 *				�ڻ���ʱ�У�������ֱ��еĲ�Ⱥ���Ϊƫ��
 *
 *									���ֵ��Ӧ����
 *
 *				    AD[1]				    AD[2]						         AD[3]             AD[4]
 *				(ˮƽ����)		����ֱ���У�					����ֱ�ҵ�У�		��ˮƽ�ҵ�У�
 *
 * @date		2022 by AceTaffy gzc
 */
static float g_fDirectionErrorTemp[2][5];



void DirectionControl(void)
{	
	g_dirControl_P = dir_P;
  g_dirControl_D = dir_D;
	
	icm20602_get_gyro();
	
//	g_adValueFilter[1] = (g_adValueFilter[1] < 10? 10:g_adValueFilter[1]);	//�ĸ����ֵ�޷�
//	g_adValueFilter[2] = (g_adValueFilter[2] < 10? 10:g_adValueFilter[2]);
//	g_adValueFilter[3] = (g_adValueFilter[3] < 10? 10:g_adValueFilter[3]);
//	g_adValueFilter[4] = (g_adValueFilter[4] < 10? 10:g_adValueFilter[4]);

	g_fDirectionError[0] = (float)KP*( sqrt(g_adValue[1]) - sqrt(g_adValue[4]) ) / ( sqrt(g_adValue[1]) + sqrt(g_adValue[4]) );//ˮƽ��еĲ�Ⱥ���Ϊƫ��
	
//	g_fDirectionError[0]=KalmanFilter_Elect(g_fDirectionError[0],g_fDirectionError[2]);//�������˲�
	
	g_fDirectionError[0] = (g_fDirectionError[0]>= 1? 1:g_fDirectionError[0]);	//ƫ���޷�
	g_fDirectionError[0] = (g_fDirectionError[0]<=-1?-1:g_fDirectionError[0]);
	
//	g_fDirectionError[1] = (float)(g_adValueFilter[2] - g_adValueFilter[3])/(g_adValueFilter[2] + g_adValueFilter[3]);//��ֱ��еĲ�Ⱥ���Ϊƫ��
//	g_fDirectionError[1] = (g_fDirectionError[1]>= 1? 1:g_fDirectionError[1]);	//ƫ���޷�
//	g_fDirectionError[1] = (g_fDirectionError[1]<=-1?-1:g_fDirectionError[1]);	
	  
//	g_fDirectionErrorTemp[0][4] = g_fDirectionErrorTemp[0][3];
	g_fDirectionErrorTemp[0][3] = g_fDirectionErrorTemp[0][2];
	g_fDirectionErrorTemp[0][2] = g_fDirectionErrorTemp[0][1];
	g_fDirectionErrorTemp[0][1] = g_fDirectionErrorTemp[0][0];
	g_fDirectionErrorTemp[0][0] = g_fDirectionError[0];
	g_fDirectionError_dot[0] = 5*(float)(g_fDirectionErrorTemp[0][0]-g_fDirectionErrorTemp[0][3]);    //ˮƽ��е�ƫ��΢��
	g_fDirectionError_dot[0] = (g_fDirectionError_dot[0]> 1.0? 1.0:g_fDirectionError_dot[0]);  //ƫ��΢���޷�
	g_fDirectionError_dot[0] = (g_fDirectionError_dot[0]<-1.0?-1.0:g_fDirectionError_dot[0]);

//	g_fDirectionErrorTemp[1][4] = g_fDirectionErrorTemp[1][3];
//	g_fDirectionErrorTemp[1][3] = g_fDirectionErrorTemp[1][2];
//	g_fDirectionErrorTemp[1][2] = g_fDirectionErrorTemp[1][1];
//	g_fDirectionErrorTemp[1][1] = g_fDirectionErrorTemp[1][0];
//	g_fDirectionErrorTemp[1][0] = g_fDirectionError[1];
//	g_fDirectionError_dot[1] = 5*(g_fDirectionErrorTemp[1][0]-g_fDirectionErrorTemp[1][3]);    //��ֱ��е�ƫ��΢��
//	g_fDirectionError_dot[1] = (g_fDirectionError_dot[1]> 0.7? 0.7:g_fDirectionError_dot[1]);  //ƫ��΢���޷�
//	g_fDirectionError_dot[1] = (g_fDirectionError_dot[1]<-0.7?-0.7:g_fDirectionError_dot[1]);
	
////	g_fDirectionErrorTemp[0][3] = g_fDirectionErrorTemp[0][2];
////	g_fDirectionErrorTemp[0][2] = g_fDirectionErrorTemp[0][1];
////	g_fDirectionErrorTemp[0][1] = g_fDirectionErrorTemp[0][0];
////	g_fDirectionErrorTemp[0][0] = g_fDirectionError[0];
////	
////	g_fDirectionError_dot[0] = (g_fDirectionErrorTemp[0][0]-g_fDirectionErrorTemp[0][1])-2
////	(g_fDirectionErrorTemp[0][1]-g_fDirectionErrorTemp[0][2])+(g_fDirectionErrorTemp[0][2]-g_fDirectionErrorTemp[0][3]);    //ˮƽ��е�ƫ��΢��

////	g_fDirectionError_dot[0] = (g_fDirectionError_dot[0]> 0.7? 0.7:g_fDirectionError_dot[0]);  //ƫ��΢���޷�
////	g_fDirectionError_dot[0] = (g_fDirectionError_dot[0]<-0.7?-0.7:g_fDirectionError_dot[0]);
	

//	g_fDirectionErrorTemp[1][3] = g_fDirectionErrorTemp[1][2];
//	g_fDirectionErrorTemp[1][2] = g_fDirectionErrorTemp[1][1];
//	g_fDirectionErrorTemp[1][1] = g_fDirectionErrorTemp[1][0];
//	g_fDirectionErrorTemp[1][0] = g_fDirectionError[1];
//	
//  g_fDirectionError_dot[1] = (g_fDirectionErrorTemp[1][0]-g_fDirectionErrorTemp[1][1])-2*
// (g_fDirectionErrorTemp[1][1]-g_fDirectionErrorTemp[1][2])+(g_fDirectionErrorTemp[1][2]-g_fDirectionErrorTemp[1][3]);    //��ֱ��е�ƫ��΢��

//	g_fDirectionError_dot[1] = (g_fDirectionError_dot[1]> 0.7? 0.7:g_fDirectionError_dot[1]);  //ƫ��΢���޷�
//	g_fDirectionError_dot[1] = (g_fDirectionError_dot[1]<-0.7?-0.7:g_fDirectionError_dot[1]);
   	T_Z = (icm20602_gyro_z - 11)/500;
	
	//�����㷨��λ��ʽPD��
//	g_fDirectionControlOut = g_fDirectionError[0]*g_dirControl_P +(g_fDirectionError_dot[0])*g_dirControl_D - g_dirControl_G*G_Z;
	g_fDirectionControlOut = g_fDirectionError[0]*g_dirControl_P +(g_fDirectionError_dot[0])*g_dirControl_D;
	
//	g_fDirectionControlOut = g_fDirectionControlOut > 60 ? 60 :g_fDirectionControlOut;
//	g_fDirectionControlOut = g_fDirectionControlOut < -60 ? -60:g_fDirectionControlOut;

g_fDirectionError[2]=g_fDirectionError[0];    //���²�Ⱥ�
		
}