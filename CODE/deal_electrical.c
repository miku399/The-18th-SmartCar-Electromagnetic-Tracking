#include "headfile.h"
int8 flag_CK=0;
int8 flag_HD=0;
int16 AD[7]={0};//��ų�ֵ
uint8 ADS[4]={0};
int16 g_adValue[4]={0};		//��ȡ�ĵ��ֵ
int16 g_adValueFilter[4]={0};	
int16 max_value[6]={0,2000,3000,3000,2000};
//int16 max_value[6]={0,4000,6000,6000,4000};
int G_AD[7];//��һ����ֵ
/***************************************
�������ܣ���ȡ�ĸ���е�ֵ��ƽ���˲�����AD[i]��ȥ��һ�����ֵ��ȥ��һ����Сֵ����ƽ��ֵ
***************************************/
void GetADCValue()
{
	
	int16 i;
//	int16 j;
//	int16 a;
//	int16 k;
//	int32 adValue[7][5];
	/*********�ɼ���ֵ**********/
	for(i = 1; i <= 4; i++)
	{
		//�ɼ���·���ֵ
		g_adValue[1] = adc_once(ADC_P00, ADC_12BIT);//L1
		g_adValue[2] = adc_once(ADC_P01, ADC_12BIT);//L2
    g_adValue[3] = adc_once(ADC_P05, ADC_12BIT);//L6
    g_adValue[4] = adc_once(ADC_P06, ADC_12BIT);//L7
		
	}
	/*********ð������ ˳�򣺴�С����**********/
//	for(i = 1;i <= 4;i++)
//	{
//		for(k = 1;k <= 4;k++)
//		{
//			for(j = 1;j <= 4-k;j++)
//			{
//				if(adValue[i][j] > adValue[i][j+1])
//				{
//					a = adValue[i][j+1];
//					adValue[i][j+1] = adValue[i][j];
//					adValue[i][j] = a;
//				}
//			}
//		}
//	}
//	
	for(i = 1;i <= 4; i++)
	{
		AD[i] = g_adValue[i];
	}
	
	for( i = 1; i <= 4; i++)     //��һ��
  {
    g_adValue[i] = 100.0 * g_adValue[i]/max_value[i];
		// �޷�
		if (g_adValue[i] <= 1) g_adValue[i] = 1;
		if (g_adValue[i] >= 100) g_adValue[i] = 100;
  }
	
		for(i = 1;i <= 4; i++)
	{
		ADS[i] = g_adValue[i];
	}
	
	
	
	if(circle_left==1)   //�󻷵�
{
	if(SC_flag_L==1)
	{
//		g_adValueFilter[1]=g_adValueFilter[1]*4;
//		g_adValueFilter[4]=g_adValueFilter[4]/3;
		g_adValue[1]=60;
		g_adValue[4]=10;
	}
	
	if(SC_flag_eL==2)
	{
		g_adValue[1]=g_adValue[1]*1.5f;
	}
	
	if(cir_cir==1)
	{
		g_adValue[4]=g_adValue[4]*1.5f;
	}
	else
	{
		g_adValue[4]=g_adValue[4];
	}
}




  if(circle_right==1)    //�һ���
	{
		if(SC_flag_R==1)
	{
		g_adValue[1]=10;
		g_adValue[4]=55;
	}
	
	if(SC_flag_eR==2)
	{
		g_adValue[4]=g_adValue[4]*1.5f;	
	}	
	else
	{
		g_adValue[4]=g_adValue[4];	
	}

	if(cir_cir==1)
	{
		g_adValue[1]=g_adValue[1]*1.5f;
	}
	else
	{
		g_adValue[1]=g_adValue[1];
	}
  }
	
	/*************************���ϲ��ٴ��*************************/
	
	if(BZ_turn==1)                      //�����
	{
		g_adValue[1]=50;    //2000
		g_adValue[4]=10;     //500
//		LBZ=0.5;
//		RBZ=1.2;
	}
	
	if(BZ_turn==2)
	{
		g_adValue[1]=25;      //350
		g_adValue[4]=45;     //1800
	}
	
		if(BZ_turn==3)
	{
		g_adValue[1]=18;      //350
		g_adValue[4]=55;     //1800
	}
	
	if(BZ_turn==4)
	{
		g_adValue[1]=35;      //350
		g_adValue[4]=35;     //1800
	}
	
	
	
//		if(BZ_turn==1)                      //�ұ���
//	{
//		g_adValue[1]=10;    //2000
//		g_adValue[4]=50;     //500
//		LBZ=0.5;
//		RBZ=1.2;
//	}
//	
//	if(BZ_turn==2)
//	{
//		g_adValue[1]=45;      //350
//		g_adValue[4]=35;     //1800
//	}
//	
//		if(BZ_turn==3)
//	{
//		g_adValue[1]=50;      //350
//		g_adValue[4]=20;     //1800
//	}
//	
//	if(BZ_turn==4)
//	{
//		g_adValue[1]=35;      //350
//		g_adValue[4]=35;     //1800
//	}
	
/**********************************************************/
	
	
	if(CK_flag==1)
	{
		g_adValue[1]=35;
		g_adValue[4]=35;
	}
	
	if(CK_flag==2)
	{
		if(CK_right==1)
		{
			g_adValue[1]=5;
			g_adValue[4]=60;
			dir_P=40;
			dir_D=60;
		}
		
		if(CK_left==1)
		{
			g_adValue[1]=60;
			g_adValue[4]=5;
			dir_P=40;
			dir_D=60;
		}
		
	}
	
	
	
	if(cir_cir==1)
	{
		g_adValue[4]=g_adValue[4]*1.5;
	}
}



//float KalmanFilter_Elect(float curr_elect_val,float last_elect_val)     //�������˲���δʹ��
//{
//  static float Q_curr = 0.1;//0.1			//Q���󣬶�̬��Ӧ���󣬹���������Э����
//  static float Q_last = 0.0001;			//����������Э�������������Э����Ϊһ��һ�����о���
//  static float R_elect = 10.0;			        //����������Э���� ������ƫ��

//  static float Pk[2][2] = { {1, 0}, {0, 1 }};
//  
//  static float Pdot[4] = {0,0,0,0};

//  static float q_bias = 0.0;
//  static float elect_err = 0.0;
//  static float PCt_0 = 0.0;
//  static float PCt_1 = 0.0;
//  static float E = 0.0;
//  static float K_0 = 0.0, K_1 = 0.0, t_0 = 0.0, t_1 = 0.0;
//  
//  Pdot[0] = Q_curr - Pk[0][1] - Pk[1][0];		//Pk-����������Э�����΢��
//  Pdot[1] = -Pk[1][1];
//  Pdot[2] = -Pk[1][1];
//  Pdot[3] = Q_last;
//  
//  Pk[0][0] += Pdot[0] * dt;				//Pk-�����������Э����΢�ֵĻ���
//  Pk[0][1] += Pdot[1] * dt;				//����������Э����
//  Pk[1][0] += Pdot[2] * dt;
//  Pk[1][1] += Pdot[3] * dt;
//  
//  elect_err = curr_elect_val - last_elect_val;			//ƫ�� = ����ֵ - Ԥ��ֵ���������
//  
//  PCt_0 = Pk[0][0];
//  PCt_1 = Pk[1][0];
//  
//  E = R_elect + PCt_0;
//  
//  K_0 = PCt_0 / E;
//  K_1 = PCt_1 / E;
//  
//  t_0 = PCt_0;
//  t_1 = Pk[0][1];
//  
//  Pk[0][0] -= K_0 * t_0;					//����������Э����
//  Pk[0][1] -= K_0 * t_1;
//  Pk[1][0] -= K_1 * t_0;
//  Pk[1][1] -= K_1 * t_1;
//  
//  curr_elect_val += K_0 * elect_err; 		        //������� �������ŵ��ֵ ���ŵ��ֵ = Ԥ��ֵ + ����������*(����ֵ-Ԥ��ֵ)
//  q_bias += K_1 * elect_err;				//������� �������		
//    
//  return curr_elect_val;

//}