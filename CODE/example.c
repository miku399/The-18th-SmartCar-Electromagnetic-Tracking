#include "headfile.h"
uint8 Str_P=0.0;
uint8 Str_D=0.0;
uint8 Cur_P=0.0;
uint8 Cur_D=0.0;
uint8 Str_KP=0.0;
uint8	Cur_KP=0.0;
uint8 RD_KP=0.0;

float Curve;
float Stright;
float L_CarSpeed;
float R_CarSpeed;
PID_CLASS MotorPID;        //����ṹ���ʼ��
PID_CLASS ServPID;        //����ṹ���ʼ��
uint8 TrackFlag = 0,SancaFlag = 0,InFlag=0;
int16 ad_avr_val[8] = { 0 };
float Angle_Y=0,Angle_Z=0;
int Right_count,Left_count;
float speed_avr;
float pout,pout0;  //�����ֵ



void PIDInit() //PID��ʼ��
{
//��eepromд�����ݣ���д���ʱ��ǵðѲ���eeprom������
//	extern_iap_write_float(30,3,1,0x00);//(30,3,1,0x00)�е�30����Ҫд�������3��������λ��1����С��λ����仰����˼�ǰ�30ת��Ϊ"+030.0"����0x00-0x06��ַ�У���Ϊ"+030.0"ռ������ַ�������ַ����Ľ����ַ�'/0'������ռ���߸���ַ��������0x00��0x06,�¸�����Ҫ���Ե�0x07Ϊ��ʼ��ַ��
//	extern_iap_write_float(15,3,1,0x07);
//	extern_iap_write_float(11000,5,1,0x0e);
//	extern_iap_write_float(315,4,1,0x17);
//	extern_iap_write_float(2.0,1,1,0x1f);
//	extern_iap_write_float(1.5,1,1,0x24);
//	extern_iap_write_float(680,3,1,0x29);
	
//��eeprom�ж�ȡ���ݣ��ǵò�Ҫ�Ѳ���eeprom������
	  speed_max  = iap_read_float(6,0x00);//��仰����˼�Ǵ�0x00��ַ��ʼȡ��7����ַ�����ݣ�Ҳ����0x00��0x06��
    speed_min  = iap_read_float(6,0x07);
    Str_P      = iap_read_float(6,0x0f);
    Str_D      = iap_read_float(6,0x17);
	  Str_KP     = iap_read_float(6,0x1e);
	  Cur_P      = iap_read_float(6,0x25);
	  Cur_D      = iap_read_float(6,0x2c);
	  Cur_KP     = iap_read_float(6,0x33);
	  RD_KP      = iap_read_float(6,0x3a);
	  island_right_data = iap_read_float(6,0x41);
	  island_left_data  = iap_read_float(6,0x48);
		CirR_KP    = iap_read_float(6,0x4f);
		CirL_KP    = iap_read_float(6,0x56);
		island_right_dis  = iap_read_float(6,0x5d);
		RK_Dis     = iap_read_float(6,0x64);
		BZ_Dis     = iap_read_float(6,0x6d);
		BZ_DisS     = iap_read_float(7,0x75);
		RK_Yaw      = iap_read_float(6,0x7c);
//	  speed_max  = iap_read_float(7,0x1f);2f
//	  speed_min  = iap_read_float(7,0x26);
//	pout0      = iap_read_float(7,0x29);
}