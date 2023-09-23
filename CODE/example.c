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
PID_CLASS MotorPID;        //电机结构体初始化
PID_CLASS ServPID;        //舵机结构体初始化
uint8 TrackFlag = 0,SancaFlag = 0,InFlag=0;
int16 ad_avr_val[8] = { 0 };
float Angle_Y=0,Angle_Z=0;
int Right_count,Left_count;
float speed_avr;
float pout,pout0;  //舵机中值



void PIDInit() //PID初始化
{
//向eeprom写入数据，在写入的时候记得把擦除eeprom区勾上
//	extern_iap_write_float(30,3,1,0x00);//(30,3,1,0x00)中的30代表要写入的数，3代表整数位，1代表小数位。这句话的意思是把30转换为"+030.0"存入0x00-0x06地址中，因为"+030.0"占六个地址，加上字符串的结束字符'/0'，所以占了七个地址，所以是0x00到0x06,下个数据要存以到0x07为起始地址。
//	extern_iap_write_float(15,3,1,0x07);
//	extern_iap_write_float(11000,5,1,0x0e);
//	extern_iap_write_float(315,4,1,0x17);
//	extern_iap_write_float(2.0,1,1,0x1f);
//	extern_iap_write_float(1.5,1,1,0x24);
//	extern_iap_write_float(680,3,1,0x29);
	
//从eeprom中读取数据，记得不要把擦除eeprom区勾上
	  speed_max  = iap_read_float(6,0x00);//这句话的意思是从0x00地址开始取出7个地址的数据，也就是0x00到0x06。
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