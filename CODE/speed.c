#include "headfile.h"

/*    	每日一拜 好过省赛             天天参拜 稳过国赛        */
/* \\ \\ \\ \\ \\ \\ \\ || || || || || || // // // // // // // //
\\ \\ \\ \\ \\ \\ \\        _ooOoo_          // // // // // // //
\\ \\ \\ \\ \\ \\          o8888888o            // // // // // //
\\ \\ \\ \\ \\             88" . "88               // // // // //
\\ \\ \\ \\                (| -_- |)                  // // // //
\\ \\ \\                   O\  =  /O                     // // //
\\ \\                   ____/`---'\____                     // //
\\                    .'  \\|     |//  `.                      //
==                   /  \\|||  :  |||//  \                     ==
==                  /  _||||| -:- |||||-  \                    ==
==                  |   | \\\  -  /// |   |                    ==
==                  | \_|  ''\---/''  |   |                    ==
==                  \  .-\__  `-`  ___/-. /                    ==
==                ___`. .'  /--.--\  `. . ___                  ==
==              ."" '<  `.___\_<|>_/___.'  >'"".               ==
==            | | :  `- \`.;`\ _ /`;.`/ - ` : | |              \\
//            \  \ `-.   \_ __\ /__ _/   .-` /  /              \\
//      ========`-.____`-.___\_____/___.-`____.-'========      \\
//                           `=---='                           \\
// //   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  \\ \\
// // //      佛祖保佑      永无BUG      永不修改        \\ \\ \\
// // // // // // || || || || || || || || || || \\ \\ \\ \\ \\ */


float Pulse_count=0;
float Pulse_count_end=0;   //编码器计算距离
int16 Pulse = 0;
int8 start_flag=0;
uint8 stop_flag=0;
uint16 differ_speed = 0;
uint16 Motor_Set_Speed = 0;
uint16 RKcount = 0;
uint16 dir_count =0;
uint16 RK_flag = 0;
uint16 RK_tcount = 0;
uint8 RK_Dis = 0;
uint8 RK_Yaw = 0;

uint8  CK_left  = 0;
uint8  CK_right = 1;

uint8 CK_flag = 0;
uint8 CK_flag_temp = 0;
uint8 BZ_flag = 0;     //避障变量
uint8 BZ_turn = 0;
uint8 BZ_stop = 0;
uint16 BZ_count = 0;
uint16 BZ_Dis = 0;
uint16 BZ_DisS = 0;
uint8 icm_count = 0;

float speed_Left_err = 0;
float speed_Right_err = 0;
float speed_Left_err_temp = 0;
float speed_Right_err_temp = 0;

uint8 SC_flag_L    = 0;   //环岛变量
uint8 SC_flag_R    = 0;
uint8 SC_flag_eL   = 0 ;
uint8 SC_flag_eR   = 0;
uint8 SC_flag_back = 0;

uint8 RD_flag=0;       //圆环变量
uint16 round_count=0;

uint16 LBZ_flag = 0;
uint8 speed=0;
uint8 speed_start=0;
uint16 speed_count=0;
uint8 speed_max=0;
uint8 speed_min=0;



uint8 fuya_flag=0;
float barrier_count[1]={0};
uint8 island_speed=1;
uint8 island_flag=0;
uint16 island_count=0;
uint8 island_flag_temp=0;

uint8 island_right_data = 0; //左右环岛阈值设定
uint8 island_left_data  = 0;
uint8 island_right_dis  = 0;

uint8 circle_right = 1;      //左右环岛设定
uint8 circle_left  = 0;

uint8 CirR_KP = 0;
uint8 CirL_KP = 0;

uint8 AC_speed=0;
int16 Ad_his[4];
int16 Err=0;

uint8 fast_flag=0;
//int16 encoder_avg=0;


float prev_Left_speed  = 0;
float prev_Right_speed = 0;
float curr_Left_speed  = 0;
float curr_Right_speed = 0;
float road_type = 0;


uint8 cir_temp=0;
uint8 cir_cir=0;
uint8 cir_count=0;

uint16 temp_count=0;
float KP = 1;
uint16 HighestSpeed=120,g_LowestSpeed=100;
#define HALL_PIN P26

Pid_Inc Right;
Pid_Inc Left;

final_direction Gyro;

int16 PulseRight=0;
int16 PulseLeft=0;

int16 P_SET=75;//85      //100       //75
int16 I_SET=13;//13       //25       //13
int16 D_SET=2;//40      //100        //5

float g_fSpeedControlOut = 80;

void Pid_Inc_Init()
{

    Right.P = P_SET;                      //p参数
    Right.I = I_SET;                      //i参数
    Right.D = D_SET;                      //d参数

    Right.Err = 0.0;                    //偏差值
    Right.Err_Prev = 0.0;               //定义上上个偏差值
    Right.Err_Last = 0.0;               //定义上一个偏差值
    Right.Set_Speed = 0;                //设定的速度
    Right.Actual_Speed = 0.0;           //定义实际速度
    Right.Out = 0.0;                    //电机输出
    Right.increment = 0.0;              //定义增量
    Right.proportion = 0.0;             //PID比例项
    Right.integration = 0.0;            //PID积分项
    Right.differential = 0.0;           //PID微分项
	
	  Left.P = P_SET;                      //p参数
    Left.I = I_SET;                      //i参数
    Left.D = D_SET;                      //d参数

    Left.Err = 0.0;                    //偏差值
    Left.Err_Prev = 0.0;               //定义上上个偏差值
    Left.Err_Last = 0.0;               //定义上一个偏差值
    Left.Set_Speed = 0;                //设定的速度
    Left.Actual_Speed = 0.0;           //定义实际速度
    Left.Out = 0.0;                    //电机输出
    Left.increment = 0.0;              //定义增量
    Left.proportion = 0.0;             //PID比例项
    Left.integration = 0.0;            //PID积分项
    Left.differential = 0.0;           //PID微分项
}

//增量式PID中被控制量一直都是偏差error，并不是直接控制速度，控制的目的是让偏差趋近于0。

void Pid_Deal(Pid_Inc *pid)
{
    pid->Err = pid->Set_Speed - pid->Actual_Speed;                              //偏差 = 期望 - 实际

    pid->proportion = pid->P * (pid->Err - pid->Err_Last);                      //比例

    pid->integration = pid->I * pid->Err;                                       //积分

    pid->differential = pid->D * (pid->Err - 2*pid->Err_Last + pid->Err_Prev);  //微分

    pid->increment = pid->proportion + pid->integration + pid->differential;    //增量 = 比例 + 积分 + 微分

    pid->Out += pid->increment;                                                 //输出

    pid->Err_Prev = pid->Err_Last;                                              //上上次偏差赋值

    pid->Err_Last = pid->Err;                                                   //上次偏差赋值
			
}



void Right_Out_Pwm()
{

	  Right.Out = Right.Out >  10000 ?  10000 : Right.Out;
    Right.Out = Right.Out < -10000 ? -10000 : Right.Out;//限幅
    if(Right.Out >= 0)//正转
    {	
			  pwm_duty(PWMA_CH1P_P60, 0);
	      pwm_duty(PWMA_CH2P_P62, Right.Out);
    }
    else
    {
	      pwm_duty(PWMA_CH1P_P60, -Right.Out);
	      pwm_duty(PWMA_CH2P_P62, 0);
    }

}

void Left_Out_Pwm()
{
	
		Left.Out = Left.Out >  10000 ?  10000 : Left.Out;
    Left.Out = Left.Out < -10000 ? -10000 : Left.Out;//限幅
	
    if(Left.Out >= 0)//正转
    {	
				pwm_duty(PWMA_CH3P_P64, 0);
				pwm_duty(PWMA_CH4P_P66, Left.Out);
    }
    else
    {
				pwm_duty(PWMA_CH3P_P64, -Left.Out);
			  pwm_duty(PWMA_CH4P_P66, 0);
		}
}


/********************************出库控制**********************************/


void CK_Start()
{
	if(CK_right==1)                  //右出库
{
	if(CK_flag!=2&&CK_flag!=3)
 {
	if(ADS[2]+ADS[3]<50&&CK_flag_temp==0)
		CK_flag=1;
	else
	{
		CK_flag=2;
		CK_flag_temp=1;
	}
 }
 
  if(icmdata_Yaw<-60||(ADS[1]>20&&ADS[4]>20&&abs(ADS[1]-ADS[4]<13)))
	{
		CK_flag=3;
	}
	
    speed_count++;
	if(speed_count>=2)
	{
		speed++;
		speed_count=0;
	}
}

	if(CK_left==1)                   //左出库
{
	if(CK_flag!=2&&CK_flag!=3)
 {
	if(ADS[2]+ADS[3]<50&&CK_flag_temp==0)
		CK_flag=1;
	else
	{
		CK_flag=2;
		CK_flag_temp=1;
	}
 }
 
  if(icmdata_Yaw>60||(ADS[1]>20&&ADS[4]>20&&abs(ADS[1]-ADS[4]<13)))
	{
		CK_flag=3;
	}
	
    speed_count++;
	if(speed_count>=2)
	{
		speed++;
		speed_count=0;
	}
}
	
	speed = speed > 85 ? 85 : speed;
	
	Left.Set_Speed  = (int32)(speed - g_fDirectionControlOut);
	Right.Set_Speed = (int32)(speed + g_fDirectionControlOut);
	
	channal_encoder();
	
	Pid_Deal(&Left); //PID处理
	Pid_Deal(&Right); //PID处理
    
	
	Left_Out_Pwm();//pwm输出设置
	Right_Out_Pwm();//pwm输出设置
}


/********************************入库控制**********************************/

void RK_Stop()
{
	if(CK_left==1)
{
	if(RK_flag==2)              //左入库
	{
		if(icmdata_Yaw<-70)
		{
			RK_tcount=0;
			RK_flag=3;
		}
		Left.Set_Speed  = -40;
	  Right.Set_Speed = -80;	
	}
	
	if(RK_flag==3)
	{
		Left.Set_Speed  = -60;
	  Right.Set_Speed = -60;
		RK_tcount++;
		if(RK_tcount>=160)
		RK_flag=4;
	}
	
	if(RK_flag==4)
	{
		Left.Set_Speed  = 0;
	  Right.Set_Speed = 0;
	}	
}	



	if(CK_right==1)
{
		if(RK_flag==2)              //右入库
	{
		if(icmdata_Yaw > RK_Yaw)
		{
			RK_tcount=0;
			RK_flag=3;
		}
		Left.Set_Speed  = -80;
	  Right.Set_Speed = -40;
	}

	if(RK_flag==3)
	{
		Left.Set_Speed  = -60;
	  Right.Set_Speed = -60;
		RK_tcount++;
		if(RK_tcount>=100)
		RK_flag=4;
	}

	if(RK_flag==4)
	{
		Left.Set_Speed  = 0;
	  Right.Set_Speed = 0;
	}	
}
	

	
	
	channal_encoder();
	
	Pid_Deal(&Left); //PID处理
	Pid_Deal(&Right); //PID处理
    
	
	Left_Out_Pwm();//pwm输出设置
	Right_Out_Pwm();//pwm输出设置
}




/********************************避障控制**********************************/

void barrier_control()    
{
	
	if(dl1a_distance_mm<=840 && dl1a_distance_mm>= 400 && LBZ_flag==0 && CK_flag==3)//避障标志位置1
		{
			BZ_flag=1;
			
			if(icm_count<=2)
			 icmdata_Yaw=0;     //陀螺仪积分清零
			
			encoder_sum=0;     //编码器积分清零
			icm_count++;
		}		
	
		if(BZ_flag==1)    //进入避障模式    //左避障
		{
			if(icmdata_Yaw<30 && LBZ_flag==0)
			{
         BZ_turn=1;
			}
			if(icmdata_Yaw>20 && BZ_turn==1)
			{
				LBZ_flag=1;
				BZ_turn=2;
				
			}
			
			if(encoder_sum > BZ_Dis && LBZ_flag==1) //回到和赛道平行
			{
				BZ_turn=3;
				LBZ_flag=2;
			}
			
			if(icmdata_Yaw < -40 && LBZ_flag==2)
			{
				 BZ_turn=4;
				 KP=2.0;
			}
			if(abs(AD[1]-AD[4])<200&&(AD[1]>250||AD[4]>250)&&LBZ_flag!=0&&BZ_turn>=4)
			{
				Pulse_count=0;
				BZ_count=0;
				BZ_flag=0;
        BZ_turn=0;
        LBZ_flag=3;
        icm_count=0;
			}			
		}
		
		
		
		
//				if(BZ_flag==1)    //进入避障模式    //右避障
//		{
//			if(icmdata_Yaw>-30 && LBZ_flag==0)
//			{
//         BZ_turn=1;
//			}
//			if(icmdata_Yaw<-20 && BZ_turn==1)
//			{
//				LBZ_flag=1;
//				BZ_turn=2;
//				
//			}
//			
//			if(encoder_sum < -35 && LBZ_flag==1) //回到和赛道平行
//			{
//				BZ_turn=3;
//				LBZ_flag=2;
//			}
//			
//			if(icmdata_Yaw > 50 && LBZ_flag==2)
//			{
//				 BZ_turn=4;
//				 KP=2.0;
//			}
//			if(abs(AD[1]-AD[4])<200&&(AD[1]>250||AD[4]>250)&&LBZ_flag!=0&&BZ_turn>=4)
//			{
//				Pulse_count=0;
//				BZ_count=0;
//				BZ_flag=0;		
//        BZ_turn=0;		
//        LBZ_flag=3;		
//        icm_count=0;				
//			}			
//		}
		
		
		
		
		
		if(BZ_flag!=0)
		{
			BZ_count++;
		}
		if(BZ_count>1000)
		{
			BZ_stop=1;
		}
}

/********************************圆环控制**********************************/

void round_control()
{
	if((ADS[2]<=1&&ADS[3]>55)||(ADS[3]<=1&&ADS[2]>55)&&RD_flag==0)
	{
		RD_flag=1;
		icmdata_Yaw=0;
	}
	
	
	if(RD_flag==1&&icmdata_Yaw>220&&icmdata_Yaw<250)				//左圆环
	{
			RD_flag=2;
	}
	  if(RD_flag==2&&ADS[3]>=20)
		{
		  RD_flag=0;
			round_count=0;
		}
	
	
	if(RD_flag==1&&icmdata_Yaw<-220&&icmdata_Yaw>-250)       //右圆环
	{
		RD_flag=3;
	}
		if(RD_flag==3&&ADS[2]>=20)
		{
			RD_flag=0;
			round_count=0;
		}
		
		if(RD_flag!=0)
		{
			 round_count++;
			 KP=(float)RD_KP/10;
			 speed=speed_min;
		}
		
		if(round_count>=500)
		{
			RD_flag=0;
			round_count=0;
		}
}








/********************************环岛控制**********************************/

void circle_control()
{
	if(circle_right==1)
	{
	if(ADS[1]+ADS[4]>island_right_data && ADS[2]>=ADS[3] && ADS[4]>=ADS[1] && SC_flag_eR==0)	   //右环岛
		    {
					island_count=0;
					icmdata_Yaw=0;
					encoder_sum=0;
					SC_flag_eR=1;
			  }
								 
			 if(SC_flag_eR==1&&encoder_sum>=island_right_dis)
				{
						KP=(float)CirR_KP/10;		
						SC_flag_R=1;
						if(icmdata_Yaw<-40)
						{
							SC_flag_R=0;
							SC_flag_eR=2;
							SC_flag_back=1;
						}
				}
 
			if(SC_flag_eR==2)
			{
				KP=(float)CirR_KP/10;
				if(icmdata_Yaw<-330 && icmdata_Yaw>-350)
				{
					SC_flag_eR=3;
					KP=1.1;
				}
			}
			
			if(icmdata_Yaw<-330 && icmdata_Yaw>-350 && cir_temp==0 && SC_flag_back==1)
			{
				cir_cir=1;
				cir_temp=1;
				encoder_sum=0;
			}
			if(encoder_sum>80)
			{
				cir_cir=0;
				cir_temp=0;
				SC_flag_back=0;
			}
	
		}
	
	if(circle_left==1)
	{
		if(ADS[1]+ADS[4]>island_left_data && ADS[2]<=ADS[3] && ADS[4]<=ADS[1] && SC_flag_eL==0)	   //左环岛
		    {
					island_count=0;
					icmdata_Yaw=0;
					encoder_sum=0;
					SC_flag_eL=1;
			  }
								 
			 if(SC_flag_eL==1&&encoder_sum>=25)
				{
						KP=1.5;		
						SC_flag_L=1;
						if(icmdata_Yaw>40)
						{
							SC_flag_L=0;
							SC_flag_eL=2;
						}
				}
 
			if(SC_flag_eL==2)
			{
				KP=(float)CirL_KP/10;
				if(icmdata_Yaw>=330 && icmdata_Yaw<=350)
				{
					SC_flag_eL=0;
					KP=1.1;
				}
				
				if(icmdata_Yaw>=330 && icmdata_Yaw<=350 && cir_temp==0)
			{
				cir_cir=1;
				cir_temp=1;
				encoder_sum=0;
			}
			if(encoder_sum>80)
			{
				cir_cir=0;
				cir_temp=0;
			}
			}
			
	}				
			
	
//	 		 if(ADS[1]+ADS[4]>90&&ADS[2]>60&&ADS[3]<40&&ADS[3]>15&&SC_flag_eL==0)	 //左环岛
//		    {
//				  SC_flag_L=1;
//					island_count=0;
//					icmdata_Yaw=0;
//			  }
//								 
//			 if(SC_flag_L==1)
//				{
//						KP=1.3;		
////						dir_count+=(icm20602_gyro_z/100);
//										 
//						if(icmdata_Yaw>22)
//						{
////							dir_count=0;
//							SC_flag_L=0;
//							SC_flag_eL=1;
//						}
//				}
//				
//			if(SC_flag_eL==1)
//			{
//				KP=1.2;
//				island_count++;
//				if(island_count>=350)
//				{
//					SC_flag_eL=0;
//					KP=1.2;
//					cir_temp=1;
//				}
//			}
//			
//			if(icmdata_Yaw>330&&cir_temp==1)
//			{
//				cir_cir=1;
//				cir_count++;
//			}
//			if(cir_count>200)
//			{
//				cir_cir=0;
//				cir_temp=0;
//			}







}

/********************************速度控制**********************************/

void speed_control()  
{	
	
	if(LBZ_flag==3)
	{
		temp_count++;
	}
	
//		speed_max=130;    //155
//	  speed_min=125;    //153
	
		if(HALL_PIN==0&&RK_flag==0)     //入库标志位
	{
		icmdata_Yaw=0;
		encoder_sum=0;
		RK_flag=1;
		start_flag=2;
	}
		if(RK_flag==1 && encoder_sum >= RK_Dis)
	{
		RK_flag=2;
	}
	

	
	
	if(BZ_flag==0 && SC_flag_L==0 && SC_flag_R==0 && RD_flag!=1 && RK_flag!=1)
	{
		Err = AD[1] - AD[4];
		Ad_his[8] = Ad_his[7];
    Ad_his[7] = Ad_his[6];
		Ad_his[6] = Ad_his[5];
		Ad_his[5] = Ad_his[4];
    Ad_his[4] = Ad_his[3];
		Ad_his[3] = Ad_his[2];
		Ad_his[2] = Ad_his[1];
		Ad_his[1] = Ad_his[0];
		Ad_his[0] =  abs(Err);
  if(Ad_his[0]<350 && Ad_his[2]<350 && Ad_his[4]<350 && Ad_his[6]<350 && Ad_his[8]<350)
    {
//			speed_count++;
//		if(speed_count>=3)
//		{
//			speed++;
//			speed_count=0;
//		}
//		   speed = speed > speed_max ? speed_max :speed;//140
			  speed = speed_max;
		  KP    = Str_KP;       //1.5  //1.6
      dir_P = Str_P;       //80    //65.5    //55
      dir_D = Str_D;       //140   //120     //85
//     ALL_Limit.output_Speed_limit = 10000;
       road_type =200;
    }
  else
    {
			
//		 speed_count++;
//		if(speed_count>=3)
//		{
//			speed--;
//			speed_count=0;
//		}
			
//		   speed  = speed < speed_min ? speed_min : speed ;//120
				  speed = speed_min;
			   KP    = (float)Cur_KP/10;   //1.7   //1.8
				 dir_P = Cur_P;   //80.5   //55
				 dir_D = Cur_D;   //150    //85
//       ALL_Limit.output_Speed_limit = Limit_Moto;
         road_type =200;
    }
  }
	
		 if(RD_flag==1)
		 {
			 speed = speed_min;
			 KP=(float)RD_KP/10;
		 }
		
//	   if(abs(AD[1]-AD[4])>100)
//	    speed = speed - abs(g_fDirectionError[0])*10;
	
		 if(BZ_flag==1)
			 speed=110;
			 
		 curr_Left_speed  = (int32)(speed - g_fDirectionControlOut);
	   curr_Right_speed = (int32)(speed + g_fDirectionControlOut);
	   speed_Left_err_temp  = (curr_Left_speed -200)/10;
	   speed_Right_err_temp = (curr_Right_speed-200)/10;
		
		 if(curr_Left_speed > curr_Right_speed && curr_Left_speed>=180)
		 {
			 speed_Right_err++;
		   speed_Right_err = speed_Right_err > speed_Left_err_temp ? speed_Left_err_temp : speed_Right_err;
		 }
		 else
			 speed_Right_err=0;
		 
	   if(curr_Left_speed < curr_Right_speed && curr_Right_speed>=180)
		 {
			 speed_Left_err++;
	     speed_Left_err = speed_Left_err > speed_Right_err_temp ? speed_Right_err_temp : speed_Left_err;
	   }
		 else
		 	speed_Left_err=0;
		 
	
	   if(curr_Left_speed > Right.Set_Speed)
			 curr_Left_speed=curr_Left_speed > 180 ? 180 :curr_Left_speed;
		 else
			 curr_Right_speed=curr_Right_speed > 180 ? 180 :curr_Right_speed;
		
		 if(curr_Left_speed > prev_Left_speed)      //速度梯度平滑变化
		 {
			 Left.Set_Speed+=road_type;
			 Left.Set_Speed = Left.Set_Speed > curr_Left_speed ? curr_Left_speed : Left.Set_Speed;
		 }
		 
	   if(curr_Left_speed < prev_Left_speed)    
		 {
			 Left.Set_Speed-=road_type;
			 Left.Set_Speed = Left.Set_Speed < curr_Left_speed ? curr_Left_speed : Left.Set_Speed;
		 }
		 
		 if(curr_Right_speed > prev_Right_speed)      
		 {
			 Right.Set_Speed+=road_type;
			 Right.Set_Speed = Right.Set_Speed > curr_Right_speed ? curr_Right_speed : Right.Set_Speed;
		 }
		 
	   if(curr_Right_speed < prev_Right_speed)      
		 {
			 Right.Set_Speed-=road_type;
			 Right.Set_Speed = Right.Set_Speed < curr_Right_speed ? curr_Right_speed : Right.Set_Speed;
		 }
		 
		 Left.Set_Speed = Left.Set_Speed >200 ? 200 : Left.Set_Speed; 
		 Right.Set_Speed = Right.Set_Speed >200 ? 200 : Right.Set_Speed;
		 
		 
			 
		 prev_Left_speed  = Left.Set_Speed;    //过去速度储存
		 prev_Right_speed = Right.Set_Speed;
		 
		 
	
 
	
 		circle_control();
		barrier_control();
//		round_control();
		fuya_start();      //负压启动与关闭函数
	
		if((ADS[1]<=2&&ADS[4]<=2&&BZ_flag==0)||CK_flag==1||BZ_stop==1)   //丢线停止运行
		{	
		  Left.Set_Speed=0;
		  Right.Set_Speed=0;
			fuya_flag=1;
		}	
		
		
		channal_encoder();

		Pid_Deal(&Left); //PID处理
		Pid_Deal(&Right); //PID处理
    
	
		Left_Out_Pwm();//pwm输出设置
	  Right_Out_Pwm();//pwm输出设置
}
