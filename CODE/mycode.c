#include "headfile.h"


templ_pluse = 0;
tempr_pluse = 0;
pid_mode= 0;
float dt = 0.004; 
float icmdata_YawVelocity;
float icmdata_Yaw=0;
float encoder_sum=0;
float encoder_avg=0;
//uint8 icm20602_acc_xy=0;
void channal_encoder()
{
	//��ȡ�ɼ����ı�����������
        templ_pluse = ctimer_count_read(SPEEDL_PLUSE);
				tempr_pluse = ctimer_count_read(SPEEDR_PLUSE);

        //����������
        ctimer_count_clean(SPEEDL_PLUSE);
				ctimer_count_clean(SPEEDR_PLUSE);

        //�ɼ�������Ϣ
        if(1 == SPEEDL_DIR)    
        {
            templ_pluse = templ_pluse;
        }
        else                  
        {
            templ_pluse = -templ_pluse;
        }
		if(1 == SPEEDR_DIR)    
        {
            tempr_pluse = -tempr_pluse;
        }
        else                  
        {
            tempr_pluse = tempr_pluse;
        }   

				Left.Actual_Speed  = templ_pluse;
				Right.Actual_Speed = tempr_pluse;
				encoder_avg = (float)(templ_pluse+tempr_pluse)/2;
}



void pid_menu()        //��������PIDֵ
{
		Right.P = P_SET;                      //p����
    Right.I = I_SET;                      //i����
    Right.D = D_SET;                      //d����
	   
	  Left.P = P_SET;  							  		  //p����
    Left.I = I_SET;                       //i����
    Left.D = D_SET;                       //d����
	
	
	if(P75==1&&P76==1)   //��������11ʱ������Pֵ
	{
		pid_mode= 1;
		if(P70==0)
		{
			delay_ms(80);
			P_SET=P_SET+1;
		}
		
		if(P71==0)
		{
			delay_ms(80);
			P_SET=P_SET-1;
		}
	}

	
	if(P75==0&&P76==1)   //��������01ʱ������Iֵ
	{
		pid_mode= 2;
		if(P70==0)
		{
			delay_ms(80);
			I_SET++;
		}
		
		if(P71==0)
		{
			delay_ms(80);
			I_SET--;
		}
	}
	
	if(P75==0&&P76==0)   //��������00ʱ������Dֵ
	{
		pid_mode= 3;
		if(P70==0)
		{
			delay_ms(80);
			D_SET++;
		}
		
		if(P71==0)
		{
			delay_ms(80);
			D_SET--;
		}
	}
}

void ICM_OneOrderFilter(void) 
{
	icm20602_gyro_z=icm20602_gyro_z-10.5;
  icmdata_YawVelocity =(float)icm20602_gyro_z / 17.3;

  icmdata_Yaw = (icmdata_YawVelocity) * dt + icmdata_Yaw;
}

/***************************����ƽ����****************************/
//����Ĵ���

const float f = 1.5F;
float my_sqrt(float number)
{
	long i;
	float x, y;
	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( f - ( x * y * y ) );
	y = y * ( f - ( x * y * y ) );
	return number * y;
}


void fuya_start()
{
	if(start_flag==1&&fuya_flag==0)
	{
		 P14=0;       
     pwm_duty(PWMB_CH4_P77, 8000);     
                     
		 P16=0;       
     pwm_duty(PWMB_CH3_P33, 8000);
	}
	
	
	if(fuya_flag!=0||start_flag==2)
	{
		 P14=0;       
     pwm_duty(PWMB_CH4_P77, 0);     
                     
		 P16=0;       
     pwm_duty(PWMB_CH3_P33, 0);
	}
}

void get_distance()
{
		encoder_sum += (float)encoder_avg*0.0068; // �ɱ�������ֵת��Ϊ��ʵ����cm

	// �����޷�,��Խ��
	if (encoder_sum > 10000 || encoder_sum < -10000)
		encoder_sum = 0;
}



