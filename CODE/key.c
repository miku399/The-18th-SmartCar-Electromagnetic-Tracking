#include "key.h"
extern uint8 TrackFlag;
extern UI_CLASS ui; 
extern uint16 ad_avr_val[8];
extern int Right_count,Left_count;
extern PID_CLASS MotorPID; 
extern PID_CLASS ServPID; 
//void key_init(void)
//{
//	gpio_mode(KEY_S1, GPO_PP);//进入二级菜单
//	gpio_mode(KEY_S2, GPO_PP);//退出二级菜单
//	gpio_mode(KEY_S3, GPO_PP);//往上
//	gpio_mode(KEY_S4, GPO_PP);//往下
//	gpio_mode(KEY_S5, GPO_PP);//翻页或者按键调参+
//	gpio_mode(KEY_S6, GPO_PP);//翻页或者按键调参-
//	gpio_mode(P73, GPO_PP);
//}
void keyScan(void)
{
	if(P73==0)
	{
		TrackFlag = 9;
	}
	if(KEY_S1==0)
	{
		ui.enter = ui.cursor[ui.page];
		ui.cursor[ui.page] = 0;
		while(1)
			if(KEY_S1==1)
				break;
	}
		
	if(KEY_S2==0)
	{	
		if(ui.enter > 0)
			ui.cursor[ui.page] = ui.enter;
		ui.enter = -1;
		while(1)
			if(KEY_S2==1)
				break;
	}
	
//	if(KEY_S3==0)
//	{
//		ui.cursor[ui.page]--;
//		if (ui.cursor[ui.page] < 0)
//		{
//			ui.cursor[ui.page] = 8;
//		}
//		beetime=10;
//		while(1)
//			if(KEY_S3==1)
//				break;
//	}
	
	if(KEY_S4==0)
	{
		ui.cursor[ui.page]++;
		if (ui.cursor[ui.page] > 8)
		{
			ui.cursor[ui.page] = 0;
		}
		while(1)
			if(KEY_S4==1)
				break;
	}	

	if(KEY_S5==0)
	{
		if((ui.cursor[0] == 0 || ui.cursor[1] == 0 ||ui.cursor[2] == 0 || ui.cursor[3] == 0 ) && ui.enter <= 0)
		{
			ui.page++;
			if(ui.page > 3)
			{
				ui.page = 3;
			}
			ui.cursor[ui.page] = 0;
			if(ui.page > 0)
			{
				ui.cursor[ui.page - 1] = 0;
			}
		}
		else
		{
			
			if(ui.enter == 2 && ui.cursor[ui.page] == 0)
			{
				speed_max += 1.0;
				extern_iap_write_float(speed_max,3,1,0x00);

			}
			if(ui.enter == 2 && ui.cursor[ui.page] == 2)
			{
				speed_min += 1.0;
				extern_iap_write_float(speed_min,3,1,0x07);
			}
			
			
			
			
			
			if(ui.enter == 4 && ui.cursor[ui.page] == 0)
			{
				if(Str_P<40)
					Str_P+=10.0;
				else
				Str_P += 1.0;
				extern_iap_write_float(Str_P,3,1,0x0f);

			}
			if(ui.enter == 4 && ui.cursor[ui.page] == 1)
			{
				if(Str_D<40)
					Str_D+=10.0;
				else
				Str_D += 1.0;
				extern_iap_write_float(Str_D,3,1,0x17);
			}
			if(ui.enter == 4 && ui.cursor[ui.page] == 2)
			{
				Str_KP += 1.0;
				extern_iap_write_float(Str_KP,3,1,0x1e);
			}	
			
			if(ui.enter == 4 && ui.cursor[ui.page] == 3)
			{
				if(Cur_P<40)
					Cur_P+=10.0;
				else
				Cur_P += 1.0;
				extern_iap_write_float(Cur_P,3,1,0x25);

			}
			if(ui.enter == 4 && ui.cursor[ui.page] == 4)
			{
				if(Cur_D<40)
					Cur_D+=10.0;
				else
				Cur_D += 1.0;
				extern_iap_write_float(Cur_D,3,1,0x2c);
			}
			if(ui.enter == 4 && ui.cursor[ui.page] == 5)
			{
				Cur_KP += 1.0;
				extern_iap_write_float(Cur_KP,3,1,0x33);
			}	
			if(ui.enter == 4 && ui.cursor[ui.page] == 6)
			{
				RD_KP += 1.0;
				extern_iap_write_float(RD_KP,3,1,0x3a);
			}
			
			
			
			
			
			
	
			if(ui.enter == 5 && ui.cursor[ui.page] == 0)
			{
				if(island_right_data<100)
				island_right_data +=10;
				else
				island_right_data += 2;
				extern_iap_write_float(island_right_data,3,1,0x41);

			}
			if(ui.enter == 5 && ui.cursor[ui.page] == 1)
			{
				if(island_left_data<100)
				island_left_data +=10;
				else
				island_left_data += 2;
				extern_iap_write_float(island_left_data,3,1,0x48);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 2)
			{
				CirR_KP += 1;
				extern_iap_write_float(CirR_KP,3,1,0x4f);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 3)
			{
				CirL_KP += 1;
				extern_iap_write_float(CirL_KP,3,1,0x56);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 4)
			{
					island_right_dis += 5;
				extern_iap_write_float(island_right_dis,3,1,0x5d);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 6)
			{
					RK_Dis += 2;
				extern_iap_write_float(RK_Dis,3,1,0x64);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 7)
			{
					RK_Yaw += 2;
				extern_iap_write_float(RK_Yaw,3,1,0x7c);
			}
			
			
			
			if(ui.enter == 6 && ui.cursor[ui.page] == 0)
			{
					BZ_Dis += 10;
				extern_iap_write_float(BZ_Dis,4,1,0x6d);
			}
			if(ui.enter == 6 && ui.cursor[ui.page] == 1)
			{
					BZ_DisS += 10;
				extern_iap_write_float(BZ_DisS,4,1,0x75);
			}
			
			
			
			if(ui.enter == 7 && ui.cursor[ui.page] == 0)
			{
				delay_ms(1000);
				start_flag=1;
			}
			if(ui.enter == 7 && ui.cursor[ui.page] == 1)
			{
				CK_left=1;
			}
			if(ui.enter == 7 && ui.cursor[ui.page] == 2)
			{
				CK_right=0;
			}
			if(ui.enter == 7 && ui.cursor[ui.page] == 3)
			{
				circle_left=1;
			}
			if(ui.enter == 7 && ui.cursor[ui.page] == 4)
			{
				circle_right=0;
			}
			
		}			
		while(1)
			if(KEY_S5==1)
				break;
	}

	if(KEY_S6==0)
	{
		if((ui.cursor[0] == 0 || ui.cursor[1] == 0 ||ui.cursor[2] == 0 || ui.cursor[3] == 0) && ui.enter <= 0)
		{
			ui.page--;
			if(ui.page < 0)
			{
				ui.page = 0;
			}
			ui.cursor[ui.page] = 0;
			if(ui.page < 3)
			{
			ui.cursor[ui.page + 1] = 0;
			}
		}
		else
		{
			
			if(ui.enter == 2 && ui.cursor[ui.page] == 0)
			{
				speed_max -= 1.0;
				extern_iap_write_float(speed_max,3,1,0x00);		
			}
			if(ui.enter == 2 && ui.cursor[ui.page] == 2)
			{
				speed_min -= 1.0;
				extern_iap_write_float(speed_min,3,1,0x07);

			}
			
			if(ui.enter == 4 && ui.cursor[ui.page] == 0)
			{
				if(Str_P>100)
				Str_P -= 10.0;
				else
				Str_P -= 1.0;
				extern_iap_write_float(Str_P,3,1,0x0f);

			}
			if(ui.enter == 4 && ui.cursor[ui.page] == 1)
			{
				if(Str_D>100)
				Str_D -= 10.0;
				else
				Str_D -= 1.0;
				extern_iap_write_float(Str_D,3,1,0x17);
			}
			if(ui.enter == 4 && ui.cursor[ui.page] == 2)
			{ 
				if(Str_KP>20)
				Str_KP -= 10.0;
				else
				Str_KP -= 1.0;
				extern_iap_write_float(Str_KP,3,1,0x1e);
			}	
			
			if(ui.enter == 4 && ui.cursor[ui.page] == 3)
			{
				if(Cur_P>100)
				Cur_P -= 10.0;
				else
				Cur_P -= 1.0;
				extern_iap_write_float(Cur_P,3,1,0x25);

			}
			if(ui.enter == 4 && ui.cursor[ui.page] == 4)
			{
				if(Cur_D>100)
				Cur_D -= 10.0;
				else
				Cur_D -= 1.0;
				extern_iap_write_float(Cur_D,3,1,0x2c);
			}
			if(ui.enter == 4 && ui.cursor[ui.page] == 5)
			{
				if(Cur_KP>20)
				Cur_KP -= 10.0;
				else
				Cur_KP -= 1.0;
				extern_iap_write_float(Cur_KP,3,1,0x33);
			}
			if(ui.enter == 4 && ui.cursor[ui.page] == 6)
			{
				if(RD_KP>20)
					RD_KP-=10.0;
				else
					RD_KP -= 1.0;
				extern_iap_write_float(RD_KP,3,1,0x3a);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 0)
			{
				if(island_right_data>100)
				island_right_data -=10;
				else
				island_right_data -= 2;
				extern_iap_write_float(island_right_data,3,1,0x41);

			}
			if(ui.enter == 5 && ui.cursor[ui.page] == 1)
			{
				if(island_left_data>100)
				island_left_data -=10;
				else
				island_left_data -= 2;
				extern_iap_write_float(island_left_data,3,1,0x48);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 2)
			{
				if(CirR_KP>10)
				CirR_KP -=10;
				else
				CirR_KP -= 1;
				extern_iap_write_float(CirR_KP,3,1,0x4f);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 3)
			{
				if(CirL_KP>10)
				CirL_KP -=10;
				else
				CirL_KP -= 1;
				extern_iap_write_float(CirL_KP,3,1,0x56);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 4)
			{
				if(island_right_dis>100)
					island_right_dis-=10;
				else
					island_right_dis -= 2;
				extern_iap_write_float(island_right_dis,3,1,0x5d);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 6)
			{
				if(RK_Dis>100)
					RK_Dis-=10;
				else
					RK_Dis -= 2;
				extern_iap_write_float(RK_Dis,3,1,0x64);
			}
			
			if(ui.enter == 5 && ui.cursor[ui.page] == 7)
			{
				if(RK_Yaw>100)
					RK_Yaw-=10;
				else
					RK_Yaw -= 2;
				extern_iap_write_float(RK_Yaw,3,1,0x7c);
			}
			
			if(ui.enter == 6 && ui.cursor[ui.page] == 0)
			{
				if(BZ_Dis>=100)
					BZ_Dis -=10;
				else
					BZ_Dis -= 1;
				extern_iap_write_float(BZ_Dis,4,1,0x6d);
			}
			if(ui.enter == 6 && ui.cursor[ui.page] == 1)
			{
				if(BZ_DisS>=1000)
					BZ_DisS -=100;
				else
					BZ_DisS -= 10;
				extern_iap_write_float(BZ_DisS,4,1,0x75);
			}
			
		}
		while(1)
			if(KEY_S6==1)
				break;
	}
}