#include "UI.h"
extern uint8 TrackFlag,SancaFlag;
extern int Right_count,Left_count;
extern int16 ad_avr_val[8];
extern PID_CLASS MotorPID; 
extern PID_CLASS ServPID;  
extern float Angle_Y,Angle_Z;
extern float err;
extern float speed_avr;
unsigned char ui_page0[10][17] =
{
    " page           ",
    " Sensor         ",
	  " Speed          ",
    " encoder        ",
    " ServPID        ",
    " CircleData     ",
	  " Gyroscope      ",
    " FLAG           ",
    "                ",
    " page1          "
};



unsigned char ui_Sensor[10][17] =
{
    " L1:       ",
    " L2:       ",
    " R2:       ",
		" R1:       ",
		" ICM:     ",
		" Dis:     ",
		" TOF:     ",
		" sp_L:    ", 
		" sp_R:    ",
	  "                ",


};

unsigned char ui_Speed[10][17] =
{
    " Sp_max:",
    "                ",
    " Sp_min:",
    "                ",
    " L_Speed:   ",
    "                ",
	  " R_Speed:   ",
    "                ",
    "                ",
    " Speed          "
};

unsigned char ui_encoder[10][17] =
{
    " LeftCnt:  ",
    "                ",
    " RightCnt: ",
    "                ",
    " L_Speed:   ",
    "                ",
    " R_Speed:   ",
    "                ",
    "                ",
    " encoder        "
};

unsigned char ui_ServPD[10][17] =
{
    " Str_P:",
	  " Str_D:",
	  " Str_KP:",
	  " Cur_P :",
    " Cur_D :",
    " Cur_KP: ",
		" RD_KP :  ",
    "                ",
    "                ",
    " ServPD         "
};
unsigned char ui_MotorPID[10][17] =
{
    " IS_R_AD: ",
		" IS_L_AD: ",
		" IS_R_KP: ",
    " IS_L_KP: ",
		" IS_R_DIS:",
    "                ",
		" RK_DIS:  ",
		" RK_Yaw:  ",
    "                ",
    " MotorPID       "
};
unsigned char ui_Gyroscope[10][17] =
{
    " BZ_Dis:   ",
		"            ",
	  "                ",
    "                ",
	  "                ",
    "                ",
    "                ",
	  "                ",
	  "                ",
    " Gyroscope      "
};
unsigned char ui_FLAG[10][17] =
{
    " StartFlag:  ",
		" CK_left:    ",
	  " CK_right:   ",
		" is_left:    ",
		" is_right:   ",
    "                ",
    "                ",
	  "                ",
	  "                ",
    " FLAG           "
};
UI_CLASS ui =
{
    &UI_Disp,
    {0}, 0, -1
};


static void UI_DispUIStrings(uint8 strings[10][17])
{
	uint8 i;
	for (i = 0; i < 10; i++)
	{
		if (i == ui.cursor[ui.page])
		strings[i][0] = '>';
		else
		strings[i][0] = ' ';
		ips114_showstr(0, i, strings[i]);
	}
}


void UI_Disp(void)
{
	switch(ui.page)
	{
		case 0:
			if(ui.enter == -1)
			{
				UI_DispUIStrings(ui_page0);
			}
			else
			{
				if(ui.enter == 0)
				{
					UI_DispUIStrings(ui_page0);
				}
				if(ui.enter == 1)
				{
					UI_DispUIStrings(ui_Sensor);
					ips114_showuint16(80,0,g_adValue[1]);
					ips114_showuint16(80,1,g_adValue[2]);
					ips114_showuint16(80,2,g_adValue[3]);
					ips114_showuint16(80,3,g_adValue[4]);
					ips114_showint16(80,4,icmdata_Yaw);
					ips114_showint16(80,5,encoder_sum);
					ips114_showint16(80,6,dl1a_distance_mm);
				  ips114_showint16(80,7,templ_pluse);
					ips114_showint16(80,8,tempr_pluse);
				}
				if(ui.enter == 2)
				{
					UI_DispUIStrings(ui_Speed);
					ips114_showfloat(88,0,speed_max,3,1);
					ips114_showfloat(88,2,speed_min,3,1);
					ips114_showfloat(96,4,L_CarSpeed,1,1);
					ips114_showfloat(96,6,R_CarSpeed,1,1);
				}
				else if(ui.enter == 3)
				{
					UI_DispUIStrings(ui_encoder);
					ips114_showuint16(80, 0, Left_count);
					ips114_showuint16(80, 2, Right_count);
					ips114_showfloat(96,4,L_CarSpeed,1,1);
					ips114_showfloat(96,6,R_CarSpeed,1,1);

				}
				else if(ui.enter == 4)
				{
					UI_DispUIStrings(ui_ServPD);
					ips114_showfloat(80, 0, Str_P ,3,1);
					ips114_showfloat(80, 1, Str_D ,3,1);
					ips114_showfloat(80, 2, Str_KP,3,1);
					ips114_showfloat(80, 3, Cur_P ,3,1);
					ips114_showfloat(80, 4, Cur_D ,3,1);
					ips114_showfloat(80, 5, Cur_KP,3,1);
					ips114_showfloat(80, 6, RD_KP ,3,1);
				}
				else if(ui.enter == 5)
				{
					UI_DispUIStrings(ui_MotorPID);
					ips114_showfloat(80, 0, island_right_data,3,1);
					ips114_showfloat(80, 1, island_left_data ,3,1);
					ips114_showfloat(80, 2, CirR_KP,3,1);
					ips114_showfloat(80, 3, CirL_KP,3,1);
					ips114_showfloat(80, 4, island_right_dis,3,1);
					
					ips114_showfloat(80, 6, RK_Dis,3,1);
					ips114_showfloat(80, 7, RK_Yaw,3,1);
				}
				else if(ui.enter ==6)
				{

					UI_DispUIStrings(ui_Gyroscope);
					ips114_showfloat(88,0,BZ_Dis,3,1);
					ips114_showfloat(88,1,BZ_DisS,4,1);
//					ips114_showint16(80, 2, 1);
//					ips114_showint16(80, 3, 1);
//					ips114_showint16(80, 4, 1);
//					ips114_showint16(80, 5, 1);
//					ips114_showint16(80, 6, 1);
//					ips114_showint16(80, 7, 1);

				}
				else if(ui.enter ==7)
				{

					UI_DispUIStrings(ui_FLAG);
					ips114_showuint8(96,0,start_flag);
					ips114_showuint8(96,1,CK_left);
					ips114_showuint8(96,2,CK_right);
					ips114_showuint8(96,3,circle_left);
					ips114_showuint8(96,4,circle_right);
				}
			}
			break;
//			case 1:
//			if(ui.enter == -1)
//			{
//				UI_DispUIStrings(ui_page1);
//			}
//			break;
//			
//			case 2:
//			if(ui.enter == -1)
//			{
//				UI_DispUIStrings(ui_page2);
//			}
//			break;

//			case 3:
//			if(ui.enter == -1)
//			{
//				UI_DispUIStrings(ui_page3);
//			}
//			break;

	}
}