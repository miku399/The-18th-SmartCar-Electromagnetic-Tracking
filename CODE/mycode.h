#ifndef _MYCODE_H_
#define _MYCODE_H_

extern int16 templ_pluse;
extern int16 tempr_pluse;
extern int16 pid_mode;
extern float icmdata_Yaw;
extern float icmdata_YawVelocity;
extern float dt;
extern float encoder_sum;

//定义脉冲引脚
#define SPEEDL_PLUSE   CTIM0_P34
#define SPEEDR_PLUSE   CTIM3_P04
//定义方向引脚
#define SPEEDL_DIR     P35
#define SPEEDR_DIR     P53

void channal_encoder();
void pid_menu();
void ICM_OneOrderFilter(void);
float my_sqrt(float number);
void fuya_start();
void get_distance();
#endif
