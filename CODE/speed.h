#ifndef CODE_SPEED_CONTROL_H_
#define CODE_SPEED_CONTROL_H_

typedef struct Pid_Increment
{
    float P;                        //p����
    float I;                        //i����
    float D;                        //d����
    float Err;                  //ƫ��ֵ
    float Err_Prev;         //�������ϸ�ƫ��ֵ
    float Err_Last;         //������һ��ƫ��ֵ
    float Set_Speed;        //�趨���ٶ�
    float Actual_Speed;     //����ʵ���ٶ�
    float Out;              //������
    float increment;       //��������
    float proportion;      //PID������
    float integration;     //PID������
    float differential;    //PID΢����
}Pid_Inc;

typedef struct
	{
    float Kp;
    float Kd;

    float direction_set;
    float direction_actul;
    float direction_err;
    float direction_err_last;

    float Pout;
    float Dout;

    float PIDout;
    float PIDout_max;
    float PIDout_min;
	}final_direction;


void speed_control();
extern Pid_Inc Right;
extern Pid_Inc Left;
void Pid_Inc_Init();

extern P_SET;
extern I_SET;
extern D_SET;

extern final_direction Gyro;
void direction_pid_in(void);


#define abs(n) (n>0?n:-n)
#define HALL_PIN P26
extern int16 Pulse;
extern float Pulse_count;
extern uint16 dir_count;
extern Pid_Inc Right;
extern Pid_Inc Left;
extern int16 add_speed;
extern int16 PulseRight;
extern int16 PulseLeft;
extern int16 send_array[8];
extern int32 stop_count;
extern uint8 stop_flag;
extern int8 start_flag;
extern uint8 island_flag;
extern uint8 island_speed;

extern uint8 island_right_data;
extern uint8 island_left_data;
extern uint8 island_right_dis;

extern uint8 circle_right;      //���һ����趨
extern uint8 circle_left;

extern uint8 CirR_KP;
extern uint8 CirL_KP;

extern int8 start_up_flag;
extern int8 start_time;
extern int32 pulse_stop_count;
extern uint16 RKcount;
extern uint16 RK_flag;
extern uint8  RK_Dis;
extern uint8  RK_Yaw;
extern uint8  CK_flag;
extern uint8  CK_left;
extern uint8  CK_right;

extern uint8 BZ_flag;
extern uint8 BZ_turn;
extern uint16 BZ_Dis;
extern uint16 BZ_DisS;

extern uint8 SC_flag_L;
extern uint8 SC_flag_R;
extern uint8 SC_flag_eL;
extern uint8 SC_flag_eR;
extern uint8 RD_flag;

extern uint8 speed;
extern uint8 speed_max;
extern uint8 speed_min;


extern uint16 RK_tcount;
extern float LBZ;
extern float RBZ;
extern uint16 temp_count;
extern uint8 cir_cir;
extern uint8 fuya_flag;


int16 limit_ab(int16 x, int16 a, int16 b);
void enter_in(void);
void Pid_Deal(Pid_Inc *pid);
void Right_Out_Pwm(void);
void Left_Out_Pwm(void);
void speed_control(void);
void Pid_Inc_Init(void);
void send_data_ary(int16 *array,int8 len);
void Speed_Turn(void);
void speed_control_all(void);
void speed_out(void);
void CK_Start(void);
void RK_Stop(void);
extern int32 all;
extern float KP;
#endif /* CODE_SPEED_CONTROL_H_ */

