void PIDInit();
extern float pout,pout0;
typedef struct
{
    float P;
    float I;
    float D;
} PID_CLASS;

extern float r_setspeed;
extern float Speed;
extern float Curve;
extern float Stright;
extern float L_CarSpeed,R_CarSpeed;

extern uint8 Str_P;            //°´¼üµ÷²Î
extern uint8 Str_D;
extern uint8 Cur_P;
extern uint8 Cur_D;
extern uint8 Str_KP;
extern uint8 Cur_KP;
extern uint8 RD_KP;

