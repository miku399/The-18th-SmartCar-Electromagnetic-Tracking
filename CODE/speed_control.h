#include "headfile.h"
#include "RG_Type_Define.h"
#include "RG_Picture_Processing.h"
#include "RG_Math.h"
#include "RG_Determine_Type.h"
#include "RG_Data_Define.h"
#include "headfile.h"
#include "RG_Control_Car.h"
#include "RG_Border_Searching.h"

void KEY_Init()
{
    gpio_init(KEY1, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY2, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY3, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(KEY4, GPI, 1, GPIO_PIN_CONFIG);
    gpio_init(BKEY1, GPI, 1, GPIO_PIN_CONFIG);  //�����1λ��
    gpio_init(BKEY2, GPI, 1, GPIO_PIN_CONFIG);  //�����2λ��
    gpio_init(BKEY3, GPI, 1, GPIO_PIN_CONFIG);  //�����3λ��
    gpio_init(BKEY4, GPI, 1, GPIO_PIN_CONFIG);  //�����4λ��
}
uint8 KEY_Read(uint8 mode)
{
    static uint8_t key_up=1;     //�����ɿ���־
    if(mode==1)
    {
        key_up=1;    //֧������
    }
    if(key_up && ((0==gpio_get(KEY1)) || (0==gpio_get(KEY2)) || (0==gpio_get(KEY3)) || (0==gpio_get(KEY4))))
    {
        systick_delay_ms(10);   //����
        key_up=0;
        if(gpio_get(KEY1)==0)
        {
            return 1;
        }
        else if(gpio_get(KEY2)==0)
        {
            return 2;
        }
        else if(gpio_get(KEY3)==0)
        {
            return 3;
        }
        else if(gpio_get(KEY4)==0)
        {
            return 4;
        }
    }
    if(1==gpio_get(KEY1) && 1==gpio_get(KEY2) && 1==gpio_get(KEY3) && 1==gpio_get(KEY4))
    {
        key_up=1;
    }
    return 0;   //�ް�������
}

/*==================================================================
* ��������  void Get_Err_SG(Steer_PID_Info* Steer,_Border_ *Border)
* ���ߣ�  SG
* ���ڣ�   2020-08-15
* ���ܣ���ȡƫ��
* ���������Steer��������ƽṹ��
*           Border���߽�ṹ�壬��������������ͼ��Ĳ���
*       
* ����ֵ��void  
*
* �޸ļ�¼��
===================================================================*/

float CirErr[IMGY]={78.8f, 21.2f, 22.4f, 23.6f, 24.8f, 26.0f, 27.2f, 28.4f, 29.6f, 30.8f, 32.0f, 33.2f, 34.4f, 35.6f, 36.8f, 38.0f, 39.2f, 40.4f, 41.6f, 42.8f, 44.0f, 45.2f, 46.4f, 47.6f, 48.8f, 50.0f, 51.2f, 52.4f, 53.6f, 54.8f, 56.0f, 57.2f, 58.4f, 59.6f, 60.8f, 62.0f, 63.2f, 64.4f, 65.6f, 66.8f, 68.0f, 69.2f, 70.4f, 71.6f, 72.8f, 74.0f, 75.2f, 76.4f, 77.6f, 78.8f,};
void Get_Err_SG(Steer_PID_Info* Steer,_Border_ *Border)
{
    int lineMax,lineMin,count=0,curveCount=0;
    if(Border->LCircle==3 || Border->RCircle==3)  //����ǰհ����������������ƻ����Ƿ�����
    {
        lineMax=26;
        lineMin=lineMax-5;        
    }
    else if(Border->Straight) //ֱ��ǰհ
    {
        lineMax=(int)(Border->StraightROW);
        lineMin=lineMax-5;
    }
    else if(Border->Angle) //���ǰհ
    {
        lineMax=(int)(Border->AngleROW);
        lineMin=lineMax-5;
    }
    //��������ƫ�������������ƫ�ƣ����ߵ�����ʵ�֣����Ȩ��������ƫ����ֻ�ǻ�ȡ��
    for (int y=5; y<Use_ROWS; ++y,++curveCount)
    {
        if (y>=lineMin&&y<=(lineMax+10)&&count<6)
        {
            if (Border->CPnt[y].y)
            {
                Steer->offPoint[count].x=Border->CPnt[y].x;
                Steer->offPoint[count++].y=Border->CPnt[y].y;
            }
        }
    }
    //��ʷƫ���
    Steer->Error[2]=Steer->Error[1];
    Steer->Error[1]=Steer->Error[0];
    static uint8 Lost=0;
    //������ߵĵ���̫�٣�˵������ƫ������̫�࣬�����Ҫ���磬���Է��������ƫ�ֱ�����ϴε�ƫ��
    if(Border->LNum<15 && Border->RNum<15)
    {
       Steer->Error[0]=Steer->Error[0]*1.0f; //�����1.0���Էŵ�1.15���߸��󣬵����������1.0�ǵÿ��ƴ�������Ȼƫ���һֱ���ۣ��������ƫ����Ʋ��٣����С����ʧ�ص�
    }
    else
    {
       if (count<=1 && Lost==0) //���Ƶ�̫��Ҳ����֮ǰ��ƫ��
       {
           Lost=1; //���ǵ��ο��Ƶı�־λ��ӦΪ֮ǰ�Ĳ���BUG��*�Ĳ���1.0��ƫ��ͻ�һֱ�������С
           Steer->Error[0]=Steer->Error[0]*1.0f;
       }
       else
       {
           //������Ǽ���ƫ��
           Lost=0;
           Steer->Error[0]=0;
           for (int y=0; y<count; ++y)
           {
               if (Steer->offPoint[y].y)
               {
                  if(Border->LCircle==2)  //�������뻷��֮ǰ���л�����
                   Steer->Error[0]+=ErrAllot[count][y]*(Steer->offPoint[y].x-Use_Line/2-15);
                  else if(Border->RCircle==2)
                  Steer->Error[0]+=ErrAllot[count][y]*(Steer->offPoint[y].x-Use_Line/2+15);
                  else  //�����������ƫ��
                  Steer->Error[0]+=ErrAllot[count][y]*(Steer->offPoint[y].x-Use_Line/2);
               }
           }
       }
    }
    if(Border->LCircle==4 || Border->RCircle==4)  //����ֱ����֮ǰ�ڻ����ڲ���ƫ�������ok�����ò���
    {
       Steer->Error[0]=(double)(Border->CircleError/Border->CircleErrorNum)*1.0f;
    }
    else if(Border->OutCarport==1) //�����ȡƫ����⣨�ٶȺ�ƫ���������ã�
    {
       if(!Border->OutL_or_R) //1Ϊ���
       Steer->Error[0]=-IncomingError[20];
       else 
       Steer->Error[0]=IncomingError[20];
    }
    GetIncoming(Border,Steer,Border->OutL_or_R);  //ͣ��ƫ���ȡ
}

extern uint32 LSeepdNum;
extern uint32 RSeepdNum;
/*==================================================================
* ��������  void Steer_Control()
* ���ߣ�  SG
* ���ڣ�   2020-08-15
* ���ܣ����ƶ�����
* ���������Steer��������ƽṹ��
*       
* ����ֵ��void  
*
* �޸ļ�¼��
===================================================================*/

void Steer_Control()
{
    static float out = 0;
    int ou=0;
    /*���ƾ��߲�*/
    Steer.EK[2]=Steer.EK[1];
    Steer.EK[1]=Steer.EK[0];
    Steer.EK[0]=Steer.Error[0]-Steer.Error[1];
    Steer.KP=(Steer.P*fabs(Steer.Error[0]*Steer.Error[0])/2000+Steer.ZP);//����P�������Լ���
    if(Border.Straight)
    {
       Steer.KP=Steer.DC_ZP;
           out=Steer.Error[0]*Steer.KP+ Steer.D1*Steer.EK[0]+Steer.D2*(Steer.EK[0]-Steer.EK[1]);
    }
    else if(Border.Angle)
    {
       Steer.KP=Steer.DC_ZP+3.0;
           out=Steer.Error[0]*Steer.KP+ (Steer.D1+15)*Steer.EK[0]+Steer.D2*(Steer.EK[0]-Steer.EK[1]);
    }
    else
    {
       Steer.KP=Steer.DC_ZP;
           out=Steer.Error[0]*Steer.KP+ Steer.D1*Steer.EK[0]+Steer.D2*(Steer.EK[0]-Steer.EK[1]);
    }
    //�������

    //�޷�����
    if (out<-1000)
    {
        out=-1000;
    }
    if (out>1000)
    {
        out=1000;
    }
    Steer.PwmOut=6910-out;
    //���²�����ʱ���ģ�ȥ����λ����ΪӲ��ԭ��
    ou = (int)(6910-(int)out);
    ou=(int)(ou/10)*10;
    if(Steer.DC_Or_Cam==0) //�������ͷ����
    pwm_duty(PWM1_MODULE0_CHA_C26,ou);
}

//�������
void Motor_OutL(int out)
{
    if(0<=out) //�ҵ��   ��ת
    {
        pwm_duty(PWM2_MODULE0_CHB_D7, out);
        pwm_duty(PWM2_MODULE0_CHA_D6, 0);
    }
    else                //�ҵ��   ��ת
    {
        pwm_duty(PWM2_MODULE0_CHB_D7, 0);
        pwm_duty(PWM2_MODULE0_CHA_D6, -out);
    }
}
void Motor_OutR(int out)
{
    if(0<=out) //����   ��ת ����ռ�ձ�Ϊ �ٷ�֮ (1000/TIMER1_PWM_DUTY_MAX*100)
    {
        pwm_duty(PWM2_MODULE1_CHB_D5, 0);
        pwm_duty(PWM2_MODULE1_CHA_D4, out);
    }
    else                //����   ��ת
    {
        pwm_duty(PWM2_MODULE1_CHB_D5, -out);
        pwm_duty(PWM2_MODULE1_CHA_D4, 0);
    }
}
//�ٶȿ�������ʽPI
void Speed_Rcontrol()
{
    static int duty,last_duty;
    static float Proportion,Integral,Derivative;
    motorR.ek[2]=motorR.ek[1];
    motorR.ek[1]=motorR.ek[0];
    motorR.ek[0]=motorR.I_speed[0]-motorR.AC;
    Proportion=(motorR.ek[0]-motorR.ek[1])*motorR.MP;
    Integral=(motorR.ek[0]*motorR.MI)/10.0f;
    Derivative=(motorR.ek[0]-2*motorR.ek[1]+motorR.ek[2])*motorR.MD;
    duty=(int)(motorR.Kvff*(motorR.I_speed[0]-motorR.I_speed[1])+
               Proportion+Integral+Derivative);
    /*-----------�����������ֿ�����-------------*/
    if (last_duty>=MAX_PWM)   if (duty>0)
        {
            duty=0;
        }
    if (last_duty<=MIN_PWM)   if (duty<0)
        {
            duty=0;
        }
    motorR.out+=duty;
    if (motorR.out>=MAX_PWM)
    {
        motorR.out=MAX_PWM;
    }
    if (motorR.out<=MIN_PWM)
    {
        motorR.out=MIN_PWM;
    }
    last_duty=duty;
    motorR.I_speed[1]=motorR.I_speed[0];
    Motor_OutR(motorR.out);
}
void Speed_Lcontrol()
{
    static int motorLduty,motorLlast_duty;
    static float ProportionL,IntegralL,DerivativeL;
    motorL.ek[2]=motorL.ek[1];
    motorL.ek[1]=motorL.ek[0];
    motorL.ek[0]=motorL.I_speed[0]-motorL.AB;
    ProportionL=(motorL.ek[ 0]-motorL.ek[1])*motorL.MP;
    IntegralL=(motorL.ek[0]*motorL.MI)/10.0f;
    DerivativeL=(motorL.ek[0]-2*motorL.ek[1]+motorL.ek[2])*motorL.MD;
    motorLduty=(int)(motorL.Kvff*(motorL.I_speed[0]-motorL.I_speed[1])+
                     (int)(ProportionL+IntegralL+DerivativeL));
    /*-----------�����������ֿ�����-------------*/
    if (motorLlast_duty>=MAX_PWM) if (motorLduty>0)
        {
            motorLduty=0;
        }
    if (motorLlast_duty<=MIN_PWM) if (motorLduty<0)
        {
            motorLduty=0;
        }
    motorL.out+=motorLduty;
    if (motorL.out>=MAX_PWM)
    {
        motorL.out=MAX_PWM;
    }
    if (motorL.out<=MIN_PWM)
    {
        motorL.out=MIN_PWM;
    }
    motorLlast_duty=motorLduty;
    motorL.I_speed[1]=motorL.I_speed[0];
    Motor_OutL(motorL.out);
}


/*==================================================================
* ��������void GetISpeed(Steer_PID_Info *Steer,_Border_ *Border)
* ���ߣ�  SG
* ���ڣ�   2020-08-15
* ���ܣ���ȡ�����ٶ�
* ���������Steer��������ƽṹ��
*       
* ����ֵ��void  
*
* �޸ļ�¼��
===================================================================*/
int32    Angle;
int16    a;
extern uint8 PICTURE_USE2[Use_ROWS][Use_Line];
void GetISpeed(Steer_PID_Info *Steer,_Border_ *Border)
{
    int TheoryStraightSpeed=0,TheoryAngleSpeed=0;
    
    //�˴�����Ӧ������һ�ηǳ���ȷ���ٶȿ��Ƶģ����鲻�ã��ͽ��͵�ֱ�Ӹ����ٶ��ˣ���Ȼ���������ܲ�������ٶȵ�      

    /*����ֱ������ٶȣ�����ʵ�ʸ�ֵ�ٶȵ�ʱ������ֱ���ٶȻ�������ٶ�������ٶ��趨*/
    /*if(ADL2<50&&ADL1<50 && ADR1<50 && ADR2<50 && Border->StopCar==0) //��Ŷ��߱����������û�е�ţ��������������ͷ���߱���
    {
           Border->StartCar = 1; //ͣ��
           ips114_showint16(0,4,Border->RCircle);  //������ʾ�����׶�
           ips114_showint16(0,5,Border->LCircle);
    }
    else */
    int BNUM=0,BROW=0;
    for(uint8 i=IMGY-20;i<IMGY;i++)
    {
        BNUM=0;
        for(uint8 j=0;j<IMGX;j++)
        {
           if(PICTURE_USE2[i][j]==B_BLACK)
           BNUM++;
        }
        if(BNUM>IMGX-40)
        BROW++;
        if(BROW>12)
        {
           Border->StartCar = 1; //ͣ��
           break;
        }
    }
    /*if(Border->OutCarport==1)  //�����ٶ�
    {
        TheoryStraightSpeed = 25,TheoryAngleSpeed=25;   
    }
    else if(Border->StopCar==1) //ͣ��һ�׶�ά��30���ٶ�
    {
        TheoryStraightSpeed = 30,TheoryAngleSpeed=30;
    }
    else if(Border->StopCar==2) //ͣ�����׶�ά��0���ٶ�
    {
        TheoryStraightSpeed = 0,TheoryAngleSpeed=0;Border->StartCar = 1;  
    }
    else if(Border->Ramp!=0 && Border->Ramp<=3) //�µ�����
    {
        TheoryStraightSpeed = (int)(Steer->StraightSpeed/2),TheoryAngleSpeed=(int)(Steer->StraightSpeed/2);
    }
    else if(Border->LCircle==1 || Border->RCircle==1) //�����뻷���٣�����ʱ�������ٶ�
    {
        TheoryStraightSpeed = 34,TheoryAngleSpeed=34;
    }
    else
    {
        TheoryStraightSpeed = (int)(Steer->StraightSpeed);
        if(Border->Angle==1)
            TheoryAngleSpeed=(int)(Steer->AngleSpeed-10);
        else if(Border->Angle==2)
           TheoryAngleSpeed=(int)(Steer->AngleSpeed+15);
    }*/
        TheoryStraightSpeed = (int)(Steer->StraightSpeed);
        if(Border->Angle==1)
            TheoryAngleSpeed=(int)(Steer->AngleSpeed-8);
        else if(Border->Angle==2)
           TheoryAngleSpeed=(int)(Steer->AngleSpeed+5);
//if(Border->Straight || Border->Ramp)
//{
//   TheoryAngleSpeed=TheoryStraightSpeed;
//}
    /*���û�ж���ͣ����־λ����ʼ�����ٶȸ�ֵ*/
    if(0==Border->StartCar)
    {     
        if(Border->Straight==1 || Border->Ramp) //�µ���ֹ����
        {
            motorL.I_speed[0] = TheoryStraightSpeed;
            motorR.I_speed[0] = TheoryStraightSpeed;
        }
        else if(Border->Straight==2 || Border->Ramp) //�µ���ֹ����
        {
            motorL.I_speed[0] = TheoryStraightSpeed+6;
            motorR.I_speed[0] = TheoryStraightSpeed+6;
        }
        else if(Border->Angle==1 || Border->StopCar==1)   //������٣����ת�ǲ��٣�
        {
            Angle = 6953 - (int)Steer->PwmOut;  //<0 ���  >0�ұߣ���ֲ�ģ���Ϊ����д���������ּ����ٹ�
           if(Angle>0)         //�ҹ�
           {
              a=(int16)(45*(Angle)*1.0/1000) ;
              if(a>45)  a=45;
              if(a<0)   a=0;
              //�����������ĸ���ǿ��ת��������㣬��Ȼ����ô��������0.0��
              double m=(double)(((double)TheoryAngleSpeed/(double)45)*(double)a*(double)Steer->CS_Val);
              motorL.I_speed[0] = TheoryAngleSpeed;
              motorR.I_speed[0] = (int)(TheoryAngleSpeed - m); 
           }
           else              //���
           {
              a=(int16)(45*(-Angle)*1.0/1000) ;
              if(a>45)  a=45;
              if(a<0)   a=0;
              double m=(double)(((double)TheoryAngleSpeed/(double)45)*(double)a*(double)Steer->CS_Val);
              motorL.I_speed[0] = (int)(TheoryAngleSpeed - m);
              motorR.I_speed[0] = TheoryAngleSpeed; 
        
           }
        }
        else if(Border->Angle==2 || Border->StopCar==1)   //������٣����ת�ǲ��٣�
        {
            Angle = 6953 - (int)Steer->PwmOut;  //<0 ���  >0�ұߣ���ֲ�ģ���Ϊ����д���������ּ����ٹ�
           if(Angle>0)         //�ҹ�
           {
              a=(int16)(45*(Angle)*1.0/1000) ;
              if(a>45)  a=45;
              if(a<0)   a=0;
              //�����������ĸ���ǿ��ת��������㣬��Ȼ����ô��������0.0��
              double m=(double)(((double)TheoryAngleSpeed/(double)45)*(double)a*(double)(Steer->CS_Val-0.02f));
              motorL.I_speed[0] = TheoryAngleSpeed;
              motorR.I_speed[0] = (int)(TheoryAngleSpeed - m); 
           }
           else              //���
           {
              a=(int16)(45*(-Angle)*1.0/1000) ;
              if(a>45)  a=45;
              if(a<0)   a=0;
              double m=(double)(((double)TheoryAngleSpeed/(double)45)*(double)a*(double)(Steer->CS_Val-0.02f));
              motorL.I_speed[0] = (int)(TheoryAngleSpeed - m);
              motorR.I_speed[0] = TheoryAngleSpeed; 
        
           }
        }
    }
    else
    {
        motorL.I_speed[0] = 0;
        motorR.I_speed[0] = 0;
    }
}

/*==================================================================
* ��������  void GetControlDecision(Steer_PID_Info *Steer,_Border_ *Border)
* ���ߣ�  SG
* ���ڣ�   2020-08-15
* ���ܣ���Ż�������ͷ���Ƶ�Ȩ�޻�ȡ
* ���������Steer��������ƽṹ��
*           Border���߽�ṹ�壬��������������ͼ��Ĳ���
*           
* ����ֵ��void  
*
* �޸ļ�¼��ΪʲôҪ�õ���أ���Ϊ���ô���ѽ����͵����һ������
===================================================================*/
void GetControlDecision(Steer_PID_Info *Steer,_Border_ *Border)
{
   if((Border->Ramp>0 && Border->Ramp<4) || Border->Cross)//���ô���ʮ�֣�����ʶ��֮��ֱ���л���ţ�����
   {
      Steer->DC_Or_Cam=1; //���
   }
   else
   {
      Steer->DC_Or_Cam=0;  //����ͷ
   }
}



//һ�����ҵ�һλ��������ڲ���ʱ��д�ĵ�ſ��ƣ��¾�û��ɾ���������ͣ��ܼ�𣬷����洫��Ŷ��ܣ���ʲô�Լ��£���
#define NM 3
double erro[3],out_Dz,ek[3],Sout;
static uint16 ave[5],sum[5],AD[5][5],temp,min[5],max[5],AD_sum[5],AD_V[5][3],ave2[5];
float sum1,sum2,sum3,sum4,guiyi[5],AD_guiyi2[5],ADave2[5],ws[3];
void DC_Control()
{
    int i,j,k;
    min[0]=290;
    min[1]=290;
    min[2]=290;
    min[3]=290;
    min[4]=290;
    max[0]=1900;
    max[1]=1900;
    max[2]=1900;
    max[3]=1900;
    max[4]=1900;
    for(uint8_t i=0; i<5; i++)
    {
        AD[0][i]=ADL1;
        AD[1][i]=ADM0;
        AD[2][i]=ADR1;
        AD[3][i]=ADL2;
        AD[4][i]=ADR2;
    }
    for(i=0; i<5; i++)   //5�����
    {
        for(j=0; j<4; j++) //�����������
        {
            for(k=0; k<4-j; k++)
            {
                if(AD[i][k] > AD[i][k+1])  //ǰ��ıȺ���Ĵ�  ����н���
                {
                    temp = AD[i][k+1];
                    AD[i][k+1] = AD[i][k];
                    AD[i][k] = temp;
                }
            }
        }
    }
    for(i=0; i<5; i++)  //���м�����ĺ�
    {
        sum[i] = AD[i][1] + AD[i][2] + AD[i][3];
        ave[i] = sum[i] / 3;
    }
    for(i = 0; i < NM-1; i ++)
    {
        AD_V[0][i] = AD_V[0][i + 1];
        AD_V[1][i] = AD_V[1][i + 1];
        AD_V[2][i] = AD_V[2][i + 1];
        AD_V[3][i] = AD_V[3][i + 1];
        AD_V[4][i] = AD_V[4][i + 1];
    }
    for(i=0; i<5; i++)
    {
        AD_V[i][NM-1] =ave[i];
    }
    for(i = 0; i < NM; i ++)
    {
        AD_sum[0] += AD_V[0][i];
        AD_sum[1] += AD_V[1][i];
        AD_sum[2] += AD_V[2][i];
        AD_sum[3] += AD_V[3][i];
        AD_sum[4] += AD_V[4][i];
    }
    for(i=0; i<5; i++) //��ƽ��
    {
        ave2[i] = AD_sum[i]/NM;
        AD_sum[i] = 0;
    }
    for (uint8_t i=0; i<5; i++)
    {
        if( ave2[i]<min[i])
        {
            ave2[i]=min[i];
        }
        else if( ave2[i]>max[i])
        {
            ave2[i]=max[i];
        }
        guiyi[i]=(ave2[i]-min[i])*100/(max[i]-min[i]);
    }
    sum1=guiyi[0]-guiyi[2];
    sum2=guiyi[0]+guiyi[2];
    sum3=guiyi[4]-guiyi[3];
    sum4=fabs(guiyi[4]+guiyi[3]);
    Steer.DC_Error[2]=Steer.DC_Error[1];
    Steer.DC_Error[1]=Steer.DC_Error[0];
    Steer.DC_Error[0]=(float)(sum1)/(sum2)*70;
    Steer.DC_KP=(Steer.DC_P*fabs(Steer.DC_Error[0]*Steer.DC_Error[0])/2000+Steer.DC_ZP);
    Steer.DC_EK[1]=Steer.DC_EK[0];
    Steer.DC_EK[0]=Steer.DC_Error[0]-Steer.DC_Error[1];
    Sout=Steer.DC_Error[0]*Steer.DC_KP+ Steer.DC_D1*Steer.DC_EK[0]+Steer.DC_D2*(Steer.DC_EK[0]-Steer.DC_EK[1]);
    if (Sout<-1000)//p=7.16013,d=4.75
    {
        Sout=-1000;
    }
    if (Sout>1000)
    {
        Sout=1000;
    }
    Steer.DC_PwmOut=Sout;
    if(Steer.DC_Or_Cam==1)
    {
        pwm_duty(PWM1_MODULE0_CHA_C26,(int)(6953+Sout));
    }
}