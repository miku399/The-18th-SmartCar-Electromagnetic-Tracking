#include "headfile.h"

//float Range_PError[5] = {0 , 25 , 45 , 65 ,100};
//float Range_DError[5] = {0 , 25 , 45 , 65, 100};

//float fuzzy_P[range][5]=
//{
//        {4 , 5 , 7 , 8 , 10},
//        {5 , 6 , 8 , 10 , 12} 
//};
//float fuzzy_D[2][5]=
//{
//        {5 , 6 , 8 , 10 , 12},
//        {9 , 10 , 12 , 13 , 15}
//};
//float Output_tmp_P_right[5] = {2.0 ,3.0 , 3.2 ,3.2,3.2};

//float Output_tmp_D_right[5] = {15, 15,15, 15 , 15};

//float Output_tmp_P_left[5] =  {2.0 ,3.0 , 3.2 ,3.2,3.2};

//float Output_tmp_D_left[5] = {15, 15,15, 15 , 15};

//int P_rule[5][5] =
//{
//    { 0, 1, 2, 3, 4 },
//    { 0, 1, 2, 3, 4 },
//    { 1, 1, 3, 3, 4 },
//    { 1, 2, 3, 4, 4 },
//    { 2, 3, 3, 4, 4 },
//};

//int D_rule[5][5] =
//{
//    { 0, 1, 1, 2, 2 },
//    { 0, 1, 1, 2, 2 },
//    { 1, 1, 2, 3, 3 },
//    { 1, 2, 3, 3, 4 },
//    { 2, 2, 3, 4, 4 },
//};

//void GetFuzzyPD(float Error,float DError)
//{    uint8 x1, y1, x2, y2;
//    float Membership_tmp[4],Membership[4];
//    float Output_tmp[4]={0};
//    float Error_tmp = Error < 0 ? -Error : Error;
//    float DError_tmp = DError < 0 ? -DError : DError;
//      int8 left_flag=0,right_flag=0;
//    if(Error>0)
//        left_flag=1;
//    else
//        right_flag=1;

//    if(Error_tmp <= Range_PError[1])//在Range_PError[0]和Range_PError[1]之间
//    {
//        x1 = 0;
//        x2 = 1;
//        Membership_tmp[1] = 100*(Error_tmp - Range_PError[0])/(Range_PError[1] - Range_PError[0]);//权重计算     Range_PError[0]的隶属度
//        Membership_tmp[0] = 100*(Range_PError[1] - Error_tmp)/(Range_PError[1] - Range_PError[0]);//       Range_PError[1]的隶属度
//    }
//    else if(Error_tmp <= Range_PError[2])//在Range_PError[1]和Range_PError[2]之间
//    {
//        x1 = 1;
//        x2 = 2;
//        Membership_tmp[1] = 100*(Error_tmp - Range_PError[1])/(Range_PError[2] - Range_PError[1]);
//        Membership_tmp[0] = 100*(Range_PError[2] - Error_tmp)/(Range_PError[2] - Range_PError[1]);
//    }
//    else if(Error_tmp <= Range_PError[3])
//    {
//        x1 = 2;
//        x2 = 3;
//        Membership_tmp[1] = 100*(Error_tmp - Range_PError[2])/(Range_PError[3] - Range_PError[2]);
//        Membership_tmp[0] = 100*(Range_PError[3] - Error_tmp)/(Range_PError[3] - Range_PError[2]);
//    }
//    else if(Error_tmp <= Range_PError[4])
//   {
//        x1 = 3;
//        x2 = 4;
//        Membership_tmp[1] = 100*(Error_tmp - Range_PError[3])/(Range_PError[4] - Range_PError[3]);
//        Membership_tmp[0] = 100*(Range_PError[4] - Error_tmp)/(Range_PError[4] - Range_PError[3]);
//   }
//    else
//    {
//        x1 = 4;
//        x2 = 4;
//        Membership_tmp[1] = 100;
//        Membership_tmp[0] = 100;
//    }

//    if(DError_tmp <= Range_DError[1])
//    {
//      y1 = 0;
//      y2 = 1;
//      Membership_tmp[3] = 100*(DError_tmp - Range_DError[0])/(Range_DError[1] - Range_DError[0]);  //  /10
//      Membership_tmp[2] = 100*(Range_DError[1] - DError_tmp)/(Range_DError[1] - Range_DError[0]);
//    }
//    else if(DError_tmp <= Range_DError[2])
//    {
//      y1 = 1;
//      y2 = 2;
//      Membership_tmp[3] = 100*(DError_tmp - Range_DError[1])/(Range_DError[2] - Range_DError[1]);  //  /10
//      Membership_tmp[2] = 100*(Range_DError[2] - DError_tmp)/(Range_DError[2] - Range_DError[1]);
//    }
//    else if(DError_tmp <= Range_DError[3])
//    {
//      y1 = 2;
//      y2 = 3;
//      Membership_tmp[3] = 100*(DError_tmp - Range_DError[2])/(Range_DError[3] - Range_DError[2]);   //  /20
//      Membership_tmp[2] = 100*(Range_DError[3] - DError_tmp)/(Range_DError[3] - Range_DError[2]);
//    }
//    else if(DError_tmp <= Range_DError[4])
//    {
//      y1 = 3;
//      y2 = 4;
//      Membership_tmp[3] = 100*(DError_tmp - Range_DError[3])/(Range_DError[4] - Range_DError[3]);  //  /50
//      Membership_tmp[2] = 100*(Range_DError[4] - DError_tmp)/(Range_DError[4] - Range_DError[3]);
//    }
//    else
//    {
//      y1 = 4;
//      y2 = 4;
//      Membership_tmp[3] = 100;
//      Membership_tmp[2] = 100;
//    }

//        //隶属度取小
//    if(Membership_tmp[0] <= Membership_tmp[2])
//        Membership[0] = Membership_tmp[0];
//    else
//        Membership[0] = Membership_tmp[2];

//    if(Membership_tmp[0] <= Membership_tmp[3])
//        Membership[1] = Membership_tmp[0];
//    else
//        Membership[1] = Membership_tmp[3];

//    if(Membership_tmp[1] <= Membership_tmp[2])
//        Membership[2] = Membership_tmp[1];
//    else
//        Membership[2] = Membership_tmp[2];

//    if(Membership_tmp[1] <= Membership_tmp[3])
//        Membership[3] = Membership_tmp[1];
//    else
//        Membership[3] = Membership_tmp[3];

//    if(right_flag==1)
//    {
//        Output_tmp[0] = Output_tmp_P_right[P_rule[y1][x1]];
//        Output_tmp[1] = Output_tmp_P_right[P_rule[y2][x1]];
//        Output_tmp[2] = Output_tmp_P_right[P_rule[y1][x2]];
//        Output_tmp[3] = Output_tmp_P_right[P_rule[y2][x2]];
//    }
//    else if(left_flag == 1)
//    {
//        Output_tmp[0] = Output_tmp_P_left[P_rule[y1][x1]];
//        Output_tmp[1] = Output_tmp_P_left[P_rule[y2][x1]];
//        Output_tmp[2] = Output_tmp_P_left[P_rule[y1][x2]];
//        Output_tmp[3] = Output_tmp_P_left[P_rule[y2][x2]];
//    }
//    ele_servo.P = (Output_tmp[0] * Membership[0] + Output_tmp[1] * Membership[1] + Output_tmp[2] * Membership[2] + Output_tmp[3] * Membership[3])/(Membership[0] + Membership[1] + Membership[2] + Membership[3]);
//    if(right_flag==1)
//    {
//        Output_tmp[0] = Output_tmp_D_right[D_rule[x1][y1]];
//        Output_tmp[1] = Output_tmp_D_right[D_rule[x2][y1]];
//        Output_tmp[2] = Output_tmp_D_right[D_rule[x1][y2]];
//        Output_tmp[3] = Output_tmp_D_right[D_rule[x2][y2]];
//    }
//    else if(left_flag == 1)
//    {
//        Output_tmp[0] = Output_tmp_D_left[D_rule[x1][y1]];
//        Output_tmp[1] = Output_tmp_D_left[D_rule[x2][y1]];
//        Output_tmp[2] = Output_tmp_D_left[D_rule[x1][y2]];
//        Output_tmp[3] = Output_tmp_D_left[D_rule[x2][y2]];
//    }
//    ele_servo.D = (Output_tmp[0] * Membership[0] + Output_tmp[1] * Membership[1] + Output_tmp[2] * Membership[2] + Output_tmp[3] * Membership[3])/(Membership[0] + Membership[1] + Membership[2] + Membership[3]);
//		 if(flag_SC==2||flag_SC==5)
//		 {
//				Output_tmp_P_right[0] = 2.4; Output_tmp_P_right[1] =2.8;Output_tmp_P_right[2] = 3.4;Output_tmp_P_right[3] = 3.5;Output_tmp_P_right[5] =3.8;

//				Output_tmp_D_right[0] =30;Output_tmp_D_right[1] =35;Output_tmp_D_right[2] =38;Output_tmp_D_right[3] =37;Output_tmp_D_right[4] =45;

//				Output_tmp_P_left[0] = 2.4;Output_tmp_P_left[1] = 2.8;Output_tmp_P_left[2] =3.4;Output_tmp_P_left[3] =3.5;Output_tmp_P_left[4] = 3.8;

//				Output_tmp_D_left[0] = 30;Output_tmp_D_left[1] = 35;	Output_tmp_D_left[2] = 38;	Output_tmp_D_left[3] =37;	Output_tmp_D_left[4] = 45;		
//				 				 
//		 }
//		 else
//		{
//				Output_tmp_P_right[0] = 2.0; Output_tmp_P_right[1] =2.6;Output_tmp_P_right[2] = 3;Output_tmp_P_right[3] = 3.5;Output_tmp_P_right[5] =3.8;

//				Output_tmp_D_right[0] =30;Output_tmp_D_right[1] =32;Output_tmp_D_right[2] =47;Output_tmp_D_right[3] =50;Output_tmp_D_right[4] =55;

//				Output_tmp_P_left[0] = 2.0;Output_tmp_P_left[1] = 2.6;Output_tmp_P_left[2] =3;Output_tmp_P_left[3] =3.5;Output_tmp_P_left[4] = 3.8;

//				Output_tmp_D_left[0] = 30;Output_tmp_D_left[1] = 32;	Output_tmp_D_left[2] = 47;	Output_tmp_D_left[3] =50;	Output_tmp_D_left[4] = 55;					
//		 }
//}
