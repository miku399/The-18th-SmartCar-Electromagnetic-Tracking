#include "headfile.h"
#include "math.h"

float StrToDouble(const char *s)
{
	
	int i = 0;
	int k = 0;
	float j;
	int flag =1;
	float result = 0.0;
	if (s[i] == '+')
	{
		i++;
	}
	if (s[i] == '-')
	{
		i++;
		flag = -1;
	}
	while (s[i] != '\0' && s[i] != '.')
	{
		j = (s[i] - '0')*1.0;
		result = result * 10 + j;
		i++;
	}
	if (s[i] == '.')
	{
		i++;
		while (s[i] != '\0'&&s[i] != ' ')
		{
			k++;
			j = s[i] - '0';
			result = result + (1.0 * j) / pow(10.0, k);   
			i++;
		}
	}
	result = flag * result;
	return result;
}

void extern_iap_write_float(double dat,uint8 num,uint8 pointnum,uint16 addr)
{
  uint8   length;
	int8    buff[34];
	int8    start,end,point;

	if(0>dat)   length = zf_sprintf( &buff[0],"%f",dat);//����
	else
	{
		length = zf_sprintf( &buff[1],"%f",dat);
		length++;
	}
	point = length - 7;         //����С����λ��
	start = point - num - 1;    //������ʼλ
	end = point + pointnum + 1; //�������λ
	while(0>start)//����λ����  ĩβӦ�����ո�
	{
		buff[end] = ' ';
		end++;
		start++;
	}
    
	if(0>dat)   buff[start] = '-';
  else        buff[start] = '+';
    
	buff[end] = '\0';

	extern_iap_write_bytes(addr,(uint8 *)buff,num+pointnum+3);
}

float iap_read_float(uint8 len, uint16 addr)
{
	uint8 buf[34];
	iap_read_bytes(addr, buf, len);
	
	return StrToDouble(buf);
}
