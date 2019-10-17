# include "include.h"
unsigned char StartReceive = 0;
unsigned char ReceiveIndex = 0;
uint8 ReceiveBuff[12];
uint8 Speed_Now[4];

uint8 Speed_Target[4]={5000,5000,5000,5000};

uint8 piancha_Last[4]={0};
uint8 piancha_Now[4]={0};
uint8 jifeng[4]={0};
uint8 jifeng_i[4];
uint8 PWM[4];
uint8 Send[14]={0xFF,0,0,0,0,0,0,0,0,0,0,0,0,0xFE};

///<summary>速度环参数</summary>
float P_Set[4] = {30, 30, 30, 30};               //{33 31 33 30}
float D_Set[4] = {10, 10, 10, 10};               //{12 10 11 10}
float I_Set[4] = {5, 5, 5, 5};                   //{5 5 5 5}
float DeadBand_Set[4] = {0, 0, 0, 0};            //{700, 680, 680, 710}
float I_limit = 8000;
float Max_output = 9500;
float Pout, Dout, Iout;                          
//电机初始化
void Motor_init(void)
{
	FTM_PWM_init(FTM0, FTM_CH1, 10000, 10000);//电机
        FTM_PWM_init(FTM0, FTM_CH2, 10000, 10000);//电机
        FTM_PWM_init(FTM0, FTM_CH3, 10000, 10000);//电机
        FTM_PWM_init(FTM0, FTM_CH4, 10000, 10000);//电机
	FTM_PWM_init(FTM1, FTM_CH0, 200, 2800);//舵机

	
}
//从UART4中收包
void Receive_Speed()
{
    uint8 buff = 0;
    uart_getchar (UART4,&buff);
     if (buff == 0xFF && StartReceive == 0)
    {
        StartReceive = 1;
        return;
    }
    else
    {
        if (StartReceive == 0)
            ReceiveIndex = 0;
    }
      if (StartReceive == 1)
    {
        if (ReceiveIndex < 12)
        {
            ReceiveBuff[ReceiveIndex] = buff;
            ReceiveIndex++;
        }
        else
        {
          if(buff==0xFE)
          {
            StartReceive=0;
            ReceiveIndex=0;
          }
        }
    }
    
}

//把包转为10进制
void turn_Packet()
{
  int j=0;
  for(int i=0; i<12; )
  {
      if(ReceiveBuff[i]==0)
        Speed_Now[j]=-(ReceiveBuff[i+1]*256+ReceiveBuff[i+2]);
      else
        Speed_Now[j]=ReceiveBuff[i+1]*256+ReceiveBuff[i+2];
      j++;
      i=i+3;
  }
}
//记录上次偏差
void pianchalast()
{
   for(int i=0; i<4; i++)
   {
  piancha_Last[i]=piancha_Now[i];
   }
}
//计算现在的偏差
void pianchanow()
{
 for(int i=0; i<4; i++)
 {
   piancha_Now[i]=Speed_Target[i]-Speed_Now[i];
 }
}
//计算输出占空比
void PID_Control()
{
  for(int i=0; i<4; i++)
  {
    jifeng[i]=jifeng[i]+piancha_Now[i];
    jifeng_i[i]=jifeng[i]*I_Set[i];
    if(jifeng_i[i]>I_limit)
    {
      jifeng_i[i]=I_limit;
    }
  }
  for(int j=0; j<4;j++)
  {
    if(Speed_Target[j]<=0)
      PWM[j]=-(piancha_Now[j]*P_Set[j]+D_Set[j]*(piancha_Now[j]-piancha_Last[j])+jifeng_i[j]);
    else
      PWM[j]=piancha_Now[j]*P_Set[j]+D_Set[j]*(piancha_Now[j]-piancha_Last[j])+jifeng_i[j];
  }
  for(int k=0; k<4; k++)
  {
    if(PWM[k]>Max_output)
      PWM[k]=Max_output;
        if(PWM[k]<-Max_output)
          PWM[k]=-Max_output;
  }
}
//换算要发出的包
void Send_Packet()
{
  int j=1,k=2,i=0;
  for(i=0; i<4; i++ )
  {
    if(PWM[i]<0)
    {
      Send[j]=0;
    }
  else
  {
    Send[j]=1;
  }
    j=j+3;
  Send[k]=abs(PWM[i])/256;
  Send[k+1]=abs(PWM[i])%256;
  k=k+3;
}

}
void send()
{
    for(int i=0;i<14;i++)
      uart_putchar(UART4,Send[i]);
}

