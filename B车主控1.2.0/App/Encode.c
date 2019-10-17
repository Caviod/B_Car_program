#include "include.h"

/*******************************************************************************
                              编码器FTM初始化
*******************************************************************************/
//正交解码初始化
void Pulse_acquire_init(void)//内部初始化调用
{
  FTM_QUAD_Init(FTM2);
  gpio_init(PTA17,GPI,0);
}

/*******************************************************************************
                              编码器速度采集
*******************************************************************************/
int Encoder_Speed = 0;
int Revolve_Flag = 0;
int Revolve_Speed = 0;  //整车速度
void Pulse_acquire(void)    //中断函数中调用
{
  Encoder_Speed = FTM_QUAD_get(FTM2);
  Revolve_Flag = gpio_get(PTA17);
  if(Revolve_Flag == 0)
  {
    Revolve_Speed = -Encoder_Speed;
  }
  else if(Revolve_Flag ==1)
  {
    Revolve_Speed = Encoder_Speed;
  }
  FTM_QUAD_clean(FTM2);
}