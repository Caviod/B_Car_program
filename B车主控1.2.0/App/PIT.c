#include "include.h"
#include "PIT.h"

void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void PIT_Init();

float a=0;
void PIT_Init()
{
  pit_init_ms(PIT0, 10); 
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数
  enable_irq (PIT0_IRQn);       
  pit_init_ms(PIT1, 20); 
  set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //设置PIT0的中断服务函数
  enable_irq (PIT1_IRQn);
}

void PIT0_IRQHandler(void)
{
  GetRemoteCMDData(); //遥控器发送
  Pulse_acquire();    //编码器采集
  RemoteControl();    //遥控控制速度
  #if 1
    AD_Collect();
    LCD_Show_Number (0,0,Plane_AD[0]);
    LCD_Show_Number (40,0,Plane_AD[1]);
    LCD_Show_Number (80,0,Plane_AD[2]);   
#endif

  PIT_Flag_Clear(PIT0);       //清中断标志位 
}

void PIT1_IRQHandler(void)
{
  PIT_Flag_Clear(PIT1);       //清中断标志位 
}
