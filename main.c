#include "common.h"
#include "include.h"
#include "stdlib.h"
void Init_All(void);
void LED_init(void);
/********************������********************/
void main(void)
{
  Init_All();
  
  while(1) 
  { 
//    Achieve_Gyro();

  }
}  

void Init_All(void)
{
  LED_init();
  InitMPU6050();
  Speed_PID_Init();
  RemoteCMDData_init(&RemoteData);
  Motor_Init();  
  RemoteInit();
  Pulse_acquire_init();
  DisableInterrupts;//��ֹ���ж�   
  PIT_Init();
  Main_ADC_INIT();
  EnableInterrupts;
}

void LED_init(void)
{
  gpio_init  (PTC17,GPO,0);   //D1
  gpio_init  (PTC16,GPO,0);   //D0
  gpio_init  (PTC19,GPO,0);   //DC
  gpio_init  (PTC18,GPO,1);   //RST
  LCD_Init();
}
