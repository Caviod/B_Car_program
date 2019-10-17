#include "include.h"
#include "PIT.h"

void PIT0_IRQHandler(void);
void PIT1_IRQHandler(void);
void PIT_Init();

void PIT_Init()
{
  pit_init_ms(PIT0, 10); 
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϷ�����
  enable_irq (PIT0_IRQn);       
  pit_init_ms(PIT1, 20); 
  set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //����PIT0���жϷ�����
  enable_irq (PIT1_IRQn);
}

void PIT0_IRQHandler(void)
{
//  GetRemoteCMDData(); //ң��������
//  Pulse_acquire();    //�������ɼ�
//  RemoteControl();    //ң�ؿ����ٶ�
  PIT_Flag_Clear(PIT0);       //���жϱ�־λ 
}

void PIT1_IRQHandler(void)
{
  PIT_Flag_Clear(PIT1);       //���жϱ�־λ 
}
