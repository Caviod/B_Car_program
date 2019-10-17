#include "include.h"

/*******************************************************************************
                              ������FTM��ʼ��
*******************************************************************************/
//���������ʼ��
void Pulse_acquire_init(void)//�ڲ���ʼ������
{
  FTM_QUAD_Init(FTM2);
  gpio_init(PTA17,GPI,0);
}

/*******************************************************************************
                              �������ٶȲɼ�
*******************************************************************************/
int Encoder_Speed = 0;
int Revolve_Flag = 0;
int Revolve_Speed = 0;  //�����ٶ�
void Pulse_acquire(void)    //�жϺ����е���
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