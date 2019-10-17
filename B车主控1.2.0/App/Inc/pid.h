/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H

#include "include.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    float32_t Kp;
    float32_t Ki;
    float32_t Kd;

    float32_t max_out;  //������
    float32_t max_iout; //���������

    float32_t set;
    float32_t fdb;

    float32_t out;
    float32_t Pout;
    float32_t Iout;
    float32_t Dout;
    float32_t Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float32_t error[3]; //����� 0���� 1��һ�� 2���ϴ�

}PidTypeDef;

extern void PID_Init(PidTypeDef *pid, uint8_t mode, const float32_t PID[3], float32_t max_out, float32_t max_iout);
extern float32_t PID_Calc(PidTypeDef *pid, float32_t ref, float32_t set);
extern void PID_clear(PidTypeDef *pid);
#endif
