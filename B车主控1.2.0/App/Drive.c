#include "include.h"
void Motor_Init();

//舵机电机初始化
void Motor_Init()
{
  FTM_PWM_init(FTM1,FTM_CH0,300,4200); //舵机  中值4200 左2950 右5450
  FTM_PWM_init(FTM0,FTM_CH1,10000,0);  //电机正转（接红线）
  FTM_PWM_init(FTM0,FTM_CH2,10000,0);  //电机反转（接绿线）
}
/*******************************************************************************
                                 舵机部分(PD控制)
*******************************************************************************/
void Direction_Control(float Direction_Error); //舵机控制函数 放于定时器中

int Steer_Left_Max = 2950;
int Steer_Right_Max = 5450;

float Direction_Kp = 14;
float Direction_Kd = 7;

float Now_Center_Error = 0;
float Last_Center_Error = 0;

int Steer_PWM_Out;

//舵机打角控制(偏差为正 右打 || 偏差为负 左打)
void Direction_Control(float Direction_Error)
{
  Last_Center_Error = Now_Center_Error;
  Now_Center_Error = Direction_Error;
  Steer_PWM_Out = (int)(4200 + Direction_Kp*Now_Center_Error + Direction_Kd*(Now_Center_Error-Last_Center_Error));
  
  if(Steer_PWM_Out >= Steer_Right_Max)
    Steer_PWM_Out = Steer_Right_Max;
  else if(Steer_PWM_Out <= Steer_Left_Max)
    Steer_PWM_Out = Steer_Left_Max;
  
  //占空比输出 调试可注释
  FTM_PWM_Duty(FTM1,FTM_CH0,Steer_PWM_Out);
}

/*******************************************************************************
                                 电机部分(PID控制)
*******************************************************************************/
void Speed_PID_Init();
void Speed_Control(int TargetSpeed);

PidTypeDef Speed_Loop;
float32_t Max_Output = 2000;
float32_t Max_I_Output = 500;
int Dead_Zone = 140;
int PID_Out = 0;
int PWM_Out = 0;
//速度可调范围在 -127--127
const float32_t Speed_Loop_PID[3] = 
{
  19,
  5,
  3
};
void Speed_PID_Init()
{
  PID_Init(&Speed_Loop, PID_POSITION, Speed_Loop_PID, Max_Output, Max_I_Output);
}

void Speed_Control(int TargetSpeed)
{
  PID_Out = (int)PID_Calc(&Speed_Loop, Revolve_Speed, TargetSpeed);
  if(PID_Out >= 0)
  {
    PWM_Out = PID_Out + Dead_Zone;
    FTM_PWM_Duty(FTM0,FTM_CH1,PWM_Out);
    FTM_PWM_Duty(FTM0,FTM_CH2,0);
  }
  else
  {
    PWM_Out = -PID_Out + Dead_Zone;
    FTM_PWM_Duty(FTM0,FTM_CH1,0);
    FTM_PWM_Duty(FTM0,FTM_CH2,PWM_Out);
  }
}