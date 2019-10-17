#ifndef _DRIVE_H
#define _DRIVE_H

extern void Motor_Init();
extern void Direction_Control(float Direction_Error); //舵机控制函数 放于定时器中

extern void Speed_PID_Init();
extern void Speed_Control(int TargetSpeed);
#endif