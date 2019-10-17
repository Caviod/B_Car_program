# ifndef _DEBUGFUNCTION_H_
# define _DEBUGFUNCTION_H_

# define Remote_Uart_Port UART1

typedef enum
{
  Left_Return0,
  Left_Left,
  Left_Right,
  Left_Up,
  Left_Down,
  Right_Return0,
  Right_Left,
  Right_Right,
  Right_Up,
  Right_Down,
  Start
}RemoteCMDMode;

/// <summary>
///接受遥控指令程序，应放入对应的串口中断内
///</summary>
void ReceiveCMD_Remote();
/// <summary>
///初始化遥控器的串口
///</summary>
void RemoteInit();


#ifndef Remote_UseDigitalReceive
typedef enum
{
    Sleep,
    SendLeftCMD,
    ReceivingLeftCMD,
    ReceivedLeftCMD,
    SendRightCMD,
    ReceivingRightCMD,
    ReceivedRightCMD
}Remote_State;


typedef struct
{
    uint8 Left_X;
    uint8 Left_Y;
    uint8 Right_X;
    uint8 Right_Y;
}ReceiveCMDData;
void GetRemoteCMDData(void);
void Chassist_init(ReceiveCMDData *Initial_Remote);

#endif
void RemoteControl();
extern ReceiveCMDData RemoteData;
extern void RemoteControl();
extern void Chassist_init(ReceiveCMDData *Initial_Remote);
#endif