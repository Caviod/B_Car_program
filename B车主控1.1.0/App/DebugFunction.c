/*!
* @file       DebugFuction.c
* @brief      用于调试用的函数库
* @details
* @author     pig's grief
* @version    v1.0
* @date       2019-9-7
* @to do
*             
*/
# include "include.h"
/*************************************************************/
/*****************遥控部分************/
/*************************************************************/
//定义遥控器初始位置
void RemoteCMDData_init(ReceiveCMDData *Initial_Remote)
{
  Initial_Remote->Right_X=127;
  Initial_Remote->Right_Y=127;
  Initial_Remote->Left_X=127;
  Initial_Remote->Left_Y=127;
}
/// <summary>
///初始化遥控器的串口
///</summary>
void RemoteInit()
{
  uart_init(Remote_Uart_Port,9600);
  set_vector_handler(UART1_RX_TX_VECTORn,ReceiveCMD_Remote);           
  uart_rx_irq_dis(Remote_Uart_Port);
  uart_rx_irq_en(Remote_Uart_Port);
}

int ReceiveIndex = 0;
unsigned char ReceiveBuff[3] = {0};
unsigned char StartReceive = 0;
RemoteCMDMode RunMode;//遥控模式
//#ifdef Remote_UseDigitalReceive

Remote_State Remote_CMD_ReceiveStatus = Sleep;
ReceiveCMDData RemoteData;
long count_error_left = 0;
long count_error_right = 0;
unsigned char Flag_RemoteStopCar = 0;
unsigned char Flag_StartPressOnce = 0;


void GetRemoteCMDData(void)
{
       if(Flag_RemoteStopCar != 0)
	      return;
       if (Remote_CMD_ReceiveStatus==Sleep)
       {
	      Remote_CMD_ReceiveStatus = SendLeftCMD;
	      Remote_CMD_ReceiveStatus = ReceivingLeftCMD;
	      uart_putchar(Remote_Uart_Port, 0xBB);
       }
       if(Remote_CMD_ReceiveStatus == ReceivingLeftCMD)
       {
	      count_error_left++;
	      count_error_right = 0;
       }
       else if(Remote_CMD_ReceiveStatus == ReceivingRightCMD)
       {
	      count_error_left = 0;
	      count_error_right++;
       }
       else
       {
	      count_error_left = 0;
	      count_error_right = 0;
       }
       if(count_error_left>=180||count_error_right>=180)
       {
	      RemoteData.Left_Y  = 127;
	      RemoteData.Left_X  = 127;
	      RemoteData.Right_Y = 127;
	      RemoteData.Right_X = 127;
	      Remote_CMD_ReceiveStatus  = Sleep;
	      count_error_left  = 0;
	      count_error_right = 0;
       }
}



//接受遥控指令程序，应放入对应的串口中断内
void ReceiveCMD_Remote()
{
       //用户需要处理接收数据
       unsigned char buff = 0;
       uart_getchar (Remote_Uart_Port,&buff);
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
	      if (ReceiveIndex < 3)
	      {
		     ReceiveBuff[ReceiveIndex] = buff;
		     ReceiveIndex++;
	      }
	      else
	      {
		     if (buff == 0xFF)
		     {
			    StartReceive = 0;
			    ReceiveIndex = 0;
			    if (ReceiveBuff[0] == 0xBB)//左摇杆
			    {
				   if (Remote_CMD_ReceiveStatus == ReceivingLeftCMD)
				   {
					  RemoteData.Left_Y = ReceiveBuff[1];
					  RemoteData.Left_X = ReceiveBuff[2];
					  Remote_CMD_ReceiveStatus=ReceivedLeftCMD;
					  Remote_CMD_ReceiveStatus=ReceivingRightCMD;
					  uart_putchar (Remote_Uart_Port, 0xCC);
				   }

			    }
			    else if (ReceiveBuff[0] == 0xCC)//右摇杆
			    {
				   if (Remote_CMD_ReceiveStatus == ReceivingRightCMD)
				   {
					  RemoteData.Right_Y = ReceiveBuff[1];
					  RemoteData.Right_X = ReceiveBuff[2];
					  Remote_CMD_ReceiveStatus = ReceivedRightCMD;
					  Remote_CMD_ReceiveStatus = Sleep;
				   }
			    }
			    else
			    {
				   RemoteData.Left_Y = 127;
				   RemoteData.Left_X = 127;
				   RemoteData.Right_Y = 127;
				   RemoteData.Right_X = 127;
				   Remote_CMD_ReceiveStatus = Sleep;
			    }
			    if(ReceiveIndex == 0 && ReceiveBuff[0] == 0xAA)
			    {
				   if(Flag_RemoteStopCar == 0)
					Flag_RemoteStopCar = 1;
				   else if (Flag_RemoteStopCar >= 1)
					Flag_RemoteStopCar++;
				   if(Flag_RemoteStopCar >= 4)
					Flag_RemoteStopCar = 0;
                                     if(Flag_StartPressOnce == 0)
                                        Flag_StartPressOnce = 1;
			    }

		     }
	      }
       }

}

void RemoteControl()
{
  //左摇杆 控制后轮
  float Remote_Left_Vx = 0; 
  float Remote_Left_Vy = 0;
//  
//  if(RemoteData.Left_X < 124 || RemoteData.Left_X > 131)
//    Remote_Left_Vx = (RemoteData.Left_X-127) * (1000/128);  //最大10%占空比
//  else
//    Remote_Left_Vx = 0;
//  if(RemoteData.Left_Y < 124 || RemoteData.Left_Y > 131)
//    Remote_Left_Vy = (RemoteData.Left_Y-127) * (1000/128);  //最大10%占空比
//  else
//    Remote_Left_Vy = 0;
//  
//  if(Remote_Left_Vy >= 0)
//  {
//    FTM_PWM_Duty(FTM0,FTM_CH1,(int)Remote_Left_Vy);
//    FTM_PWM_Duty(FTM0,FTM_CH2,0);
//  }
//  if(Remote_Left_Vy < 0)
//  {
//    FTM_PWM_Duty(FTM0,FTM_CH1,0);
//    FTM_PWM_Duty(FTM0,FTM_CH2,-(int)Remote_Left_Vy);
//  }
  
  if(RemoteData.Left_X < 124 || RemoteData.Left_X > 131)
    Remote_Left_Vx = (RemoteData.Left_X-127);  //最大10%占空比
  else
    Remote_Left_Vx = 0;
  if(RemoteData.Left_Y < 124 || RemoteData.Left_Y > 131)
    Remote_Left_Vy = (RemoteData.Left_Y-127);  //最大10%占空比
  else
    Remote_Left_Vy = 0;
  Speed_Control(Remote_Left_Vy);


  //右摇杆 控制转向
  float Remote_Right_Vx = 0;  //正则向右 负则向左
  float Remote_Right_Vy = 0;
  
  if(RemoteData.Right_X < 124 || RemoteData.Right_X > 131)
    Remote_Right_Vx = (RemoteData.Right_X-127)*(1250/128);
  else
    Remote_Right_Vx=0;
  if(RemoteData.Right_Y < 124 || RemoteData.Right_Y > 131)
    Remote_Right_Vy = (RemoteData.Right_Y-127)*(1250/128);
  else
    Remote_Right_Vy=0;

  FTM_PWM_Duty(FTM1,FTM_CH0,4250+Remote_Right_Vx);
}




