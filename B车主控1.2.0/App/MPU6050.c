#include "MPU6050.h"
#include "control.h"

unsigned char   mpu6050_buffer[14];     //I2C读取后存放数据
int ACC_OFFSET_X = -515;
int ACC_OFFSET_Y = 0;
int ACC_OFFSET_Z = -1;
int GYRO_OFFSET_X = 257;
int GYRO_OFFSET_Y = -515;
int GYRO_OFFSET_Z = 15420;

unsigned char	GYRO_OFFSET_OK = 1;
unsigned char	ACC_OFFSET_OK = 1;

int MPU6050_ACC_LAST_X,MPU6050_ACC_LAST_Y,MPU6050_ACC_LAST_Z;
int MPU6050_GYRO_LAST_X,MPU6050_GYRO_LAST_Y,MPU6050_GYRO_LAST_Z;


/**********************************************************/
//函数名称：void MPU6050_Init
//函数功能：MPU6050初始化
/**********************************************************/
void MPU6050_Init(void)
{
  i2c_init(MPU6050_I2C_Moudle,0,0,4);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_PWR_MGMT_1, 0x00);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_PWR_MGMT_2, 0x00);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_SMPLRT_DIV, 0x00);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_CONFIG, MPU6050_DLPF_BW_256);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_2000);
  i2c_writeaddr(MPU6050_I2C_Moudle, MPU6050_ADDRESS,MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_4); 
}
/**********************************************************/
//函数名称：void MPU6050_Dataanl
//函数功能：MPU6050数据读取并处理  整定零偏
/**********************************************************/
void MPU6050_Dataanl(void)
{
  MPU6050_ACC_LAST_X = GetAccelX() - ACC_OFFSET_X;
  MPU6050_ACC_LAST_Y = GetAccelY() - ACC_OFFSET_Y;
  MPU6050_ACC_LAST_Z = GetAccelZ() - ACC_OFFSET_Z;
  
  MPU6050_GYRO_LAST_X = GetAnguX() - GYRO_OFFSET_X;
  MPU6050_GYRO_LAST_Y = GetAnguY() - GYRO_OFFSET_Y;
  MPU6050_GYRO_LAST_Z = GetAnguZ() - GYRO_OFFSET_Z;
  
  while(!GYRO_OFFSET_OK)
  {
    static long int tempgx=0,tempgy=0,tempgz=0;
    static unsigned char cnt_g=0;

    if(cnt_g==0)
    {
      GYRO_OFFSET_X=0;
      GYRO_OFFSET_Y=0;
      GYRO_OFFSET_Z=0;
      tempgx = 0;
      tempgy = 0;
      tempgz = 0;
      cnt_g = 1;
    }
    tempgx+= MPU6050_GYRO_LAST_X;
    tempgy+= MPU6050_GYRO_LAST_Y;
    tempgz+= MPU6050_GYRO_LAST_Z;
    if(cnt_g==200)
    {
      GYRO_OFFSET_X=tempgx/cnt_g;
      GYRO_OFFSET_Y=tempgy/cnt_g;
      GYRO_OFFSET_Z=tempgz/cnt_g;
      cnt_g = 0;
      GYRO_OFFSET_OK = 1;
      
    }
    cnt_g++;
  }
  while(!ACC_OFFSET_OK)
  {
    static long int tempax=0,tempay=0,tempaz=0;
    static unsigned char cnt_a=0;
    
    if(cnt_a==0)
    {
      ACC_OFFSET_X = 0;
      ACC_OFFSET_Y = 0;
      ACC_OFFSET_Z = 0;
      tempax = 0;
      tempay = 0;
      tempaz = 0;
      cnt_a = 1;
      
    }
    tempax += MPU6050_ACC_LAST_X;//累加
    tempay += MPU6050_ACC_LAST_Y;
    tempaz += MPU6050_ACC_LAST_Z;
    if(cnt_a==200)
    {
      ACC_OFFSET_X = tempax/cnt_a;
      ACC_OFFSET_Y = tempay/cnt_a;
      ACC_OFFSET_Z = tempaz/cnt_a;
      cnt_a = 0;
      ACC_OFFSET_OK = 1;
      
    }
    cnt_a++;
  }
}
/**********************************************************/
//函数功能：读取数据
/**********************************************************/
int16 MPU6050_GetDoubleData(uint8 Addr)
{
  uint16 data=0x0000;
  data=i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, Addr);
  data=(uint16)((data<<8)&0xff00);
  data+=i2c_readaddr(MPU6050_I2C_Moudle,MPU6050_ADDRESS, Addr+1);
  return (int16)data;//合成数据，为有符号整形数
}
/**********************************************************/
//函数功能：获取MPU6050相应轴上的加速度数据
/**********************************************************/
// X/Y/Z-Axis Acceleration
int GetAccelX ()
{
  return MPU6050_GetDoubleData(MPU6050_GYRO_XOUT);
}

int GetAccelY ()
{
  return MPU6050_GetDoubleData(MPU6050_GYRO_YOUT);
}

int GetAccelZ ()
{
  return MPU6050_GetDoubleData(MPU6050_GYRO_ZOUT);
}
/**********************************************************/
//函数功能：获取MPU6050相应轴上的角速度数据
/**********************************************************/
// X/Y/Z-Axis Angular velocity
int GetAnguX ()
{
  return MPU6050_GetDoubleData(MPU6050_ACCEL_XOUT);
}

int GetAnguY ()
{
  return MPU6050_GetDoubleData(MPU6050_ACCEL_YOUT);
}

int GetAnguZ ()
{
  return MPU6050_GetDoubleData(MPU6050_ACCEL_ZOUT);
}