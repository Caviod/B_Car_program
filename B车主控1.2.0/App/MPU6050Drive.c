# include "include.h"

# ifdef MPU6050UseDMP
# include  "inv_mpu_dmp_motion_driver.h"
# include "inv_mpu.h"
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = { 0 };
/* The sensors can be mounted onto the board in any orientation. The mounting
* matrix seen below tells the MPL how to rotate the raw data from thei
* driver(s).
* TODO: The following matrices refer to the configuration on an internal test
* board at Invensense. If needed, please modify the matrices to match the
* chip-to-body matrix for your particular set up.
*/
static signed char gyro_orientation[9] = { -1, 0, 0,
0, -1, 0,
0, 0, 1 };
/* These next two functions converts the orientation matrix (see
* gyro_orientation) to a scalar representation for use by the DMP.
* NOTE: These functions are borrowed from Invensense's MPL.
*/
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
    XYZ  010_001_000 Identity Matrix
    XZY  001_010_000
    YXZ  010_000_001
    YZX  000_010_001
    ZXY  001_000_010
    ZYX  000_001_010
    */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
/* Every time new gyro data is available, this function is called in an
* ISR context. In this example, it sets a flag protecting the FIFO read
* function.
*/
static void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}
unsigned char accel_fsr;
unsigned short gyro_rate, gyro_fsr;

int MPU6050DMP_Init()
{
    u8 res = 0;
    //InitMPU6050();
    struct int_param_s int_param;

    I2C_Init(I2C_0);

    res = mpu_init(&int_param);
    if (res) return -1;
    //
    res = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (res) return 1;
    res = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (res) return 2;
    res = mpu_set_sample_rate(100);
    if (res) return 3;

    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);

    res = dmp_load_motion_driver_firmware();
    if (res) return 4;
    res = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if (res) return 5;
    res = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL);
    if (res) return 6;
    res = dmp_set_fifo_rate(100);
    if (res) return 7;
    res = mpu_set_dmp_state(1);
    if (res) return 9;

    return 0;
}
#endif

# define simiic_write_reg simiic_write_reg(

void InitMPU6050(void)
{
   // unsigned char databuff[8];
#ifndef I2C_SOFTWARE
    IIC_init();
    DELAY_MS(100);
    //I2C_Init(I2C_0);
    //simiic_write_reg MPU6050_DEV_ADDR, PWR_MGMT_1, 0x80);	   //�������״̬
    //DELAY_MS(10);
    simiic_write_reg MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);	   //�������״̬
    DELAY_MS(10);
    simiic_write_reg MPU6050_DEV_ADDR, 0x6C, 0x00);	   //�������״̬
    DELAY_MS(10);
    simiic_write_reg MPU6050_DEV_ADDR, SMPLRT_DIV, 0x07);      //125HZ������ 0x07
        DELAY_MS(10);
    simiic_write_reg MPU6050_DEV_ADDR, CONFIG, 0x04);          //0x04
        DELAY_MS(10);
    simiic_write_reg MPU6050_DEV_ADDR, GYRO_CONFIG, 0x08);     //500deg/s 0x08
        DELAY_MS(10);
    simiic_write_reg MPU6050_DEV_ADDR, ACCEL_CONFIG, 0x00);    //2g 0x00
        DELAY_MS(10);
    simiic_write_reg MPU6050_DEV_ADDR, User_Control, 0x00);
        DELAY_MS(10);
    simiic_write_reg MPU6050_DEV_ADDR, INT_PIN_CFG, 0x02);
        DELAY_MS(10);
#else
    //IIC_Software_PortSet(PTD, 8, PTD, 9, PTD9);
    //databuff[0] = 0x00; IIC_Write_Arr(MPU6050_DEV_ADDR, PWR_MGMT_1, databuff, 1);
    //databuff[0] = 0x00; IIC_Write_Arr(MPU6050_DEV_ADDR, 0x6C, databuff, 1);

    //databuff[0] = 0x07; IIC_Write_Arr(MPU6050_DEV_ADDR, SMPLRT_DIV, databuff, 1);
    //databuff[0] = 0x04; IIC_Write_Arr(MPU6050_DEV_ADDR, CONFIG, databuff, 1);
    //databuff[0] = 0x18; IIC_Write_Arr(MPU6050_DEV_ADDR, GYRO_CONFIG, databuff, 1);
    //databuff[0] = 0x10; IIC_Write_Arr(MPU6050_DEV_ADDR, ACCEL_CONFIG, databuff, 1);
    //databuff[0] = 0x00; IIC_Write_Arr(MPU6050_DEV_ADDR, User_Control, databuff, 1);
    //databuff[0] = 0x02; IIC_Write_Arr(MPU6050_DEV_ADDR, INT_PIN_CFG, databuff, 1);
    IIC_Software_PortSet(PTD, 8, PTD, 9, PTD9);
    Single_Write(MPU6050_DEV_ADDR, PWR_MGMT_1, 0x00);
    Single_Write(MPU6050_DEV_ADDR, 0x6C, 0x00);

    Single_Write(MPU6050_DEV_ADDR, SMPLRT_DIV, 0x07);
    Single_Write(MPU6050_DEV_ADDR, CONFIG, 0x04);
    Single_Write(MPU6050_DEV_ADDR, GYRO_CONFIG, 0x18);
    Single_Write(MPU6050_DEV_ADDR, ACCEL_CONFIG, 0x10);
    Single_Write(MPU6050_DEV_ADDR, User_Control, 0x00);
    Single_Write(MPU6050_DEV_ADDR, INT_PIN_CFG, 0x02);

#endif
    Accel_OriginData.X = 0;
    Accel_OriginData.Y = 0;
    Accel_OriginData.Z = 0;
    Accel_OriginData.Offset_X = 0;
    Accel_OriginData.Offset_Y = 0;
    Accel_OriginData.Offset_Z = 0;
    GYRO_OriginData.X = 0;
    GYRO_OriginData.Y = 0;
    GYRO_OriginData.Z = 0;
    GYRO_OriginData.Offset_X = 0;
    GYRO_OriginData.Offset_Y = 0;
    GYRO_OriginData.Offset_Z = 0;

    Get_OffsetWithMPU6050(&Accel_OriginData, &GYRO_OriginData);
}

//�ڲ�ʹ���û��������
int16 GetOData(uint8 REG_Address)
{
    uint8 L;   uint16 H;

    //uint8 data[2];
    //IIC_Read_Arr(MPU6050_DEV_ADDR, REG_Address, data, 2);
    //H = data[0];
    //L = data[1];
    H = simiic_read_reg(MPU6050_DEV_ADDR, REG_Address, IIC);
    My_Delay_Us(20);
    L = simiic_read_reg(MPU6050_DEV_ADDR, REG_Address + 1, IIC);

    return (H << 8) + L;   //�ϳ�����
}

void Get_IMU2D(struct IMU_2D *imu, struct AccelData * Accel_Data, struct GYROData * GYRO_Data)
{
    imu->AX = Round(GetOData(ACCEL_XOUT_H) - Accel_Data->Offset_X);
    imu->AY = Round(GetOData(ACCEL_YOUT_H) - Accel_Data->Offset_Y);
    imu->GZ = Round(GetOData(GYRO_ZOUT_H) - GYRO_Data->Offset_Z);
}

struct AccelData Accel_OriginData;
void Get_AccData(struct AccelData * Accel_Data)
{
    Accel_Data->X = (GetOData(ACCEL_XOUT_H) - Accel_Data->Offset_X); //* 1.225189904435187e-4;
    Accel_Data->Y = (GetOData(ACCEL_YOUT_H) - Accel_Data->Offset_Y); // * 1.225189904435187e-4;
    Accel_Data->Z = (GetOData(ACCEL_ZOUT_H) - Accel_Data->Offset_Z); //* 1.225189904435187e-4;
}

/// <summary>
/// ���г�N�����ֵ
/// </summary>
void Sort_MAX_N_F(float arr[], int size, int maxnum)
{
	int i = 0, k = 0;
	float temp = 0;
	for (k = 0; k<maxnum; k++)
	{
		//�������У�
		for (i = size - 1; i>k; i--)
		{
			if (arr[i]>arr[i - 1])
			{
				temp = arr[i - 1];
				arr[i - 1] = arr[i];
				arr[i] = temp;
			}
		}
	}
}

/// <summary>
/// N�����޷���ֵ�˲�
/// </summary>
float fiter_Medium_F(float num, float * queue, float * queue_buff, int size)
{
	int i = 0;


	//!+<<<<<<<<<<<<<<<<<<<<�޷�>>>>>>>>>>>>>>>>>>>>+//
//	if (num - queue[0] < -4 && num - queue[0]>4)
//	{
//		num = queue[0];
//	}

	//!+<<<<<<<<<<<<<<<<<<<<���¶���>>>>>>>>>>>>>>>>>>>>+//
	for (i = size - 1; i > 0; i--)
	{
		queue[i] = queue[i - 1];
	}
	queue[0] = num;

	//!+<<<<<<<<<<<<<<<<<<<<����λֵ>>>>>>>>>>>>>>>>>>>>+//
	for (i = 0; i < size; i++)
		queue_buff[i] = queue[i];

	Sort_MAX_N_F(queue_buff, size, (1 + size) / 2);

	return queue_buff[(1 + size) / 2 - 1];
}
float GYROArr[3] = { 0 };
float GYROBuffArr[3] = { 0 };
struct GYROData GYRO_OriginData;
float GYROBuff = 0;
float GYROBuffTemp = 0;
void Get_GyroWithMPU6050(struct GYROData * GYRO_Data)
{
//    GYRO_Data->X = (GetOData(GYRO_XOUT_H) - GYRO_Data->Offset_X)* 0.0609756097560976;
//    GYRO_Data->Y = (GetOData(GYRO_YOUT_H) - GYRO_Data->Offset_Y)* 0.0609756097560976;
//    GYRO_Data->Z = (GetOData(GYRO_ZOUT_H) - GYRO_Data->Offset_Z)* 0.0609756097560976;
    //GYRO_Data->X = (GetOData(GYRO_XOUT_H) - GYRO_Data->Offset_X);
    //GYRO_Data->Y = (GetOData(GYRO_YOUT_H) - GYRO_Data->Offset_Y);
    GYROBuff = (GetOData(GYRO_ZOUT_H) - GYRO_Data->Offset_Z);
    //GYRO_Data->Z = fiter_Medium_F(GYROBuff,GYROArr ,GYROBuffArr, 3);
    GYRO_Data->Z = GYROBuff;
    //GYRO_Data->Y = (GetOData(GYRO_YOUT_H) - GYRO_Data->Offset_Y);
    //GYRO_Data->Z = (GetOData(GYRO_ZOUT_H) - GYRO_Data->Offset_Z);
}

//��ʼ�������Ǻͼ��ٶȼƵ���ƫֵ
void Get_OffsetWithMPU6050(struct AccelData * Accel_Data, struct GYROData * GYRO_Data)
{
    int Count_Collect = 0;
    float temp_X = 0, temp_Y = 0, temp_Z = 0;
    while (Count_Collect<200)
    {
        Count_Collect++;
        DELAY_MS(1);
        Get_AccData(Accel_Data);
        temp_X += Accel_Data->X;
        temp_Y += Accel_Data->Y;
        temp_Z += Accel_Data->Z;
    }
    Accel_Data->Offset_X = temp_X / Count_Collect;
    Accel_Data->Offset_Y = temp_Y / Count_Collect;
    Accel_Data->Offset_Z = temp_Z / Count_Collect;

    Count_Collect = 0;
    temp_X = 0;
    temp_Y = 0;
    temp_Z = 0;
    while (Count_Collect<200)
    {
        Count_Collect++;
        DELAY_MS(1);
        Get_GyroWithMPU6050(GYRO_Data);
        temp_X += GYRO_Data->X;
        temp_Y += GYRO_Data->Y;
        temp_Z += GYRO_Data->Z;
    }
    GYRO_Data->Offset_X = temp_X / Count_Collect;
    GYRO_Data->Offset_Y = temp_Y / Count_Collect;
    GYRO_Data->Offset_Z = temp_Z / Count_Collect;
}
/*******************************************************************************
                              ������ٶȲɼ�
*******************************************************************************/
struct IMU_2D imu_2d_raw;
void Achieve_Gyro()
{
  Get_IMU2D(&imu_2d_raw, &Accel_OriginData, &GYRO_OriginData);
}




