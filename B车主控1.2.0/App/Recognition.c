#include "include.h"
#include  "Recognition.h"
#define ADC_GET_PL adc_once(ADC1_SE15,ADC_8bit)//1 11
#define ADC_GET_PM adc_once(ADC0_SE10,ADC_8bit)//0 13
#define ADC_GET_PR adc_once(ADC0_SE8,ADC_8bit)//0 8
#define ADC_GET_VL adc_once(ADC1_SE14,ADC_8bit)//1 10
#define ADC_GET_VR adc_once(ADC0_SE9,ADC_8bit)//0 12

int Trace = 0;//����Ԫ�ر�־
/*===================================================================
���ܣ����߿������˲�
===================================================================*/
#define Q_mid 0.5 
#define R_mid 10
int x_last_mid, x_mid_mid, x_now_mid;
float p_last_mid, p_mid_mid, p_now_mid;
float kg_mid;

//�ⲿ����
extern float piancha;
extern float pre_piancha;
extern int flag_buzz;
/*===================================================================
���ܣ��ɼ���ʼ��
���룺��
===================================================================*/
void Main_ADC_INIT()
{
	adc_init(ADC1_SE15);//1 11
	adc_init(ADC0_SE10);//0 13
	adc_init(ADC0_SE9);//0 8
	adc_init(ADC1_SE14);//1 10
	adc_init(ADC0_SE8);//0 12
}

/*===================================================================
���ܣ���Ȩƽ��
���룺����->avg���飬Ȩϵ����->weighted���飬���ݸ���->size
�������ԣ�6.4�ղ�����BUG
===================================================================*/
//Ȩ�ر�
# define PianchaFiter_N 5
# define MAXNUM 1     //�ҳ�MAXNUM��������
uint16 Weighted_List_cbh[3] = { 1, 4, 1 };
uint16 Weight_Piancha[PianchaFiter_N] = { 16, 4, 4, 1, 1 };
uint16 Weight_Piancha_3[PianchaFiter_N] = { 4, 2, 2, 1, 1 };

/// <summary>
/// int�ͼ�Ȩƽ��������->avg���飬Ȩϵ����->weighted���飬���ݸ���->size
/// </summary>
int Weighted_Avg_Int(uint16 avg[], uint16 * weighted, int size)
{
	int index1 = 0;
	int All_Weight = 0;
	long sum = 0;
	for (index1 = 0; index1 < size; index1++)
	{
		All_Weight += weighted[index1];
		sum += (avg[index1] * weighted[index1]);
	}
	return (sum / All_Weight);
}

/// <summary>
/// float�ͼ�Ȩƽ��������->avg���飬Ȩϵ����->weighted���飬���ݸ���->size
/// </summary>
float Weighted_Avg_float(float avg[], uint16 * weighted, int size)
{
	int index1 = 0;
	uint16 All_Weight = 0;
	double sum = 0;
	for (index1 = 0; index1 < size; index1++)
	{
		All_Weight += weighted[index1];
		sum += (avg[index1] * weighted[index1]);
	}
	return (sum / All_Weight);
}


/*===================================================================
���ܣ����г�N�����ֵ�ֱ��Ӧ�����±�0��1��2��������n-1
���룺����->arr���飬���ݸ���->size,���ֵ����->maxnum
�������ԣ�6.4�ղ�����BUG
===================================================================*/
/// <summary>
/// ���г�N�����ֵ
/// </summary>
void Sort_MAX_N(uint16 arr[], int size, int maxnum)
{
	int i = 0, k = 0;
	int temp = 0;
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


/*===================================================================
���ܣ�ˮƽ��й�һ��
���룺arr_fΪ��һ��ǰ�ĵ�вɼ����飬arr_oΪ��һ����ĵ�вɼ�����
��ע�������±��ʾ��0->���У�1->�е�У�2->�ҵ��
�������ԣ�6.4�ղ�����BUG
===================================================================*/
uint16 NORMAL_MIN[5] = { 1, 3, 2, 3, 3 }; //��һ���õĵ�е���Сֵ�±�0��1��2��3��4�ֱ��ʾˮƽ�����С��ҵ��ֵ����ֱ�����ҵ��
uint16 NORMAL_MAX[5] = { 65534, 65534, 65534, 655364, 65534 }; //��һ���õĵ�е����ֵ�±�0��1��2��3��4�ֱ��ʾˮƽ�����С��ҵ��ֵ����ֱ�����ҵ��
/// <summary>
/// ˮƽ��й�һ��
/// </summary>
void Normal_P(uint16 * arr_f, float * arr_o)
{
	int i = 0;

	arr_o[0] = (float)(arr_f[0] - NORMAL_MIN[0]) * 10000.0 / (float)(NORMAL_MAX[0] - NORMAL_MIN[0]);
	arr_o[1] = (float)(arr_f[1] - NORMAL_MIN[1]) * 10000.0 / (float)(NORMAL_MAX[1] - NORMAL_MIN[1]);
	arr_o[2] = (float)(arr_f[2] - NORMAL_MIN[2]) * 10000.0 / (float)(NORMAL_MAX[2] - NORMAL_MIN[2]);

	//!+<<<<<<<<<<<<<<<<<<<<�޷�>>>>>>>>>>>>>>>>>>>>+//
	for (i = 0; i < 3; i++)
	{
		if (arr_o[i] < 1)
			arr_o[i] = 1;
		if (arr_o[i] > 10000)
			arr_o[i] = 10000;
	}
}

/*===================================================================
���ܣ���ֱ��й�һ��
���룺arr_fΪ��һ��ǰ�ĵ�вɼ����飬arr_oΪ��һ����ĵ�вɼ�����
��ע�������±��ʾ��0->���У�1->�ҵ��
�������ԣ�6.4�ղ�����BUG
===================================================================*/
/// <summary>
/// ��ֱ��й�һ��
/// </summary>
void Normal_V(int * arr_f, int * arr_o)
{
	int i = 0;


	arr_o[0] = (arr_f[0] - NORMAL_MIN[3]) * 100 / (NORMAL_MAX[3] - NORMAL_MIN[3]);
	arr_o[1] = (arr_f[1] - NORMAL_MIN[4]) * 100 / (NORMAL_MAX[4] - NORMAL_MIN[4]);


	for (i = 0; i < 3; i++)
	{
		if (arr_o[i] < 1)
			arr_o[i] = 1;
		if (arr_o[i] > 100)
			arr_o[i] = 100;
	}
}

/*===================================================================
���ܣ���Ȩ����ƽ���˲�
���룺num->ʵʱֵ��queue->�˲�����,weighted->Ȩ�ر�,num_q->�˲�����
===================================================================*/
/// <summary>
/// int�ͼ�Ȩ����ƽ���˲�
/// </summary>
int Fiter_int(int num, uint16 * queue, uint16 * weighted, int num_q)
{
	//!+<<<<<<<<<<<<<<<<<<<<���¶���>>>>>>>>>>>>>>>>>>>>+//
	int i = 0;
	for (i = num_q - 1; i > 0; i--)
	{
		queue[i] = queue[i - 1];
	}
	queue[0] = num;

	return Weighted_Avg_Int(queue, weighted, num_q);
}
/*===================================================================
���ܣ���Ȩ����ƽ���˲�
���룺num->ʵʱֵ��queue->�˲�����,weighted->Ȩ�ر�,num_q->�˲�����
===================================================================*/
/// <summary>
/// float�ͼ�Ȩ����ƽ���˲�
/// </summary>
float Fiter_float(float num, float * queue, uint16 * weighted, int num_q)
{
	//!+<<<<<<<<<<<<<<<<<<<<���¶���>>>>>>>>>>>>>>>>>>>>+//
	int i = 0;
	for (i = num_q - 1; i > 0; i--)
	{
		queue[i] = queue[i - 1];
	}
	queue[0] = num;

	return Weighted_Avg_float(queue, weighted, num_q);
}

/*===================================================================
���ܣ������޷��˲�
���룺num->ʵʱֵ��queue->�˲�����,N->������������,A->�޷���ֵ
===================================================================*/
# define ED_N 3
# define ED_A 3
int num_count_PL = 0;//�����˲�����
int num_count_VL = 0;//�����˲�����
int num_count_PM = 0;//�����˲�����
int num_count_VR = 0;//�����˲�����
int num_count_PR = 0;//�����˲�����

int PL_queue_ED[ED_N] = { 0 };
int PM_queue_ED[ED_N] = { 0 };
int PR_queue_ED[ED_N] = { 0 };
int VL_queue_ED[ED_N] = { 0 };
int VR_queue_ED[ED_N] = { 0 };

/*===================================================================
���ܣ�N�����޷���ֵ�˲�
���룺num->ʵʱֵ,queue->�˲����У�queue_buff->�˲����л���
�������ԣ�6.4�ղ�����BUG
===================================================================*/
# define Medium_N 5
uint16 Queue_buff[Medium_N] = { 0 };
uint16 PL_queue_M[Medium_N] = { 0 };
uint16 PM_queue_M[Medium_N] = { 0 };
uint16 PR_queue_M[Medium_N] = { 0 };
uint16 VL_queue_M[Medium_N] = { 0 };
uint16 VR_queue_M[Medium_N] = { 0 };

/// <summary>
/// N�����޷���ֵ�˲�
/// </summary>
uint16 fiter_Medium(uint16 num, uint16 * queue, uint16 * queue_buff, int size)
{
	int i = 0;


	//!+<<<<<<<<<<<<<<<<<<<<�޷�>>>>>>>>>>>>>>>>>>>>+//
	if (num - queue[0] < -4 && num - queue[0]>4)
	{
		num = queue[0];
	}

	//!+<<<<<<<<<<<<<<<<<<<<���¶���>>>>>>>>>>>>>>>>>>>>+//
	for (i = size - 1; i > 0; i--)
	{
		queue[i] = queue[i - 1];
	}
	queue[0] = num;

	//!+<<<<<<<<<<<<<<<<<<<<����λֵ>>>>>>>>>>>>>>>>>>>>+//
	for (i = 0; i < size; i++)
		queue_buff[i] = queue[i];

	Sort_MAX_N(queue_buff, size, (1 + size) / 2);

	return queue_buff[(1 + size) / 2 - 1];
}
/*===================================================================
���ܣ���вɼ�
���룺
�������ԣ�6.4�ղ�����BUG
===================================================================*/
# define N_COLLECT 50//�ɼ�����
# define N_lvbo 5 //��вɼ��˲���С
int Vertical_AD_L[N_COLLECT] = { 0 };//�ɼ���������
int Vertical_AD_R[N_COLLECT] = { 0 };//�ɼ���������
uint16 Plane_AD_L[N_COLLECT] = { 0 };//�ɼ���������
uint16 Plane_AD_M[N_COLLECT] = { 0 };//�ɼ���������
uint16 Plane_AD_R[N_COLLECT] = { 0 };//�ɼ���������
uint16 Plane_AD[3] = { 0 };//�±�0��1��2�ֱ��ʾˮƽ�����С��ҵ��ֵ
int PL_queue[N_lvbo] = { 0 };
int PM_queue[N_lvbo] = { 0 };
int PR_queue[N_lvbo] = { 0 };
int VL_queue[N_lvbo] = { 0 };
int VR_queue[N_lvbo] = { 0 };
int Weight_AD[N_lvbo] = { 1, 1, 1, 1, 2 };
float Plane_AD_N_New[3] = { 0 };//�±�0��1��2�ֱ��ʾˮƽ�����С��ҵ��ֵ
float Plane_AD_N[3] = { 0 };//�±�0��1��2�ֱ��ʾˮƽ�����С��ҵ��ֵ
int Vertical_AD[2] = { 0 };//�±�0��1�ֱ��ʾ��ֱ�����ҵ��ֵ
int Vertical_AD_N[2] = { 0 };//�±�0��1�ֱ��ʾ��ֱ�����ҵ��ֵ
float Vertical_AD_N_New[2] = { 0 };//�±�0��1�ֱ��ʾ��ֱ�����ҵ��ֵ
int Distance_MID[5] = { 0 };//�±�0��1��2��3��4�ֱ��ʾˮƽ�����С��ҵ��ֵ����ֱ�����ҵ��

void AD_Collect()
{
	int i = 0, j = 0;
	float rate_buff = 0;

	//!+<<<<<<<<<<<<<<<<<<<<ADC�ɼ�>>>>>>>>>>>>>>>>>>>>+//
	for (i = 0; i < N_COLLECT; i++)
	{
		Plane_AD_L[i] = ADC_GET_PL;
		Plane_AD_M[i] = ADC_GET_PM;
		Plane_AD_R[i] = ADC_GET_PR;
		//Vertical_AD_L[i] = ADC_GET_VL;
		//Vertical_AD_R[i] = ADC_GET_VR;
		//dwt_delay_us(1);
	}

	//!+<<<<<<<<<<<<<<<<<<<<������η�ֵ>>>>>>>>>>>>>>>>>>>>+//
	//�����һ��Ԫ�ؾ������ֵ��
	Sort_MAX_N(Plane_AD_L, N_COLLECT, MAXNUM);
	Sort_MAX_N(Plane_AD_M, N_COLLECT, MAXNUM);
	Sort_MAX_N(Plane_AD_R, N_COLLECT, MAXNUM);
	//Sort_MAX_N(Vertical_AD_L, N_COLLECT, MAXNUM);
	//Sort_MAX_N(Vertical_AD_R, N_COLLECT, MAXNUM);

	/*
	//!+<<<<<<<<<<<<<<<<<<<<ȡ��ֵ>>>>>>>>>>>>>>>>>>>>+//
	Plane_AD[0] = Weighted_Avg_Int(Plane_AD_L, Weighted_List_Collect, MAXNUM);
	Plane_AD[1] = Weighted_Avg_Int(Plane_AD_M, Weighted_List_Collect, MAXNUM);
	Plane_AD[2] = Weighted_Avg_Int(Plane_AD_R, Weighted_List_Collect, MAXNUM);
	Vertical_AD[0] = Weighted_Avg_Int(Vertical_AD_L, Weighted_List_Collect, MAXNUM);
	Vertical_AD[1] = Weighted_Avg_Int(Vertical_AD_R, Weighted_List_Collect, MAXNUM);
	*/
	//!+<<<<<<<<<<<<<<<<<<<<ȡ�����ֵ>>>>>>>>>>>>>>>>>>>>+//
	Plane_AD[0] = Plane_AD_L[0];
	Plane_AD[1] = Plane_AD_M[0];
	Plane_AD[2] = Plane_AD_R[0];
	//Vertical_AD[0] = Vertical_AD_L[0];
	//Vertical_AD[1] = Vertical_AD_R[0];


	//////!+<<<<<<<<<<<<<<<<<<<<ƽ���˲�����С5>>>>>>>>>>>>>>>>>>>>+//
	//Fiter_int(Plane_AD[0], PL_queue,Weight_AD,N_lvbo);
	//Fiter_int(Plane_AD[1], PM_queue, Weight_AD, N_lvbo);
	//Fiter_int(Plane_AD[2], PR_queue, Weight_AD, N_lvbo);
	//Fiter_int(Vertical_AD[0], VL_queue, Weight_AD, N_lvbo);
	//Fiter_int(Vertical_AD[1], VR_queue, Weight_AD, N_lvbo);
	/*
	//!+<<<<<<<<<<<<<<<<<<<<�����޷�����С2���޷�3>>>>>>>>>>>>>>>>>>>>+//
	Plane_AD[0] = fiter_EDA(Plane_AD[0],PL_queue_ED,num_count_PL,2,3);
	Plane_AD[1] = fiter_EDA(Plane_AD[1], PM_queue_ED, num_count_PM, 2, 3);
	Plane_AD[2] = fiter_EDA(Plane_AD[2], PR_queue_ED, num_count_PR, 2, 3);
	Vertical_AD[0] = fiter_EDA(Vertical_AD[0], VL_queue_ED, num_count_VL, 2, 3);
	Vertical_AD[1] = fiter_EDA(Vertical_AD[1], VR_queue_ED, num_count_VR, 2, 3);
	*/

	//!+<<<<<<<<<<<<<<<<<<<<�崰����ֵ�˲�>>>>>>>>>>>>>>>>>>>>+//
	Plane_AD[0] = fiter_Medium(Plane_AD[0], PL_queue_M, Queue_buff, Medium_N);
	Plane_AD[1] = fiter_Medium(Plane_AD[1], PM_queue_M, Queue_buff, Medium_N);
	Plane_AD[2] = fiter_Medium(Plane_AD[2], PR_queue_M, Queue_buff, Medium_N);
	//Vertical_AD[0] = fiter_Medium(Vertical_AD[0], VL_queue_M, Queue_buff, Medium_N);
	//Vertical_AD[1] = fiter_Medium(Vertical_AD[1], VR_queue_M, Queue_buff, Medium_N);

	//!+<<<<<<<<<<<<<<<<<<<<��һ��>>>>>>>>>>>>>>>>>>>>+//
	Normal_P(Plane_AD, Plane_AD_N);
	//Normal_V(Vertical_AD, Vertical_AD_N);
}

/*===================================================================
���ܣ����㷽��ƫ��
���룺
===================================================================*/
float PianchaFiter[PianchaFiter_N] = { 0 };
float error_lr_N = 0;//���ҵ�е�ƫ���һ����ģ�
float error_Vlr_N = 0;
float average_distance = 0;
float sum_plane = 0;
float I_error = 0;
float R_error = 0;
float piancha_test = 0;
extern float Dir_Diff;
extern int16 GYRO_z;
extern float DIRC_P, DIRC_D, DIRC_I;
int flag_turn_left = 0;
int flag_turn_right = 0;
int flag_const = 0;
extern float yuanhuan_distance;
/// <summary>
/// Trace=4��Բ��ƫ���㷨
/// </summary>
int flag_Trace5 = 0;//�ȶ�5ģʽ��־λ
float rate_e3 = 4;
float rate_e4_dj = 1.3;//1.3
float rate_e4 = 2;
float rate_e6 = 1;
# define Dajia_50 140
# define Dajia_100 280
int NUM_O = 4;//Բ���ܸ���
int count_O = -1;//��Բ����������
int radium_save[12] = { 70, 100, 50, 100, 50, 100, 50, 100, 50, 100, 50, 100 };
float Rate4_S =20;//С���Ŵ���20
float Rate4_M =10;//�л��Ŵ���16
float Rate4_B =4.5;//�󻷷Ŵ���4.5
int distance_test = 0;//����
int flag_count_S = 0;//����־λ
float Trace5_count = 0;
char mode = 0;
void Error_4(int radium, int flag_dajiao)//ƫ������㷨����
{
	float error_buff = 0;
	float Start_buff = 0;
	float rate_buff = 0;
	float K = 0;
	float B = 0;

	if (mode != 5)
	{
		if (radium <= 65)
			rate_e4 = Rate4_S;
		if (radium > 65 && radium <= 85)
			rate_e4 = Rate4_M;
		if (radium > 85)
			rate_e4 = Rate4_B;
	}

	K = (46 - 12) / 50;//0 46
	B = 12 - 50 * K;
	Start_buff = K * radium + B;

	K = (0 - 5) / 50;
	B = 5 - 50 * K;
	rate_buff = K * radium + B;

	if (Trace == 4 || Trace == 5)//����־
	{
		flag_count_S = 1;
	}
	else
	{
		flag_count_S = 0;
	}
	if (flag_turn_right)
	{
		error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);
		error_Vlr_N = -1 * (float)(Vertical_AD_N[0] - Vertical_AD_N[1]);
		error_buff = error_lr_N;//(error_lr_N + rate_buff * error_Vlr_N) / (1 + rate_buff);//error_lr_N;//
		piancha_test = rate_e4 * error_buff;

		if (distance_test >= 0 && distance_test < 0)//��ʼ���ǰ
		{
			if (flag_turn_right)
				error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
			else if (flag_turn_left)
				error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
			sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			piancha_test = error_lr_N * rate_e3;//error_lr_N * 100 / sum_plane;
		}

		if (mode != 5)
		{
			if (piancha_test > 200)
				piancha_test = 0;
		}
	}
	else if (flag_turn_left)
	{
		error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);
		error_Vlr_N = (float)(Vertical_AD_N[1] - Vertical_AD_N[0]);
		error_buff = error_lr_N;//(1 * error_lr_N + rate_buff * error_Vlr_N) / (1 + rate_buff);//error_lr_N;//
		piancha_test = rate_e4 * error_buff;
		if (distance_test >= 0 && distance_test < 0)//��ʼ���ǰ
		{
			if (flag_turn_right)
				error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
			else if (flag_turn_left)
				error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
			sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			piancha_test = error_lr_N * rate_e3;//error_lr_N * 100 / sum_plane;
		}
		if (mode != 5)
		{
			if (piancha_test < -200)
				piancha_test = 0;
		}
	}
}
int flag_CheckT6 = 0;//��ʼ���6ģʽ��־λ
/*===================================================================
���ܣ�4ģʽ�̶����
���룺radium:Բ���뾶
===================================================================*/
extern float DIRC_P;
int DAJIAO_START = 70;
int DAJIAO_END = 140;
int DAJIAO_Clear = 1000;
//С������
# define DAJIA_SMALL 600//18~20:730 8~10:600 
# define DJ_START_S 55
# define DJ_END_S 140
# define DAJIA_BIG 400//18~20:545  8~10:400
# define DJ_START_B 150
# define DJ_END_B 320

float Speed_O = 0;//�뻷�ٶ�

void ConstError_4(int radium)
{
	int dajiao = 0;
	float K = 0;
	float B = 0;
	//���Ը�ֵ����
	//��Ǹ�ֵ
	K = (DAJIA_BIG - DAJIA_SMALL) / 50;
	B = DAJIA_SMALL - K * 50;
	dajiao = K*radium+B;
	//�ٶȴ�Ǹ�ֵ
	K = (19 - 9) / 9;
	B = 9 - K * 9;
	dajiao = K*Speed_O + B;

	//��ʼֵ��ֵ
	K = (DJ_START_B - DJ_START_S) / 50;
	B = DJ_START_S - K * 50;
	DAJIAO_START = K*radium + B;
	//����ֵ��ֵ
	K = (DJ_END_B - DJ_END_S) / 50;
	B = DJ_END_S - K * 50;
	DAJIAO_END = K*radium + B;



	if (flag_count_S == 1)
	{

		//!+<<<<<<<<<<<<<<<<<<<<�̶���ǿ��Ʋ���>>>>>>>>>>>>>>>>>>>>+//
		if (distance_test >= 0 && distance_test < DAJIAO_START)//��ʼ���ǰ
		{
			if (flag_turn_right)
				error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
			else if (flag_turn_left)
				error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
			sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			piancha = error_lr_N * rate_e3;//error_lr_N * 100 / sum_plane;
		}
		if (distance_test >= DAJIAO_START && distance_test <= DAJIAO_END)//��ǲ���
		{
			if (flag_turn_left)
				piancha = dajiao / DIRC_P;
			else if (flag_turn_right)
				piancha = -dajiao / DIRC_P;
		}
		if (distance_test > DAJIAO_END && distance_test < DAJIAO_Clear)//��ǽ���
		{

		}
		if (distance_test >= DAJIAO_Clear || flag_count_S == 0)//�������
		{
			distance_test = 0;
		}
	}
}
float temp_piancha = 0;
void Error_Calculation(void)
{
	//�򵥵�ϵ��
	//if (radium_save[count_O] == 100)
	//{
	//	rate_e3 = 2;
	//	rate_e4 = 6;
	//	rate_e6 = 1;
	//}
	//if (radium_save[count_O] == 50)
	//{
	//	rate_e3 = 6;
	//	rate_e4 = 9;
	//	rate_e6 = 1;
	//}
	//!+<<<<<<<<<<<<<<<<<<<<��ģʽƫ�ʽ>>>>>>>>>>>>>>>>>>>>+//
	switch (Trace)
	{
		//!+<<<<<<<<<<<<<<<<<<<<Բ������ƫ�����>>>>>>>>>>>>>>>>>>>>+//
	case 3://Բ����ʶ
	{
                  if(mode != 5)
                  {
					  if (flag_turn_right)
						  error_lr_N = (float)(Plane_AD[0] - Plane_AD[1] -1);//L-M
					  else if (flag_turn_left)
						  error_lr_N = (float)(Plane_AD[1] - Plane_AD[2] +1);//M-R
                  }
                  else
                  {
  			   if (flag_turn_right)
                                                     error_lr_N = (float)(Plane_AD[0] - Plane_AD[1] );//L-M
			   else if (flag_turn_left)
                                                     error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R                  
                  }
			   //sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			   piancha = error_lr_N * rate_e3;//error_lr_N * 100 / sum_plane;
			   break;
	}
	case 4://��Բ��״̬
	{
			   //if (radium_save[count_O]>75)//���󲻴��
			   // Error_4(radium_save[count_O], 0);
			   //else
			   /*Error_4(radium_save[count_O], 0);
			   piancha = piancha_test;*/
		if (flag_turn_right)
		{
			error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);
		}
		else if (flag_turn_left)
		{
			error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]-2);
		}
                
		//error_Vlr_N = (float)(Vertical_AD[0] - Vertical_AD[1]);
                
		piancha = error_lr_N * rate_e4;// +0.1* error_Vlr_N;
	
			   break;
	}
	//case 5://Բ����״̬
	//{
		
			   /*
			   error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
			   sum_plane = (float)(Weighted_Avg_Int(Plane_AD_N, Weighted_List_cbh, 3));
			   piancha = error_lr_N * 100 / sum_plane;
			   */


		//if (flag_turn_right)
			//error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);
		//else if (flag_turn_left)
			//error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);
		//piancha = error_lr_N * rate_e4;
					//if(Trace5_count >= 150)
					//	flag_CheckT6 = 1;
					//else
					//	flag_CheckT6 = 0;

     //             if(Trace5_count >= 150)
     //             {
			  // error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
			  // sum_plane = (float)(Weighted_Avg_Int(Plane_AD_N, Weighted_List_cbh, 3));
			  // piancha = error_lr_N * 100 / sum_plane;
     //                                              piancha = rate_e6*piancha;
     //             }
     //             else
     //             {
     //                                              if(mode!=5){
			  // Error_4(radium_save[count_O], 0);
			  // piancha = piancha_test;}
     //             }
	//		   break;
	//}
	case 6://��Բ����־
	{
		if (flag_turn_right)
		{
			error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[1]);//L-R
		}
		else if (flag_turn_left)
		{
			error_lr_N = (float)(Plane_AD_N[1] - Plane_AD_N[2]);//L-R
		}
			   //error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
			   //sum_plane = (float)(Weighted_Avg_Int(Plane_AD_N, Weighted_List_cbh, 3));
			   //piancha = error_lr_N * 100 / sum_plane;
			   piancha = rate_e6 * error_lr_N;
			   break;
	}
	//case 8://��Բ��״̬
	//{
			   //if (flag_turn_left)
			   //	error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
			   //else if (flag_turn_right)
			   //	error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
			   //sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
			   //piancha = error_lr_N * 9;//error_lr_N * 100 / sum_plane;
			   //break;

	//		   error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
	//		   sum_plane = (float)(Weighted_Avg_Int(Plane_AD_N, Weighted_List_cbh, 3));
	//		   piancha = error_lr_N * 100 / sum_plane;
	//		   piancha = rate_e6*piancha;
	//		   break;
	//}
		//case 8:
		//{
		//		if (flag_turn_left)
		//		  	error_lr_N = (float)(Plane_AD[1] - Plane_AD[2]);//M-R
		//		else if (flag_turn_right)
		//		  	error_lr_N = (float)(Plane_AD[0] - Plane_AD[1]);//L-M
		//		sum_plane = (float)(Weighted_Avg_Int(Plane_AD, Weighted_List_cbh, 3));
		//		piancha = error_lr_N * 9;//error_lr_N * 100 / sum_plane;
		//		break;
		//}
		//!+<<<<<<<<<<<<<<<<<<<<��ͨ����ƫ�����>>>>>>>>>>>>>>>>>>>>+//
	default:
	{
			   error_lr_N = (float)(Plane_AD_N[0] - Plane_AD_N[2]);//L-R
			   sum_plane = (float)(Weighted_Avg_float(Plane_AD_N, Weighted_List_cbh, 3));
			   if (sum_plane == 0)
				   sum_plane = 0.1;
			   piancha = error_lr_N * 100 / sum_plane / 2;
			   piancha = Fiter_float(piancha, PianchaFiter, Weight_Piancha, PianchaFiter_N);
			   break;
	}
}
	//!+<<<<<<<<<<<<<<<<<<<<Բ���̶���ǲ���>>>>>>>>>>>>>>>>>>>>+//
	if (flag_const == 1)
	{
		ConstError_4(radium_save[count_O]);//550
	}
                //!+<<<<<<<<<<<<<<<<<<<<ƫ���Ȩ����ƽ���˲�>>>>>>>>>>>>>>>>>>>>+//
        piancha = Fiter_float(piancha, PianchaFiter, Weight_Piancha, PianchaFiter_N);

	//!+<<<<<<<<<<<<<<<<<<<<���ߴ�����>>>>>>>>>>>>>>>>>>>>+//
        temp_piancha = piancha;
        if(Plane_AD[0] <= 0 && Plane_AD[1] <= 0 && Plane_AD[2] <= 0);
        else           
          Lose_Pro();
        
	////!+<<<<<<<<<<<<<<<<<<<<Ϲ�����˲�>>>>>>>>>>>>>>>>>>>>+//
	//if (Trace == 3 || Trace == 6 || Trace == 7 || Trace == 8 || Trace == 5)
	//	piancha = Kalman_Filter_mid(piancha);
	//if (Trace == 3 || Trace == 6 || Trace == 7 || Trace == 8 || Trace == 5)
	//	piancha = Fiter_float(piancha, PianchaFiter, Weight_Piancha_3, PianchaFiter_N);
	//else

	//GYRO_z = 0.07*Get_Gyro(5, 'Z') - 15;
}

/*===================================================================
���ܣ���������������һ��(ʹʵ������ķŴ����ڼ���ʱ���ûص���ʱ���״̬)
���룺
===================================================================*/
//������ֵ��PL->240 PR->240 VL->240 VR->240 PM���������������ֵһ���趨
//�����趨ֵ���ǹ�һ�����ֵ
# define Plane_M_Set 41//�е����һ������µ���ͨ���ֵ�����������ϣ�
# define Sum_Inductance_Set 90//��������������еĺ͵��趨ֵ
# define PL_set 23//���������������ϵ��趨ֵ
# define PM_set 41//�е�������������ϵ��趨ֵ
# define PR_set 23//�ҵ�������������ϵ��趨ֵ

float PL_fact = 0;//���������������ϵ�ʵ��ֵ
float PM_fact = 0;//�е�������������ϵ�ʵ��ֵ
float PR_fact = 0;//�ҵ�������������ϵ�ʵ��ֵ
float Sum_Inductance_fact = 0;//��������������еĺ͵�ʵ��ֵ
int AD_Feature[5] = { 0 };//AD��������
/// <summary>
/// ��Ŀǰ��û��ʽд�ú�����
/// </summary>
void Feature_Normal(void)
{
	AD_Feature[0] = Plane_AD_N[0];
	AD_Feature[1] = Plane_AD_N[1];
	AD_Feature[2] = Plane_AD_N[2];
	AD_Feature[3] = Vertical_AD_N[0];
	AD_Feature[4] = Vertical_AD_N[1];
}
/*===================================================================
���ܣ�0\1\12\2\3�ķ��������,��Ҫ��Ϊ�����ֳ�3ģʽ
���������ģʽ3��3��ģʽ2��2��ģʽ0��0
===================================================================*/
int CTree_3(void)
{
	int x1 = AD_Feature[0] + AD_Feature[2];//����ˮƽ��к�
	int x2 = abs(AD_Feature[3] - AD_Feature[4]);//������ֱ��в�
	int x3 = AD_Feature[3] + AD_Feature[4];

	if (x1 >= 90)//ˮƽ��к�����Ҫ���ڵ���87 75
	{
		if (x1 > 96)//ʮ�ִ�ˮƽ������ֵ 85
			return 3;
		//else//2 3 12ģʽ����һ��Ĳ���
		//{
		//	//x2<=50����ʮ�� x2 >= 15�����µ�
		//	if (x2<=50 && x2 >= 15 && x3 > 82)//��ֱ��кͺܴ�һ����Բ��
		//		return 3;
		else
			return 0;
		//}
	}
	else//0 2ģʽ����һ��Ĳ���
	{
		return 0;
	}


}
/*===================================================================
���ܣ�������־���
���룺
===================================================================*/
int r_flag = 0;//�����־,��ţ�0
int x_flag = 0;//ʮ�ֱ�־,��ţ�2
int ss_flag = 0;//СS��־,��ţ�11
int p_flag = 0;//�µ���־,��ţ�12
int o_flag = 0;//Բ����־,��ţ�3
int o_clear_flag = 0;//Բ�������־
int o_flag_enter = 0;//��Բ����־,��ţ�4
int o_flag_in = 0;//Բ���ڱ�־,��ţ�5
int o_flag_out = 0;//��Բ����־,��ţ�6
int o_flag_outing = 0;//��Բ��ʱ��־,��ţ�7
int o_flag_out_end = 0;//��Բ��������־,��ţ�8
int sum_inductance = 0;//5��еĺ�
int sum_inductance_F = 0;
/// <summary>
/// ������־���
/// </summary>
void CheckTrace(int flag_index)
{
	int i = 0;
	int num_buff = 0;
	Feature_Normal();

	sum_inductance_F = AD_Feature[0] + AD_Feature[2];//����ˮƽ��еĺ�;

	switch (flag_index)
	{
	case 1://ֱ����־,������+SVM
	{
			   num_buff = CTree_3();
			   if (num_buff == 3)
			   {
				   o_flag++;
			   }
			   else if (num_buff == 0)
			   {
				   r_flag++;
			   }
			   break;
	}
	case 4://��Բ����־
	{
			   if (flag_turn_left&&AD_Feature[3]<=4&&AD_Feature[1]>60) //AD_Feature[3]<3|| AD_Feature[4]<3) //Plane_AD[1]>157 AD_Feature[1]>=48) //abs(AD_Feature[3] - AD_Feature[4]) <= 12)//Ctree_4() == 4)
			   {
				   o_flag_enter++;
			   }
			   if (flag_turn_right&&AD_Feature[4]<=4&& AD_Feature[1] > 60) //AD_Feature[3]<3|| AD_Feature[4]<3) //Plane_AD[1]>157 AD_Feature[1]>=48) //abs(AD_Feature[3] - AD_Feature[4]) <= 12)//Ctree_4() == 4)
			   {
				   o_flag_enter++;
			   }
			   /*num_buff = CTree_3();
			   if (num_buff == 0)
			   {
				   o_clear_flag++;
			   }*/
			   break;
	}
	case 5://Բ���ڱ�־
	{
		if(AD_Feature[1] <=30&&sum_inductance_F < 79)//sum_inductance_F < 60 && (AD_Feature[3] + AD_Feature[4]) >= 29)//1.59
			o_flag_in++;
		break;
	}
	case 6:	//Բ����־
	{
		//if (flag_turn_left && AD_Feature[2] + AD_Feature[1] >110)// && (AD_Feature[3] + AD_Feature[4]) >= 73)//sum_inductance_F >= 60)
		if (AD_Feature[1] >= 51)
		{
			o_flag_out++;
		}
		//if (flag_turn_right && AD_Feature[0] + AD_Feature[1] > 110)// && (AD_Feature[3] + AD_Feature[4]) >= 73)//sum_inductance_F >= 60)
		//{
		//	o_flag_out++;
		//}
		break;
	}
	case 7:
	{
		if ((AD_Feature[0] + AD_Feature[1] + AD_Feature[2]) > 89)
		{
			o_flag_outing++;
		}
		break;
	}
	case 8:
	{
		if (AD_Feature[0] + AD_Feature[2] < 54)
		{
			o_flag_out_end++;
		}
		break;
	}
	default:
	{
		r_flag++;
		break;
	}
	}
}
/*===================================================================
���ܣ�������־��0
���룺
===================================================================*/
/// <summary>
/// ������־��0
/// </summary>
void Flag_Clear()
{
	r_flag = 0;//�����־,��ţ�2
	o_flag = 0;//Բ����־,��ţ�3
	o_flag_enter = 0;//��Բ����־,��ţ�4
	o_flag_in = 0;//Բ���ڱ�־,��ţ�5
	o_flag_out = 0;//��Բ����־,��ţ�6
	o_flag_out_end = 0;
	o_flag_outing = 0;
	x_flag = 0;
	ss_flag = 0;
	p_flag = 0;
	o_clear_flag = 0;
}




/*===================================================================
���ܣ�ģʽʶ��
���룺
===================================================================*/
extern float  nAngleControlOut, nSpeedControlOut, nDirControlOut;
extern int flag_stop;
extern float nSpeed;
//Trace��
//0->��� 1->ֱ�� 2-> ʮ�� 11-> СS 3->ǰ����Բ�� 4->��Բ�� 5->Բ���� 6->��Բ��״̬ 7->�ѳ�Բ��
void ModeRecognition(void)
{
	if (o_flag == 60)
	{
		Trace = 3;

		//!+<<<<<<<<<<<<<<<<<<<<�������ת>>>>>>>>>>>>>>>>>>>>+//
		if (Vertical_AD[0] > Vertical_AD[1])
		{
			flag_turn_right = 0;
			flag_turn_left = 1;
		}
		else if (Vertical_AD[0] < Vertical_AD[1])
		{
			flag_turn_left = 0;
			flag_turn_right = 1;
		}
		Flag_Clear();
	}
	if (Trace == 3 && o_clear_flag == 12)//Բ����־���
	{
		Trace = 0;
		Flag_Clear();
	}
	if (o_flag_enter == 5)//��Բ���ж�
	{
		//count_O++;//����һ��Բ��
		//if (count_O >= NUM_O)
		//	count_O = 0;
		/*Speed_O = nSpeed;*/
		Trace = 4;
		yuanhuan_distance = 0;
		Flag_Clear();
	}
	if (o_flag_in == 30)//Բ�����ж�
	{
		Trace = 5;
		Flag_Clear();
	}
	if (x_flag == 8)//ʮ���ж�
	{
		Trace = 2;
		Flag_Clear();
	}
	//else if (p_flag == 7)
	//{
	//	Trace = 12;
	//	Flag_Clear();
	//}
	//else if (ss_flag == 7)
	//{
	//	Trace = 11;
	//	Flag_Clear();
	//}
	if (o_flag_out == 8)//��Բ����־�ж� 6
	{
		Trace = 6;
		flag_CheckT6 = 0;
		Flag_Clear();
	}
	if (o_flag_outing == 6)
	{
		Trace = 7;
		Flag_Clear();
	}
	if (o_flag_out_end == 6)//��Բ�������ж�
	{
		Trace = 8;
		yuanhuan_distance = 0;
		flag_CheckT6 = 0;
		Flag_Clear();
	}
	if (r_flag == 10)//����ж�
	{
		Trace = 0;
		Flag_Clear();
	}

	//!+<<<<<<<<<<<<<<<<<<<<Բ��ʶ�𲿷�>>>>>>>>>>>>>>>>>>>>+//
	if (Trace == 3)
	{
		CheckTrace(4);
		//CheckTrace(1);
	}
	if (Trace == 4)
	{
		CheckTrace(5);
	}
	if (Trace == 5)
	{
		//if (flag_CheckT6 == 1)
		//{
			CheckTrace(6);
		//}
	}
	if (Trace == 6)
	{
		flag_Trace5 = 0;
		CheckTrace(8);
	}
	if (Trace == 7)
	{
		CheckTrace(8);
	}
	if (Trace == 8)
	{
		flag_turn_left = 0;
		flag_turn_right = 0;
		CheckTrace(0);
	}
	//!+<<<<<<<<<<<<<<<<<<<<����ʶ�𲿷�>>>>>>>>>>>>>>>>>>>>+//
	if (Trace < 3 || Trace >8)
	{
		CheckTrace(1);
	}

	////!+<<<<<<<<<<<<<<<<<<<<��������>>>>>>>>>>>>>>>>>>>>+//
	//if ((Plane_AD[0] <= 20) && (Plane_AD[1] <= 20) && (Plane_AD[2] <= 20))
	//	flag_stop = 1;
	//else
	//	flag_stop = 0;
}

/*===================================================================
���ܣ����ߴ������
���룺
===================================================================*/
#define PM_MIN 27000//����ʱ�е�����ֵ

float PM__lose_set = 60;
float rate_pc = 0.3;
int flag_lose = 0;//���߱�־λ
int flag_lose_left = 0;//���߱�־
int flag_lose_right = 0;//�Ҷ��߱�־
int flag_lose_whole = 0;
int last_flag = 0;
float piancha_lose = 0;//����ƫ��
float error_compensate = 0;
int flag_stopcar = 0;

typedef enum
{
  left_lose,
  right_lose,
  whole_lose,
  no_lose
}Lose_Status;
Lose_Status ThisLoseStatus = no_lose;

long stop_count = 0;
long restart_count = 0;
void Lose_Pro()
{
	if (Plane_AD[1] < PM_MIN)
        {
          if(ThisLoseStatus == no_lose)
          {
                if (Plane_AD_N[0] > Plane_AD_N[2])//����Ҷ���
		{
                  ThisLoseStatus = right_lose;	
		}
		if (Plane_AD_N[0] < Plane_AD_N[2])//����Ҷ���
		{
                  ThisLoseStatus = left_lose;
		}
          }
        }
        else
        {
          ThisLoseStatus = no_lose;
        }


        if (ThisLoseStatus == right_lose)
        {
                //piancha = piancha + error_compensate * rate_pc;
                piancha = -1000;
        }
        if (ThisLoseStatus == left_lose)
        {
                //piancha = piancha - error_compensate * rate_pc;
                piancha = 1000;
        }

//	if (AD_Feature[0] < 5 && AD_Feature[2] < 5 && AD_Feature[3] < 5)
//	{
//		stop_count++;
//	}
//	else
//	{
//		restart_count++;
//		flag_stopcar = 0;
//	}
//	if (stop_count >= 2)
//	{
//		flag_stopcar = 1;
//		stop_count = 0;
//		restart_count = 0;
//	}
//	if (restart_count >= 1000)
//	{
//		flag_stopcar = 0;
//		stop_count = 0;
//		restart_count = 0;
//	}
	last_flag = flag_lose;

}


