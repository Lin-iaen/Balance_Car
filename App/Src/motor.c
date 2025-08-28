#include "motor.h"
#include "tim.h"
#include "encoder.h"
#include "mpu6050.h"
#include "inv_mpu.h"

#define PWM_MAX 6500
#define PWM_MIN -6500
#define L_Dead_Band 260
#define R_Dead_Band 330
#define INTEGRAL_DEADBAND 100

#define SPEED_Y 20 //����(ǰ��)����趨�ٶ�
#define SPEED_Z 150//ƫ��(����)����趨�ٶ� 

Motor_Typedef Lmotor,Rmotor;

extern float pitch,roll,yaw;
extern short gyrox,gyroy,gyroz;
extern short gyrox_offset,gyroy_offset,gyroz_offset;
extern uint8_t Fore,Back,Left,Right;
float Vertical_Kp=-430*0.6,Vertical_Kd=-0.6*0.6;
float SpeedKp = 0.5,SpeedKi = 0.5/200;//0.4

float Turn_Kp = 10.0;
float Turn_Kd = 0.5;

// --- ң��ָ����� ---
// ��Щ����Ӧ�������������жϻ�����ָ���������������
int Target_Speed = 0;
int Target_Turn = 0;

void Car_Init(void)
{
	Lmotor.target_speed = 0;
	//pid_controller_create(&Lmotor.Vertical,80,0,10,0,6500);
	//pid_controller_create(&Lmotor.SpeedLoop,SpeedKp,SpeedKi,0,0,1000);
	
	
	Rmotor.target_speed = 0;
	//pid_controller_create(&Rmotor.Vertical,80,0,10,0,6500);
	//pid_controller_create(&Rmotor.SpeedLoop,SpeedKp,SpeedKi,0,0,1000);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}

int Vertical(float Med,float Angle,float gyro_Y)
{
	int temp;
	temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
	return temp;
}

int Velocity(int Target,int encoder_L,int encoder_R)
{
	static int Err_LowOut_last,Encoder_S;
	static float a=0.70; // a���˲�ϵ����0.7��һ������ֵ
	int Err,Err_LowOut,temp;

	Err=(encoder_L+encoder_R)-Target;
	Err_LowOut=(1-a)*Err+a*Err_LowOut_last; // һ�׵�ͨ�˲�
	Err_LowOut_last=Err_LowOut;

	Encoder_S+=Err_LowOut;
	if(Encoder_S > 8000) Encoder_S = 8000; // �����޷�
	if(Encoder_S < -8000) Encoder_S = -8000;
	
	
	temp = SpeedKp * Err_LowOut + SpeedKi * Encoder_S;
	return temp;
}

int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	// ����һ��PDת������������Ƶ���ת���ٶ�
	temp = Turn_Kp * Target_turn + Turn_Kd * gyroz; // �������PDת��
	return temp;
}
void Car_ControlLoop(void)
{
	float Med_angle = 0;
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	
	gyrox -= gyrox_offset;
    gyroy -= gyroy_offset;
    gyroz -= gyroz_offset;
		
	Lmotor.current_speed = -Read_Speed(&htim2);
	Rmotor.current_speed =  Read_Speed(&htim4);
	//ң��������
	if((Fore==0)&&(Back==0))Target_Speed=0;//δ���ܵ�ǰ������ָ��-->�ٶ����㣬����ԭ��
	if(Fore==1)Target_Speed = -SPEED_Y;
	if(Back==1)Target_Speed = SPEED_Y;//
	//Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//�޷�
	
	/*����*/
	if((Left==0)&&(Right==0))Target_Turn=0;
	if(Left==1)Target_Turn+=30;	//��ת
	if(Right==1)Target_Turn-=30;	//��ת
	Target_Turn=Target_Turn>SPEED_Z?SPEED_Z:(Target_Turn<-SPEED_Z?(-SPEED_Z):Target_Turn);//�޷�( (20*100) * 100   )
	
	/*ת��Լ��*/
	if((Left==0)&&(Right==0))Turn_Kd=0.6;//��������ת��ָ�����ת��Լ��
	else if((Left==1)||(Right==1))Turn_Kd=0;//������ת��ָ����յ�����ȥ��ת��Լ��
	
	//----�ٶȻ�(�⻷)----//
	float speed_output = Velocity(Target_Speed,Lmotor.current_speed,Rmotor.current_speed);
	//speed_output = 0;
	//----ת��(����)----//
	int turn_out = Turn(gyroz,Target_Turn);
	//turn_out = 0;

	//----ֱ����(�ڻ�)----//
	//Ŀ��Ƕȱ��ٶȻ����������
	int final_output = Vertical(speed_output + Med_angle,roll,gyrox);

	int MOTO1 = final_output - turn_out;
	int MOTO2 = final_output + turn_out;
	pwmLimit(&MOTO1,&MOTO2);
	Load(MOTO1,MOTO2);
	STOP_Car(&Med_angle,roll);
}

int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}

void Load(int moto1,int moto2)			//-7200~7200
{
	if(moto1<0)
	{
		moto1 -= L_Dead_Band;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	}
	else if(moto1>0)
	{
		moto1 += L_Dead_Band;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(moto1));
	if(moto2<0)
	{
		moto2 -= R_Dead_Band;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	else if(moto2>0)
	{
		moto2 += R_Dead_Band;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(moto2));
}

void pwmLimit(int * moto1,int * moto2)
{
	if(*moto1 > PWM_MAX)*moto1 = PWM_MAX;
	if(*moto1 < PWM_MIN)*moto1 = PWM_MIN;
	if(*moto2 > PWM_MAX)*moto2 = PWM_MAX;
	if(*moto2 < PWM_MIN)*moto2 = PWM_MIN;
}

void STOP_Car(float * target_angle,float roll)
{
	
	if(abs((int)(*target_angle-roll))>60)
	{
		Load(0,0);
		//stop=1;
	}
}








