#include "header.h"

// float Vertical_Kp, Vertical_Kd;
// float Velocity_Kp, Velocity_Ki;
// int Turn_Kp,Turn_Kd;
extern int Encoder_Left,Encoder_Right;
extern int Robot_enabel;
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short	aacx,aacy,aacz;
extern TIM_HandleTypeDef htim2,htim4;
uint8_t stop;
extern uint8_t rx_buffer[2];
#define SPEED_Y 30 //俯仰(前后)最大设定速度
#define SPEED_Z 150//偏航(左右)最大设定速度 
//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
float Med_Angle=0;//平衡时角度值偏移量（机械中值）
//参数
float Vertical_Kp=-180,Vertical_Kd=-2;			//直立环 数量级（Kp：0~1000、Kd：0~10）
float Velocity_Kp=0.6,Velocity_Ki=0.003;		//速度环 数量级（Kp：0~1）
float Turn_Kp=10,Turn_Kd=0.6;			
extern uint8_t Fore,Back,Left,Right;
uint8_t stop;
// 直立PID控制器
// 输入：期望角度、真实角度、角速度
int Vertical(float med, float angle, float gyro_Y)
{
    int temp;
    temp = Vertical_Kp * (angle - med) + Vertical_Kd * gyro_Y;
    return temp;
}

// 速度环PID控制器
// 输入：期望速度、左编码器、右编码器
int Velocity(int target, int encoder_L, int encoder_R)
{
    static int Err_LowOut_last,Encoder_S;
    static float a = 0.7;
    // 偏差值计算
    int Err, Err_LowOut,temp;
    Err = (encoder_L + encoder_R) - target;
    // 低通滤波
    Err_LowOut = (1 - a) * Err + a * Err_LowOut_last;
    Err_LowOut_last = Err_LowOut;
    //积分
    Encoder_S+=Err_LowOut;
    //限幅
    Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
    if(stop==1)
    {
        Encoder_S=0;
        stop=1;
    }
    //速度环计算
    temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
    return temp;
}
int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
	return temp;
}
void Control(void)	//每隔10ms调用一次
{
	int PWM_out;
	//1、读取编码器和陀螺仪的数据
	Encoder_Left=Read_Speed(htim2);
	Encoder_Right=-Read_Speed(htim4);
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	
	//遥控
	if((Fore==0)&&(Back==0))Target_Speed=0;//未接受到前进后退指令-->速度清零，稳在原地
	if(Fore==1)
	{
//		if(distance<50)
//			Target_Speed--;
//		else
//			Target_Speed++;
		Target_Speed++;
		
	}
	if(Back==1){Target_Speed--;}//
	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//限幅
	
	/*左右*/
	if((Left==0)&&(Right==0))Target_turn=0;
	if(Left==1)Target_turn-=30;	//左转
	if(Right==1)Target_turn+=30;	//右转
	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);//限幅( (20*100) * 100   )
	
	/*转向约束*/
	if((Left==0)&&(Right==0))Turn_Kd=0.6;//若无左右转向指令，则开启转向约束
	else if((Left==1)||(Right==1))Turn_Kd=0;//若左右转向指令接收到，则去掉转向约束
	//2、将数据传入PID控制器，计算输出结果，即左右电机转速值
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
	Vertical_out=Vertical(Velocity_out+Med_Angle,roll,gyrox);
	Turn_out=Turn(gyroz,Target_turn);
	PWM_out=Vertical_out;
	MOTO1=PWM_out-Turn_out;
	MOTO2=PWM_out+Turn_out;
	Limit(&MOTO1,&MOTO2);
  Load(MOTO1,MOTO2);
    
}
