#ifndef _MOTOR_H
#define _MOTOR_H
#include "stm32f1xx_hal.h"


typedef struct
{
    float kP;           //比例项系数
    float kI;           //积分项系数
    float kD;           //微分项系数

    float MaxError;     //误差限幅
    float MaxOutput;    //输出限幅
    float I_Max;        //积分限幅

    float DeadBand;     //控制死区
    float IntegralBand; //积分区间
} PID_t;

typedef struct
{
    float Q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度
    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // float atanxz;
    // float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} Inertial_t;

extern Inertial_t Inertial;
extern PID_t Velocity_PID;
extern PID_t Vertical_PID;



#endif
