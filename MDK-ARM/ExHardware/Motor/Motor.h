#ifndef _MOTOR_H
#define _MOTOR_H
#include "stm32f1xx_hal.h"
#include "stdint.h"

typedef struct
{
    struct
    {
        int Set;  // 速度输出值
        int Real; // 速度真实值
    } Speed;
    struct
    {
        int Set;  // 位置设定值
        int Real; // 位置真实值
    } Position;
    int encoder; //编码器值
}Motor_t;

extern Motor_t M520;
extern void Load(int moto1, int moto2);
extern void Limit(int *motoA, int *motoB);
#endif
