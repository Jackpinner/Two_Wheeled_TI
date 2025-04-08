#ifndef _MOTOR_H
#define _MOTOR_H
#include "stm32f1xx_hal.h"

extern void Load(int moto1,int moto2);
extern void Limit(int *motoA,int *motoB);
#endif
