
#include "stm32f1xx_hal.h"
#include "stdint.h"

#define IMU_CALI_TIMESTAMP 2410122331u  //将此数改为另一个值即可触发一次陀螺仪校准 此处可以将其设置为时间戳 注意该值要满足32位宽度

#include "PID.h"
#include "Motor.h"
#include "oled.h"
#include "mpu6050.h"
#include "IIC.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "stdio.h"
#include "Encoder.h"
#include "Inertial.h"

