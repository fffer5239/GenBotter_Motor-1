/**
 * @file    servo_motor.h
 * @brief   舵机驱动头文件
 * @author  Dr. FENG
 * @date    2026-05-06
 * @version V1.0
 * @note    该文件适用于GenBotter Motor-1电机开发板, 且使用FSMC外设连接LCD
 */


#ifndef __SERVO_MOTOR_H__
#define __SERVO_MOTOR_H__
 
#include "main.h"

typedef enum {
    servo_1 = 1,
    servo_2 = 2,
    servo_3 = 3,
} servoID;

#define SERVO_ANGLE_MIN 0
#define SERVO_ANGLE_MAX 180
#define SERVO_ANGLE_STEP 10


void ServoMotor_Init(void);

//Get the comparison value of the specified angle
uint16_t ServoMotor_GetCompareValue(uint8_t angle);
void ServoMotor_SetAngle(servoID id, uint8_t angle);



#endif
