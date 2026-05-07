/*
 * @brief  brush_motor.h
 * @author  fffer
 * @date    2026-05-07
 * @version 1.0
 * @brief   无刷电机接口
*/


#ifndef __BRUSH_MOTOR_H
#define __BRUSH_MOTOR_H

#include "main.h"

void BrushMotor_Init(void);
void BrushMotor_Enable(void);
void BrushMotor_Stop(void);
void BrushMotor_SetSpeed(uint8_t speed);
void BrushMotor_SetDirection(uint8_t direction);

#endif
