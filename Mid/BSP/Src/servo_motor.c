/**
 * @file    servo_motor.c
 * @brief   舵机驱动源文件
 * @author  Dr. FENG
 * @date    2026-05-06
 * @version V1.0
 * @note    该文件适用于GenBotter Motor-1电机开发板, 且使用FSMC外设连接LCD
 */

 #include "servo_motor.h"

extern TIM_HandleTypeDef htim8;
/**
 * @brief  Initialize the servo motor
 * @param  None
 * @retval None
 */
 void ServoMotor_Init(void)
 {
    //open CH1、CH2、CH3 PWM output
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    //set the initial angle to 90
    ServoMotor_SetAngle(servo_1, 90);
    ServoMotor_SetAngle(servo_2, 90);
    ServoMotor_SetAngle(servo_3, 90);
 }



/**
 * @brief  Get the comparison value of the specified angle
 * @param  angle: the angle of the servo
 * @retval the comparison value of the servo
 */
uint16_t ServoMotor_GetCompareValue(uint8_t angle)
{
    //check the angle range
//    if(angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if(angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    //map the angle to the comparison value
    // return (uint16_t)(500 + angle * 2000 /180);
    return (uint16_t)(500 + angle * 100 /9);

}

/**
 * @brief  Set the angle of the specified servo
 * @param  id: the ID of the servo
 * @param  angle: the angle of the servo
 * @retval None
 */
void ServoMotor_SetAngle(servoID id, uint8_t angle)
{
    if(angle > SERVO_ANGLE_MAX)
    {
        //if angle is out of range, return
        return;
    }

    uint16_t compare_value = ServoMotor_GetCompareValue(angle);

    //set the comparison value of the specified servo
    switch (id)
    {
    case servo_1:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, compare_value);
        break;
    case servo_2:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, compare_value);
        break;
    case servo_3:
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, compare_value);
        break;
    default:
        break;
    }

}
