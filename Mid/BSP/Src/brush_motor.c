/*
 * @brief  brush_motor.c
 * @author  fffer
 * @date    2026-05-07
 * @version 1.0
 * @brief   无刷电机实现
 */
#include "brush_motor.h"

#include "tim.h"

/**
 * @brief  motor init function for H bridge
 * @param  None
 * @retval None
 */
void BrushMotor_Init(void)
{
    BrushMotor_Stop();
}

/**
 * @brief  motor enable function for H bridge,activate the IR2104 driver
 * @param  None
 * @retval None
 */
void BrushMotor_Enable(void)
{
  HAL_GPIO_WritePin(PM1_SD_GPIO_Port, PM1_SD_Pin, GPIO_PIN_SET);
}

/**
 * @brief  motor stop function for H bridge,deactivate the IR2104 driver
 * @param  None
 * @retval None
 */
void BrushMotor_Stop(void)
{
  HAL_GPIO_WritePin(PM1_SD_GPIO_Port, PM1_SD_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  set speed of motor
 * @param speed 0-100
 * @retval None
 */
void BrushMotor_SetSpeed(uint8_t speed)
{
    // set speed
    if(speed > 100)
    {
        speed = 100;
    }
    // uint32_t compare = speed * 1000 / 100;
    uint32_t compare = speed * 10;
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, compare);

}
/**
 * @brief  set direction of motor
 * @param direction 0: forward, 1: backward
 * @retval None
 */
void BrushMotor_SetDirection(uint8_t direction)
{
    // set direction
    if(direction == 0)//forward
    {
        HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);//stop CH1N
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//start CH1
    }
    else//backward
    {
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);//start CH1N
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);//stop CH1
    }

}

