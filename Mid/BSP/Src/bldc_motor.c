#include "bldc_motor.h"

#include "tim.h"
#include "gpio.h"

/* 定义电机控制结构体 */
_bldc_obj g_bldc_motor1 = {STOP,0,0,CCW,0,0,0,0,0,0};   /* 电机结构体初始值 */

/**
 * @brief       BLDC控制函数
 * @param       dir :电机方向, Duty:PWM占空比
 * @retval      无
 */
void bldc_ctrl(uint8_t motor_id,int32_t dir,float duty)
{
    if(motor_id == MOTOR_1)
    {
        g_bldc_motor1.dir = dir;            /* 方向 */
        g_bldc_motor1.pwm_duty = duty;      /* 占空比 */
    }
}

/**
 * @brief       获取霍尔传感器引脚状态
 * @param       motor_id ： 电机接口号
 * @retval      霍尔传感器引脚状态
 */
uint32_t hallsensor_get_state(uint8_t motor_id)
{
    __IO static uint32_t state ;
    state  = 0;
    if(motor_id == MOTOR_1)
    {
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH1_GPIO,HALL1_TIM_CH1_PIN) != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x01U;
        }
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH2_GPIO,HALL1_TIM_CH2_PIN) != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x02U;
        }
        if(HAL_GPIO_ReadPin(HALL1_TIM_CH3_GPIO,HALL1_TIM_CH3_PIN) != GPIO_PIN_RESET)  /* 霍尔传感器状态获取 */
        {
            state |= 0x04U;
        }
    }
    return state;
}

/************************************* BLDC相关函数 *************************************/

/**
  * @brief  关闭电机运转
  * @param  无
  * @retval 无
  */
void stop_motor1(void)
{
    /* 关闭半桥芯片输出 */
    SHUTDOWN_OFF;
    /* 关闭PWM输出 */
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
    /* 上下桥臂全部关断 */
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR3 = 0;
    /* 关闭下桥臂 */
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_GPIO_Port,M1_LOW_SIDE_U_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_GPIO_Port,M1_LOW_SIDE_V_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_GPIO_Port,M1_LOW_SIDE_W_Pin,GPIO_PIN_RESET);
}

/**
  * @brief  开启电机运转
  * @param  无
  * @retval 无
  */
void start_motor1(void)
{
    SHUTDOWN_EN;
    /* 使能PWM输出 */
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
}

/*************************** 上下桥臂的导通情况，共6种，也称为6步换向（接口一） ****************************/

/*  六步换向函数指针数组 */
pctr pfunclist_m1[6] =
{
    &m1_uhwl, &m1_vhul, &m1_vhwl,
    &m1_whvl, &m1_uhvl, &m1_whul
};

/**
  * @brief  U相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void m1_uhvl(void)
{
    htim1.Instance->CCR1 = g_bldc_motor1.pwm_duty;                 /* U相上桥臂PWM */
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_GPIO_Port,M1_LOW_SIDE_V_Pin,GPIO_PIN_SET);   /* V相下桥臂导通 */
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_GPIO_Port,M1_LOW_SIDE_U_Pin,GPIO_PIN_RESET); /* U相下桥臂关闭 */
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_GPIO_Port,M1_LOW_SIDE_W_Pin,GPIO_PIN_RESET); /* W相下桥臂关闭 */
}

/**
  * @brief  U相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void m1_uhwl(void)
{
    htim1.Instance->CCR1 = g_bldc_motor1.pwm_duty;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_GPIO_Port,M1_LOW_SIDE_W_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_GPIO_Port,M1_LOW_SIDE_U_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_GPIO_Port,M1_LOW_SIDE_V_Pin,GPIO_PIN_RESET);
}

/**
  * @brief  V相上桥臂导通，W相下桥臂导通
  * @param  无
  * @retval 无
  */
void m1_vhwl(void)
{
    htim1.Instance->CCR1=0;
    htim1.Instance->CCR2 = g_bldc_motor1.pwm_duty;
    htim1.Instance->CCR3=0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_GPIO_Port,M1_LOW_SIDE_W_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_GPIO_Port,M1_LOW_SIDE_U_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_GPIO_Port,M1_LOW_SIDE_V_Pin,GPIO_PIN_RESET);
}

/**
  * @brief  V相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void m1_vhul(void)
{
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = g_bldc_motor1.pwm_duty;
    htim1.Instance->CCR3 = 0;
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_GPIO_Port,M1_LOW_SIDE_U_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_GPIO_Port,M1_LOW_SIDE_V_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_GPIO_Port,M1_LOW_SIDE_W_Pin,GPIO_PIN_RESET);
}

/**
  * @brief  W相上桥臂导通，U相下桥臂导通
  * @param  无
  * @retval 无
  */
void m1_whul(void)
{
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = g_bldc_motor1.pwm_duty;

    HAL_GPIO_WritePin(M1_LOW_SIDE_U_GPIO_Port,M1_LOW_SIDE_U_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_GPIO_Port,M1_LOW_SIDE_V_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_GPIO_Port,M1_LOW_SIDE_W_Pin,GPIO_PIN_RESET);
}

/**
  * @brief  W相上桥臂导通，V相下桥臂导通
  * @param  无
  * @retval 无
  */
void m1_whvl(void)
{
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = g_bldc_motor1.pwm_duty;

    HAL_GPIO_WritePin(M1_LOW_SIDE_V_GPIO_Port,M1_LOW_SIDE_V_Pin,GPIO_PIN_SET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_GPIO_Port,M1_LOW_SIDE_U_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_GPIO_Port,M1_LOW_SIDE_W_Pin,GPIO_PIN_RESET);
}

/**
 * @brief       定时器中断回调
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)                                     /* 55us */
    {
#ifdef H_PWM_L_ON
        if(g_bldc_motor1.run_flag == RUN)
        {
            if(g_bldc_motor1.dir == CW)                                     /* 正转 */
            {
                g_bldc_motor1.step_sta = hallsensor_get_state(MOTOR_1);     /* 顺序6,2,3,1,5,4 */
            }
            else                                                            /* 反转 */
            {
                g_bldc_motor1.step_sta = 7 - hallsensor_get_state(MOTOR_1); /* 顺序5,1,3,2,6,4 。使用7减完后可与数组pfunclist_m1对应上顺序 实际霍尔值为：2,6,4,5,1,3*/
            }
            
            if((g_bldc_motor1.step_sta <= 6)&&(g_bldc_motor1.step_sta >= 1))/* 判断霍尔组合值是否正常 */
            {
                pfunclist_m1[g_bldc_motor1.step_sta-1]();                   /* 通过数组成员查找对应的函数指针 */
                
            }
            else                                                            /* 霍尔传感器错误、接触不良、断开等情况 */
            {
                stop_motor1();
                g_bldc_motor1.run_flag = STOP;
            }
        }

#endif
    }
}


