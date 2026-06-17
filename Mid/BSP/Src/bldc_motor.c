#include "bldc_motor.h"

#include "tim.h"
#include "gpio.h"

#include "bldc_adc.h"

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

/***********************************************定时器中断回调函数***********************************************/
/**
 * @brief       定时器中断回调
 * @param       无
 * @retval      无
 */
int32_t  temp_pwm1=0.0;
int32_t motor_pwm_s= 0;

#define ADC_AMP_OFFSET_TIMES 50                     /* 停机状态三相电流的ADC采集次数 */
uint16_t adc_amp_offset[3][ADC_AMP_OFFSET_TIMES+1]; /* 停机状态下的ADC数据缓冲区 */
uint8_t adc_amp_offset_p = 0;
int16_t adc_amp[3];

int16_t adc_amp_un[3];                  
float  adc_amp_bus = 0.0f;

volatile uint16_t adc_val_m1[ADC_CH_NUM];           /* ADC数据缓冲区 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t bldc_dir=0;
    uint8_t i;
    static uint8_t times_count=0;           /* 定时器时间记录 */
    int16_t temp_speed=0;                   /* 临时速度存储 */
    if(htim->Instance == TIM1)     /* 55us */
    {
#ifdef H_PWM_L_ON
        if(g_bldc_motor1.run_flag == RUN)
        {
            g_bldc_motor1.count_j++;
            if(g_bldc_motor1.dir == CW)     /* 顺时针旋转 */
            {
                g_bldc_motor1.step_sta = hallsensor_get_state(MOTOR_1);
            }
            else                            /* 逆时针旋转 */
            {
                g_bldc_motor1.step_sta = 7 - hallsensor_get_state(MOTOR_1);
            }
            if((g_bldc_motor1.step_sta <= 6)&&(g_bldc_motor1.step_sta >= 1))
            {
                pfunclist_m1[g_bldc_motor1.step_sta-1]();
            }
            else                            /* 编码器错误、接触不良、断开等情况 */
            {
                stop_motor1();
                g_bldc_motor1.run_flag = STOP;
            }
            #if 0  // 霍尔信号检测
            g_bldc_motor1.hall_sta_edge = uemf_edge(g_bldc_motor1.hall_single_sta); /* 检测单个霍尔信号的变化 */
            if(g_bldc_motor1.hall_sta_edge == 0)                                    /* 统计单个霍尔信号的高电平时间 */
            {
                /* 计算速度 */
                if(g_bldc_motor1.dir == CW)
                    temp_speed = (SPEED_COEFF/g_bldc_motor1.count_j);
                else
                    temp_speed = -(SPEED_COEFF/g_bldc_motor1.count_j);
                FirstOrderRC_LPF(g_bldc_motor1.speed, temp_speed, 0.2379);          /* 一阶滤波 */
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 1)                                    /* 当采集到下降沿时数据清0 */
            {
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 2)
            {
                g_bldc_motor1.no_single++;                                          /* 不换相时间累计 超时则判定速度为0 */
                
                if(g_bldc_motor1.no_single > 15000)
                {
                    
                    g_bldc_motor1.no_single = 0;
                    g_bldc_motor1.speed = 0;                                        /* 超时换向 判定为停止 速度为0 */
                }
            }
            if(g_bldc_motor1.step_last != g_bldc_motor1.step_sta)
            {
                g_bldc_motor1.hall_keep_t = 0;
                bldc_dir = check_hall_dir(&g_bldc_motor1);
                if(bldc_dir == CCW)
                {
                    g_bldc_motor1.pos -= 1;
                }
                else if(bldc_dir == CW)
                {
                    g_bldc_motor1.pos += 1;
                }
                g_bldc_motor1.step_last = g_bldc_motor1.step_sta;
            }
            else if(g_bldc_motor1.run_flag == RUN)                                      /* 运行且霍尔保持时 */
            {
                g_bldc_motor1.hall_keep_t++;                                            /* 换向一次所需计数值（时间） 单位1/18k */
            }     
            #endif  
            /* 三相电流采集 */
            for(i = 0; i < 3; i++)
            {
                adc_val_m1[i] = g_adc_val[i+2];
                adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][ADC_AMP_OFFSET_TIMES];   /* 运动状态ADC值 - 停机状态ADC值 = 实际作用ADC值 */
                if(adc_amp[i] >= 0)                                                     /* 去除反电动势引起的负电流数据 */
                    adc_amp_un[i] = adc_amp[i];
            }
            /* 运算母线电流（母线电流为任意两个有开关动作的相电流之和） */
            if(g_bldc_motor1.step_sta == 0x05)
            {
                adc_amp_bus= (adc_amp_un[0] + adc_amp_un[1])*ADC2CURT;   /* UV */
            }
            else if(g_bldc_motor1.step_sta == 0x01)
            {
                adc_amp_bus= (adc_amp_un[0] + adc_amp_un[2])*ADC2CURT;   /* UW */
            }
            else if(g_bldc_motor1.step_sta == 0x03)
            {
                adc_amp_bus= (adc_amp_un[1] + adc_amp_un[2])*ADC2CURT;   /* VW */
            }
            else if(g_bldc_motor1.step_sta == 0x02)
            {
                adc_amp_bus= (adc_amp_un[0] + adc_amp_un[1])*ADC2CURT;   /* UV */
            }
            else if(g_bldc_motor1.step_sta == 0x06)
            {
                adc_amp_bus= (adc_amp_un[0] + adc_amp_un[2])*ADC2CURT;   /* WU */
            }
            else if(g_bldc_motor1.step_sta == 0x04)
            {
                adc_amp_bus= (adc_amp_un[2] + adc_amp_un[1])*ADC2CURT;   /* WV */
            }         
        }
#endif
    }
    else if(htim->Instance == TIM6)
    {
        /* 计算未开始启动时的基准电压 */
        times_count++;
        if(g_bldc_motor1.run_flag == STOP)
        {
            uint8_t i;
            uint32_t avg[3] = {0,0,0};
            adc_amp_offset[0][adc_amp_offset_p] = g_adc_val[2];     /* 获取电机停机状态下的三相电流 U */
            adc_amp_offset[1][adc_amp_offset_p] = g_adc_val[3];     /* V */
            adc_amp_offset[2][adc_amp_offset_p] = g_adc_val[4];     /* W */
            adc_amp_offset_p ++;
            NUM_CLEAR(adc_amp_offset_p,ADC_AMP_OFFSET_TIMES);       /* 如果溢出，从头开始计数 */
            for(i = 0; i < ADC_AMP_OFFSET_TIMES; i++)
            {
                avg[0] += adc_amp_offset[0][i];                     /* 各相数值累加 */
                avg[1] += adc_amp_offset[1][i];
                avg[2] += adc_amp_offset[2][i];
            }
            for(i = 0; i < 3; i++)
            {
                avg[i] /= ADC_AMP_OFFSET_TIMES;                     /* 取平均 */
                adc_amp_offset[i][ADC_AMP_OFFSET_TIMES] = avg[i];   /* 赋值 */
            }
        }
    }
}
