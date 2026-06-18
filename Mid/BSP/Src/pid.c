/**
 ****************************************************************************************************
 * @file        pid.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID算法实现
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407开发板
 * 论坛    :www.yuanzige.com
 * 官方论坛:www.openedv.com
 * 公司地址:www.alientek.com
 * 淘宝地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 首次发布
 *
 ****************************************************************************************************
 */

#include "pid.h"

/******************************************************************************************/
/* 全局变量定义 */

PID_TypeDef  g_location_pid;    /* 位置PID控制结构体 */
PID_TypeDef  g_speed_pid;       /* 速度PID控制结构体 */

/**
 * @brief       PID参数初始化
 * @param       无
 * @retval      无
 */
void pid_init(void)
{
    g_speed_pid.SetPoint = 0;           /* 设定目标值 */
    g_speed_pid.ActualValue = 0.0f;     /* 输出值清零 */
    g_speed_pid.SumError = 0.0f;        /* 误差累计清零 */
    g_speed_pid.Error = 0.0f;           /* 当前误差清零 */
    g_speed_pid.LastError = 0.0f;       /* 上次误差清零 */
    g_speed_pid.PrevError = 0.0f;       /* 上上次误差清零 */
    g_speed_pid.Proportion = KP;        /* 比例系数 P */
    g_speed_pid.Integral = KI;          /* 积分系数 I */
    g_speed_pid.Derivative = KD;        /* 微分系数 D */
    g_speed_pid.IngMax = 6000;          /* 积分限幅最大值 */
    g_speed_pid.IngMin = -6000;         /* 积分限幅最小值 */
    g_speed_pid.OutMax = 6000;          /* 输出限幅最大值 */
    g_speed_pid.OutMin = -6000;         /* 输出限幅最小值 */
}

/**
 * @brief       PID计算函数（支持位置式和增量式）
 * @param       PID     : PID结构体指针
 * @param       Feedback_value : 实际反馈值
 * @retval      PID计算输出值
 */
int32_t increment_pid_ctrl(PID_TypeDef *PID, float Feedback_value)
{
    PID->ActualSpeed = Feedback_value;   /* 更新实际速度 */
    PID->Error = (float)(PID->SetPoint - Feedback_value);   /* 计算当前误差 */

#if  INCR_LOCT_SELECT
    /* 增量式PID计算 */
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))            /* 比例项 P */
                      + (PID->Integral * PID->Error)                                 /* 积分项 I */
                      + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* 微分项 D */
    PID->PrevError = PID->LastError;    /* 保存误差，用于下次计算 */
    PID->LastError = PID->Error;
#else
    /* 位置式PID计算 */
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)               /* 比例项 P */
                     + (PID->Integral * PID->SumError)              /* 积分项 I */
                     + (PID->Derivative * (PID->Error - PID->LastError));  /* 微分项 D */
    PID->LastError = PID->Error;
#endif

    /* 输出限幅 */
    if(PID->ActualValue > PID->OutMax)
    {
        PID->ActualValue = PID->OutMax;
    }
    else if(PID->ActualValue < PID->OutMin)
    {
        PID->ActualValue = PID->OutMin;
    }

    return ((int32_t)(PID->ActualValue));   /* 返回实际控制输出值 */
}
