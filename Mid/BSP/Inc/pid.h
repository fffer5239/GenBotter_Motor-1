/**
 ****************************************************************************************************
 * @file        pid.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID算法头文件
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F407开发板
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
#ifndef __PID_H
#define __PID_H

#include "main.h"

/******************************************************************************************/
/* PID参数定义 */

#define  INCR_LOCT_SELECT  0    /* 0: 选择位置式PID算法, 1: 选择增量式PID算法 */

#if INCR_LOCT_SELECT
/* 增量式PID参数宏 */
#define  KP      0.00800f       /* 比例系数 P */
#define  KI      0.00025f       /* 积分系数 I */
#define  KD      0.00020f       /* 微分系数 D */
#define SMAPLSE_PID_SPEED  50   /* 采样周期, 单位ms */
#else
/* 位置式PID参数宏 */
#define  KP      1.600f         /* 比例系数 P */
#define  KI      0.00025f       /* 积分系数 I */
#define  KD      0.00020f       /* 微分系数 D */
#define SMAPLSE_PID_SPEED  40   /* 采样周期, 单位ms */
#endif

/******************************************************************************************/
/* PID结构体定义 */

typedef struct
{
    __IO float  SetPoint;       /* 设定目标值 */
    __IO float  ActualSpeed;   /* 实际值（位置或速度） */
    __IO float  ActualValue;    /* PID输出值 */
    __IO float  FeedbackValue;  /* 实际反馈值 */
    __IO float  SumError;       /* 误差累计值 */
    __IO float  Proportion;     /* 比例系数 P */
    __IO float  Integral;       /* 积分系数 I */
    __IO float  Derivative;     /* 微分系数 D */
    __IO float  Error;          /* 当前误差 Error[k] */
    __IO float  LastError;      /* 上次误差 Error[k-1] */
    __IO float  PrevError;      /* 上上次误差 Error[k-2] */
    __IO float  IngMin;         /* 积分限幅最小值 */
    __IO float  IngMax;         /* 积分限幅最大值 */
    __IO float  OutMin;         /* 输出限幅最小值 */
    __IO float  OutMax;         /* 输出限幅最大值 */
} PID_TypeDef;

/******************************************************************************************/
/* 外部变量声明 */

extern PID_TypeDef  g_location_pid;     /* 位置PID控制结构体 */
extern PID_TypeDef  g_speed_pid;        /* 速度PID控制结构体 */

/******************************************************************************************/
/* 外部接口函数 */

void pid_init(void);                                            /* PID参数初始化 */
int32_t increment_pid_ctrl(PID_TypeDef *PID, float Feedback_value);  /* PID计算函数 */

#endif /* __PID_H */
