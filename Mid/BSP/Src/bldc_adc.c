/**
 ****************************************************************************************************
 * @file        bldc_adc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-18
 * @brief       ADC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211018
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "bldc_adc.h"

#include "adc.h"
#include <math.h>

/* 多通道ADC采集 DMA读取 */
// ADC_HandleTypeDef g_adc_nch_dma_handle;             /* 与DMA关联的ADC句柄 */
// DMA_HandleTypeDef g_dma_nch_adc_handle;             /* 与ADC关联的DMA句柄 */
uint8_t g_adc_dma_sta = 0;                          /* DMA传输状态标志, 0,未完成; 1, 已完成 */

uint16_t g_adc_value[ADC_CH_NUM * ADC_COLL] = {0};  /* 存储ADC原始值 */
float g_adc_u_value[ADC_CH_NUM] = {0};              /* 存储ADC转换后的电压值 */

/***************************************多通道ADC采集(DMA读取)程序*****************************************/

/**
 * @brief       adc 初始化函数
 * @note        配置ADC转换通道
 * @param       无
 * @retval      无
 */
void adc_init(void)
{
    
}

/**
 * @brief       ADC DMA读取 初始化函数
 * @note        本函数还是使用adc_init对ADC进行大部分配置,有差异的地方再单独配置
 * @param       par         : 外设地址
 * @param       mar         : 存储器地址
 * @retval      无
 */
void adc_nch_dma_init(void)
{
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)g_adc_value,ADC_CH_NUM * ADC_COLL);
}

/*************************************    第二部分    电压电流温度采集    **********************************************/
/*
    Rt = Rp *exp(B*(1/T1-1/T2))
    Rt 是热敏电阻在T1温度下的阻值；
    Rp是热敏电阻在T2常温下的标称阻值；
    exp是e的n次方，e是自然常数，就是自然对数的底数，近似等于 2.7182818；
    B值是热敏电阻的重要参数，教程中用到的热敏电阻B值为3380；
    这里T1和T2指的是开尔文温度，T2是常温25℃，即(273.15+25)K
    T1就是所求的温度
*/

const float Rp = 10000.0f;                  /* 10K */
const float T2 = (273.15f + 25.0f);         /* T2 */
const float Bx = 3380.0f;                   /* B */
const float Ka = 273.15f;

/**
 * @brief       计算温度值
 * @param       para: 温度采集对应ADC通道的值（已滤波）
 * @note        计算温度分为两步：
                1.根据ADC采集到的值计算当前对应的Rt
                2.根据Rt计算对应的温度值
 * @retval      温度值
 */
float get_temp(uint16_t para)
{
    float r_ntc;
    float temp_k;
    float temp_adc_value;
    float temp_voltage;
    float temp_value;

    /* 1. 直接读取 ADC 滤波后的数据 */
    temp_adc_value = para;

    /* 防止异常值 */
    if (temp_adc_value < 1.0f)
    {
        temp_adc_value = 1.0f;
    }
    /* 2. ADC → 电压 */
    temp_voltage = temp_adc_value * (TEMP_ADC_REF_VOLTAGE / TEMP_ADC_RESOLUTION);
    /* 3. 电压 → NTC 阻值
     * Rntc = Rpull × (Vref / V - 1)
     */
    r_ntc = NTC_PULL_DOWN_RESISTOR *
            (TEMP_ADC_REF_VOLTAGE / temp_voltage - 1.0f);
    /* 4. NTC → 温度（B 参数公式） */
    temp_k = 1.0f /
            ( (1.0f / NTC_T0)
            + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0) );
    temp_value = temp_k - 273.15f;
    return temp_value;
}
/**
 * @brief       计算ADC的平均值（滤波）
 * @param       * p ：存放ADC值的指针地址
 * @note        此函数对电压、温度、电流对应的ADC值进行滤波
 * @retval      无
 */
void calc_adc_val(uint16_t * p)
{
    uint32_t temp[ADC_CH_NUM] = {0,0,0};            /* 定义一个缓存数组 */
    int i,j;
    for(i = 0; i < ADC_COLL; i++)                   /* 循环采集ADC_COLL次数 */
    {
        for(j = 0; j < ADC_CH_NUM; j++)             /* 根据ADC通道数循环获取，并累加 */
        {
            temp[j] += g_adc_value[j+i*ADC_CH_NUM]; /* 将采集到的ADC值，各通道进行累加 */
        }
    }
    for(j = 0; j < ADC_CH_NUM; j++)
    {
        temp[j] /= ADC_COLL;                        /* 获取平均值 */
        p[j] = temp[j];                             /* 存到*p */
    }
}


uint16_t g_adc_val[ADC_CH_NUM];                     /* ADC平均值存放数组 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)                     /* 大约2.6ms采集完成进入中断 */
    { 
        HAL_ADC_Stop_DMA(&hadc1);    /* 关闭DMA转换 */
        calc_adc_val(g_adc_val);                    /* ADC数值转换 */
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&g_adc_value, (uint32_t)(ADC_SUM)); /* 再启动DMA转换*/
    }
}


/**
 * @brief       获取通道ch的转换值，取times次, 然后平均
 * @param       ch: 通道号, 0~17
 * @retval      通道ch的times次转换结果平均值
 */
uint32_t adc_get_result_average(uint8_t ch)
{
    uint32_t temp_val = 0;
    uint16_t t;

    for (t = ch; t < ADC_SUM; t += ADC_CH_NUM )     /* 获取times次数据 */
    {
        temp_val += g_adc_value[t];
    }

    return temp_val / ADC_COLL;                     /* 返回平均值 */
}
