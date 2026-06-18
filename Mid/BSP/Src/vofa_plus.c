#include "vofa_plus.h"
#include "usart.h"


#include "key_led.h"


#if VOFA_PLUS_ENABLE

// 静态发送缓冲区，避免每次调用都分配内存
static uint8_t vofa_send_buf[VOFA_SEND_BUF_MAX_LEN] = {0};

/******************** VOFA+核心函数实现 ********************/
/**
 * @brief  VOFA+模块初始化
 * @note   串口初始化建议在main.c中统一做，此处仅做空实现（避免重复初始化）
 */
void VOFA_Plus_Init(void) {
    // VOFA+串口初始化建议在main.c中统一做，此处仅做空实现（避免重复初始化）
    // 例如：MX_USART1_UART_Init();

    // 清空发送缓冲区，避免脏数据干扰调试
    memset(vofa_send_buf, 0, sizeof(vofa_send_buf));
}

/**
 * @brief 封装VOFA+待发送数据（FireWater协议）
 * @param vofa_data: 待发送的数据结构体
 * @param send_buf: 输出缓冲区
 * @param buf_len: 缓冲区长度
 * @retval 封装后的数据长度
 */
uint16_t VOFA_Plus_PackData(VOFA_Data_t *vofa_data, uint8_t *send_buf, uint16_t buf_len) {
    // 简单封装示例：将结构体数据按顺序转换为字节流
    // 注意：实际使用时需根据VOFA+协议要求进行封装，如添加帧头、校验等
    if (buf_len < sizeof(VOFA_Data_t)) {
        return 0; // 缓冲区不足，无法封装
    }

#if VOFA_PROTOCOL == 1 // FireWater协议示例（字符串格式）
    // 以字符串格式封装数据，便于调试和查看
    if (buf_len < 32) { // 字符串格式需要更大的缓冲区
        return 0; // 缓冲区不足，无法封装
    }

    // FireWater协议示例：以逗号分隔的字符串格式，回车换行结尾 "RPM:100.0,ACT:95.5,PID:20.0,OUT:25.0\r\n"
    uint16_t len = snprintf((char *)send_buf, buf_len, "%.1f,%.1f,%.1f\r\n",
                            vofa_data->SetPoint,
                            vofa_data->ActualSpeed,
                            vofa_data->ActualValue
                        );
    return len; // 返回封装后的数据长度

#elif VOFA_PROTOCOL == 2 // RawData格式（暂未测试）

    // 以RawData格式封装数据，直接内存复制到发送缓冲区
    if (buf_len < 16) { // 4个float数据需要16字节缓冲区
        return 0; // 缓冲区不足，无法封装
    }
    // RawData协议：按顺序发送4个float，注意字节序和对齐问题
    memcpy(send_buf, &vofa_data->SetPoint, sizeof(float));                     // 目标值
    memcpy(send_buf + sizeof(float), &vofa_data->ActualValue, sizeof(float));  // PID输出值
    memcpy(send_buf + sizeof(float) * 2, &vofa_data->FeedbackValue, sizeof(float)); // 反馈值
    memcpy(send_buf + sizeof(float) * 3, &vofa_data->SumError, sizeof(float));      // 误差累计
    return 16; // 返回4个float的字节数
#else // 默认RawData格式，直接内存复制结构体到发送缓冲区
    return 0; // 协议选择错误，无法封装
#endif
}

/**
 * @brief  VOFA+模块发送数据到串口（封装上层接口，简化业务代码调用）
 * @note   无需手动传参，该函数可在speed_loop.c的主循环或定时器回调中调用，确保实时性
 */
void VOFA_Plus_SendData(VOFA_Data_t *data) {
    if(data == NULL) {
        return;
    }

    #if 0
    // DMA模式：必须等上一次传输完成，否则HAL_UART_Transmit_DMA返回HAL_BUSY
    if(VOFA_UART_HANDLE.gState != HAL_UART_STATE_READY) {
        // 调试：如果卡在这里，说明DMA中断没触发，回调链断了
        static uint32_t skip_cnt = 0;
        if(++skip_cnt > 50) {
            // 连续50次忙，强制中止，恢复UART状态
            HAL_UART_AbortTransmit(&VOFA_UART_HANDLE);
            skip_cnt = 0;
        }
        return;
    }
    #endif

    // 封装数据
    uint16_t data_len = VOFA_Plus_PackData(data, vofa_send_buf, VOFA_SEND_BUF_MAX_LEN);
    if(data_len == 0) {
        return;
    }

    #if VOFA_UART_DMA_ENABLE
    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(&VOFA_UART_HANDLE, vofa_send_buf, data_len);
    if(ret != HAL_OK) {
        // 发送失败，可通过ret值排查：HAL_BUSY=串口忙, HAL_ERROR=参数错误
        Led_Toggle(LED2);
    }
    #else
    HAL_UART_Transmit(&VOFA_UART_HANDLE, vofa_send_buf, data_len, 10);
    #endif
}

// 简化接口：获取当前速度环PID数据并发送到VOFA+
void VOFA_Plus_SendSpeedLoopData(void) {
    VOFA_Plus_SendData(&g_speed_pid);
}

/**
 * @brief  DMA发送完成回调（HAL回调，DMA中断链的最后一环）
 * @note   如果这个函数没被调用，说明DMA中断链断了
 *         可在此打断点或翻转LED来确认回调是否触发
 */
#if VOFA_UART_DMA_ENABLE
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        // DMA发送完成，gState已由HAL自动恢复为READY
        // Led_Toggle(LED2);  // 用LED2区分主循环的LED1
    }
}
#endif
#endif
