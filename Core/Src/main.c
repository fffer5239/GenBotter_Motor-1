/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "key_led.h"
#include "bldc_motor.h"
#include "bldc_adc.h"
#include "stdio.h"
#include "pid.h"  
#include "vofa_plus.h"
#include "pid_param_parse.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern int32_t motor_pwm_s;
extern int32_t temp_pwm1;
extern int16_t adc_amp_un[3];
extern float  adc_amp_bus;

float*user_setpoint = (float*)(&g_speed_pid.SetPoint);    /* 设置目标值指针 指向存放目标值地址 */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init(); // 初始化DWT

  Key_Init();
  Led_Init();
  lcd_init();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim6);  // 启动TIM6中断,ADC采集
  adc_nch_dma_init();
  pid_init();
  PID_ParamParser_Init();
  lcd_show_string(10, 50, 300, 32, 32, "GenBotter-Motor-1", RED);
  lcd_show_string(10, 85, 450, 24, 24, "Chap06_LCD_KEY_LED_TempPro", BLUE);
  printf("Hello World!\n");
  int16_t pwm_duty_temp = 0;
  int8_t t;
  char buf[32];
  float current[3]= {0.0f};
  float current_lpf[4]= {0.0f};
  uint32_t vofa_plus_send_time = 0;
  uint32_t pid_param_lcd_flash = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    KeyPressedID key_id = KEY_None;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // printf("HallSensor State: 0x%02X\n", hallsensor_get_state(MOTOR_1));
    // HAL_Delay(100);
    #if 0    
    t++;
    if(t % 200 == 0)
    {

        sprintf(buf,"g_speed_pid.SetPoint:%.1f,pwm_duty:%d",
          (float)g_speed_pid.SetPoint,g_bldc_motor1.pwm_duty);
        lcd_show_string(10,150,400,16,16,buf,g_point_color);
        sprintf(buf,"Power:%.3fV ",g_adc_val[0]*ADC2VBUS);
        lcd_show_string(10,170,200,16,16,buf,g_point_color);
        sprintf(buf,"Temp:%.1fC ",get_temp(g_adc_val[1]));
        lcd_show_string(10,190,200,16,16,buf,g_point_color);           
        

        current[0] = adc_amp_un[0]* ADC2CURT;               /* 计算出三相电流值，U */
        current[1] = adc_amp_un[1]* ADC2CURT;               /* 计算出三相电流值，V */
        current[2] = adc_amp_un[2]* ADC2CURT;               /* 计算出三相电流值，W */
        
        /*一阶数字滤波 滤波系数0.1 用于显示*/
        FirstOrderRC_LPF(current_lpf[0],current[0],0.1f);   /* U相电流 */
        FirstOrderRC_LPF(current_lpf[1],current[1],0.1f);   /* V相电流 */
        FirstOrderRC_LPF(current_lpf[2],current[2],0.1f);   /* W相电流 */
        FirstOrderRC_LPF(current_lpf[3],adc_amp_bus,0.1f);  /* 母线电流 */
        
        if(g_bldc_motor1.run_flag == STOP)                  /* 停机的电流显示 */
        {
            current_lpf[0] = 0;
            current_lpf[1] = 0;
            current_lpf[2] = 0;
            current_lpf[3] = 0;
        }
        /* LCD显示提示信息 */
        sprintf(buf,"Amp U:%.3fmA ",(float)current_lpf[0]);
        lcd_show_string(10,210,200,16,16,buf,g_point_color);
        sprintf(buf,"Amp V:%.3fmA ",(float)current_lpf[1]);
        lcd_show_string(10,230,200,16,16,buf,g_point_color);
        sprintf(buf,"Amp W:%.3fmA ",(float)current_lpf[2]);
        lcd_show_string(10,250,200,16,16,buf,g_point_color);
        sprintf(buf,"Amp Bus:%.3fmA ",(float)current_lpf[3]);
        lcd_show_string(10,270,200,16,16,buf,g_point_color);
        sprintf(buf,"Speed:%.1frpm ",(float)g_bldc_motor1.speed);
        lcd_show_string(10,290,200,16,16,buf,g_point_color);
        
        /* 串口打印信息 */
        // printf("Valtage:%.1fV \r\n", g_adc_val[0]*ADC2VBUS);
        // printf("Temp:%.1fC \r\n", get_temp(g_adc_val[1]));
        // printf("U相电流为：%.3fmA\r\n", (current_lpf[0]));
        // printf("V相电流为：%.3fmA\r\n", (current_lpf[1]));
        // printf("W相电流为：%.3fmA\r\n", (current_lpf[2]));
        // printf("母线电流为：%.3fmA\r\n", (current_lpf[3]));
        // printf("\r\n");
        Led_Toggle(LED1);                          /* LED1(红灯) 翻转 */
        VOFA_Plus_SendSpeedLoopData(); // 发送当前速度环PID数据到VOFA+
    }
    #endif

    if(HAL_GetTick() - vofa_plus_send_time >= 100)
    {
      pid_param_lcd_flash ++;
      if(pid_param_lcd_flash >= 10)
      {
        pid_param_lcd_flash = 0;
        sprintf(buf,"KP:%.5f,KI:%.5f,KD:%.5f",
          (float)g_speed_pid.Proportion,(float)g_speed_pid.Integral,(float)g_speed_pid.Derivative);
        lcd_show_string(10, 150, 400, 16,16,buf,g_point_color);
      }
      vofa_plus_send_time = HAL_GetTick();
      VOFA_Plus_SendSpeedLoopData(); // 发送当前速度环PID数据到VOFA+
    }

    key_id = Key_Scan();
    if(key_id == KEY0_Pressed)
    { 
            lcd_show_string(10, 115, 200, 24, 24, "key 0 pressed.", BLUE);  
            g_bldc_motor1.run_flag = RUN;   /* 开启运行 */
            g_bldc_motor1.dir = CW;         /* 顺时针旋转 */
            start_motor1();                 /* 开启运行 */
            if(*user_setpoint == 0 && g_bldc_motor1.dir == CCW)
            {
                pid_init();                 /* 换向时刻，重新初始化PID，防止速度突变 */
                g_bldc_motor1.dir = CW;
            }
            
            *user_setpoint += 400;          /* 顺时针旋转下递增 */
            if(*user_setpoint >= 4000)
                *user_setpoint = 4000;
            if(*user_setpoint == 0)
            {
                g_bldc_motor1.run_flag = STOP; 
                stop_motor1();              /* 停机 */
                g_bldc_motor1.speed = 0;
                motor_pwm_s = 0;
                g_bldc_motor1.pwm_duty = 0;
                printf("Speed 0\r\n");
            }
    }
    else if(key_id == KEY1_Pressed)
    {
            lcd_show_string(10, 115, 200, 24, 24, "key 1 pressed.", BLUE);
            g_bldc_motor1.run_flag = RUN;   /* 开启运行 */
            g_bldc_motor1.dir = CCW;        /* 逆时针旋转 */
            start_motor1();                 /* 开启运行 */
            if(*user_setpoint == 0 && g_bldc_motor1.dir == CW)
            {     
                pid_init();
                g_bldc_motor1.dir = CCW;
            }
            *user_setpoint -= 400;          /* 逆时针旋转下递增 */
            if(*user_setpoint <= -4000)
                *user_setpoint = -4000;
            if(*user_setpoint == 0)
            {
                g_bldc_motor1.run_flag = STOP;  
                stop_motor1();              /* 停机 */
                g_bldc_motor1.speed = 0;
                motor_pwm_s = 0;
                g_bldc_motor1.pwm_duty = 0;
                printf("Speed 0\r\n");
            }      
    }
    else if(key_id == KEY2_Pressed){
        lcd_show_string(10, 115, 200, 24, 24, "key 2 pressed.", BLUE);
        stop_motor1();                          /* 停机 */
        g_bldc_motor1.run_flag = STOP;          /* 标记停机 */
        pwm_duty_temp = 0;                      /* 数据清0 */
        g_bldc_motor1.pwm_duty = 0;
    }  
    
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
