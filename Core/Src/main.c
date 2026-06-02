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
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "key_led.h"
#include "stepper_motor.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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
__IO uint32_t g_set_speed  = 1000;          /* 最大速度 单位为0.1rad/sec */
__IO uint32_t g_step_accel = 25;            /* 加速度 单位为0.1rad/sec^2 */
__IO uint32_t g_step_decel = 20;            /* 减速度 单位为0.1rad/sec^2 */
__IO uint16_t g_step_angle = 0;             /* 设置的步数*/
extern __IO uint32_t g_add_pulse_count;     /* 脉冲个数累计*/
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
  MX_FSMC_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init(); // 初始化DWT

  Key_Init();
  Led_Init();
  lcd_init();
  
  lcd_show_string(10, 50, 300, 32, 32, "GenBotter-Motor-1", RED);
  // lcd_show_string(10, 85, 450, 24, 24, "Chap06_LCD_KEY_LED_TempPro", BLUE);

  KeyPressedID key_id = KEY_None;
  uint8_t t;
  char buf[32];
  uint32_t apb2_freq_LCD = HAL_RCC_GetPCLK2Freq();
  lcd_show_num(10, 85, apb2_freq_LCD, 10, 24, RED);
  printf("Hello World!\n");
  printf("KEY0开启梯形加减速\r\n");
  printf("KEY1 add step\r\n");
  printf("KEY2 sub step\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// 功能
    // key_id = Key_Scan();
    // if(key_id == KEY0_Pressed){
    //     lcd_show_string(10, 115, 200, 24, 24, "key 0 pressed.", BLUE);
    //      // 调速的操作
    //     if(current_enable == STEPPER_ENABLE){
    //       // Stepper_SetSpeed(STEPPER_1, speed_table[speed_gear]);
    //       stepper_set_angle(STEPPER_1, speed_table[speed_gear]);
    //       speed_gear++;
    //       if(speed_gear > 2){
    //         speed_gear = 0;
    //       }

    //     }
    // }
    // else if(key_id == KEY1_Pressed){
    //     lcd_show_string(10, 115, 200, 24, 24, "key 1 pressed.", BLUE);
    //     // 旋转方向的操作
    //     current_dir = current_dir == STEPPER_DIR_CW ? STEPPER_DIR_CCW : STEPPER_DIR_CW;
    //     Stepper_SetDir(STEPPER_1, current_dir);
    //     // 用LED0标识方向，cw亮、ccw熄灭
    //     if(current_dir == STEPPER_DIR_CW){
    //       Led_On(LED1);
    //     }
    //     else{
    //       Led_Off(LED1);
    //     }
    // }
    // else if(key_id == KEY2_Pressed){
    //     lcd_show_string(10, 115, 200, 24, 24, "key 2 pressed.", BLUE);
    //     current_enable = current_enable == STEPPER_ENABLE ? STEPPER_DISABLE : STEPPER_ENABLE;
    //     Stepper_SetEnable(STEPPER_1, current_enable);
    //     // 用LED1标识步进Enable，enable亮、disable熄灭
    //     if (current_enable == STEPPER_ENABLE)
    //     {
    //       Led_On(LED2);
    //       printf("stepper1 enable\r\n");
    //     }
    //     else
    //     {
    //       Led_Off(LED2);
    //       printf("stepper1 disable\r\n");
    //     }
    // }


        t++;
        if(t % 200 == 0)
        {            
            sprintf(buf,"Set_Aangle:%d     ",g_step_angle);             /* 设置的旋转位置（角度）*/
            lcd_show_string(10,50,300,32,32,buf,RED);
            sprintf(buf,"Add_Aangle:%.2f    ",g_add_pulse_count*0.1125); /* 累计旋转的角度 */
            lcd_show_string(10,110,300,32,32,buf,RED);
            Led_Toggle(LED1);                                              /* LED1(红灯) 翻转 */        
        }
        key_id = Key_Scan();
        if(key_id == KEY0_Pressed)                                            /* 开启梯形加减速 */
        {
            create_t_ctrl_param(SPR*g_step_angle, g_step_accel, g_step_decel, g_set_speed);
            g_add_pulse_count=0;
        }
        else if(key_id == KEY1_Pressed)                                       /* 增加步数 */
        {
            g_step_angle+=10;
            if(g_step_angle>100)  g_step_angle=1;
        }
        else if(key_id == KEY2_Pressed)                                       /* 减少步数 */
        {
            g_step_angle-=10;
            if(g_step_angle<1)  g_step_angle=100;
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
