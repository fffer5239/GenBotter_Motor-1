#ifndef __STEPPER_MOTOR_H_
#define __STEPPER_MOTOR_H_

#include "main.h"

/**一些宏定义**/

extern TIM_HandleTypeDef htim8;

/*硬件资源的宏定义*/

//stepper1的宏,
//ST1_EN->PF15,ST1_DIR->PF14,ST1_STEP->PI5(步进脉冲引脚,PWM通道)
//ST1_TIM->TIM8,ST1_TIM_CHANNEL->TIM_CHANNEL_1
#define ST1_EN_PORT      GPIOF
#define ST1_EN_PIN       GPIO_PIN_15
#define ST1_DIR_PORT     GPIOF
#define ST1_DIR_PIN      GPIO_PIN_14
#define ST1_STEP_PORT    GPIOI      
#define ST1_STEP_PIN     GPIO_PIN_5
#define ST1_TIM          &htim8
#define ST1_TIM_CHANNEL  TIM_CHANNEL_1

//stepper2的宏,
//ST2_EN->PF13,ST2_DIR->PF12,ST2_STEP->PI6
//ST2_TIM->TIM8,ST2_TIM_CHANNEL->TIM_CHANNEL_2
#define ST2_EN_PORT      GPIOF
#define ST2_EN_PIN       GPIO_PIN_13
#define ST2_DIR_PORT     GPIOF
#define ST2_DIR_PIN      GPIO_PIN_12
#define ST2_STEP_PORT    GPIOI
#define ST2_STEP_PIN     GPIO_PIN_6
#define ST2_TIM          &htim8
#define ST2_TIM_CHANNEL  TIM_CHANNEL_2

//stepper3的宏,
//ST3_EN->PF11,ST3_DIR->PB2,ST3_STEP->PI7
//ST3_TIM->TIM8,ST3_TIM_CHANNEL->TIM_CHANNEL_3
#define ST3_EN_PORT      GPIOF
#define ST3_EN_PIN       GPIO_PIN_11
#define ST3_DIR_PORT     GPIOB
#define ST3_DIR_PIN      GPIO_PIN_2
#define ST3_STEP_PORT    GPIOI
#define ST3_STEP_PIN     GPIO_PIN_7
#define ST3_TIM          &htim8
#define ST3_TIM_CHANNEL  TIM_CHANNEL_3

//stepper4的宏,
//ST4_EN->PH3,ST4_DIR->PH2,ST4_STEP->PC9
//ST4_TIM->TIM8,ST4_TIM_CHANNEL->TIM_CHANNEL_4
#define ST4_EN_PORT      GPIOH
#define ST4_EN_PIN       GPIO_PIN_3
#define ST4_DIR_PORT     GPIOH
#define ST4_DIR_PIN      GPIO_PIN_2
#define ST4_STEP_PORT    GPIOC
#define ST4_STEP_PIN     GPIO_PIN_9
#define ST4_TIM          &htim8
#define ST4_TIM_CHANNEL  TIM_CHANNEL_4

//最小和最大的步进脉冲频率
#define STEPPER_MIN_RPM  1
#define STEPPER_MAX_RPM  30000

/* 步进电机参数相关宏 */
#define PULSE_REV       3200.0              /* 每圈脉冲数（细分数16） */
#define MAX_STEP_ANGLE  0.1125               /* 最小步距(1.8/PULSE_REV) */

#define STEPPER_PRESCALER  84 //预分频系数 */
#define STEPPER_PERIOD    1000 //PWM周期 */

/**一些enum**/
//步进电机接口组的ID
typedef enum{
    STEPPER_1 = 0,
    STEPPER_2,
    STEPPER_3,
    STEPPER_4,
    STEPPER_SUM //表示步进电机接口组的数量
}StepperID;

//步进电机的方向
typedef enum{
    STEPPER_DIR_CW = 0,
    STEPPER_DIR_CCW
}StepperDir;

//步进电机的使能状态
typedef enum{
    STEPPER_DISABLE = 0,
    STEPPER_ENABLE
}StepperEnableState;

//错误码
typedef enum{
    STEPPER_OK = 0,
    STEPPER_ERROR,
    STEPPER_INVALID_ID,
    STEPPER_INVALID_PARAM
}StepperErrorCode;

/*一些结构体*/
typedef struct{
    GPIO_TypeDef *en_port;
    uint16_t en_pin;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    GPIO_TypeDef *step_port;
    uint16_t step_pin;
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;
}StepperMotorHwRes;

// 步进电机工作状态的结构体
typedef struct{
    StepperEnableState enable;
    StepperDir dir;           /* 方向 */
    uint32_t speed;           /* 设置需要旋转的角度 */
    StepperErrorCode error; //最近的错误码
    volatile uint32_t pulse_count;          /* 脉冲个数记录 */
    volatile int add_pulse_count;           /* 脉冲个数累计 */  
}StepperStatus;


/*************************************加减速曲线参数**********************************/

#define TIM_FREQ            168000000U                      /* 定时器主频 */
#define MAX_STEP_ANGLE      0.1125                           /* 最小步距(1.8/MICRO_STEP) */
#define PAI                 3.1415926                       /* 圆周率*/
#define FSPR                200                             /* 步进电机单圈步数 */
#define MICRO_STEP          16                               /* 步进电机驱动器细分数 */
#define T1_FREQ             (TIM_FREQ/84)                   /* 频率ft值 */
#define SPR                 (FSPR*MICRO_STEP)               /* 旋转一圈需要的脉冲数 */

/* 数学常数 */

#define ALPHA               ((float)(2*PAI/SPR))            /* α = 2*pi/spr */
#define A_T_x10             ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148         ((float)((T1_FREQ*0.69)/10))    /* 0.69为误差修正值 */
#define A_SQ                ((float)(2*100000*ALPHA))
#define A_x200              ((float)(200*ALPHA))            /* 2*10*10*a/10 */

typedef struct
{
    __IO uint8_t  run_state;                                /* 电机旋转状态 */
    __IO uint8_t  dir;                                      /* 电机旋转方向 */
    __IO int32_t  step_delay;                               /* 下个脉冲周期（时间间隔），启动时为加速度 */
    __IO uint32_t decel_start;                              /* 开始减速位置 */
    __IO int32_t  decel_val;                                /* 减速阶段步数 */
    __IO int32_t  min_delay;                                /* 速度最快，计数值最小的值(最大速度，即匀速段速度) */
    __IO int32_t  accel_count;                              /* 加减速阶段计数值 */
} speedRampData;

enum STA
{
    STOP = 0,                                               /* 加减速曲线状态：停止*/
    ACCEL,                                                  /* 加减速曲线状态：加速阶段*/
    DECEL,                                                  /* 加减速曲线状态：减速阶段*/
    RUN                                                     /* 加减速曲线状态：匀速阶段*/
};
 
/***********************************************************************************/

/* 一些接口函数的声明 */
StepperErrorCode Stepper_Init(StepperID id);
StepperErrorCode Stepper_SetSpeed(StepperID id, uint32_t speed); //步进电机控制脉冲的速度
StepperErrorCode Stepper_SetDir(StepperID id, StepperDir dir);
StepperErrorCode Stepper_SetEnable(StepperID id, StepperEnableState enable);
StepperErrorCode Stepper_GetStatus(StepperID id, StepperStatus *status);

void stepper_set_angle(StepperID id, uint16_t angle);/* 将角度转换成脉冲个数 */

void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed); /* 梯形加减速控制函数 */    

#endif /* __STEPPER_MOTOR_H_ */
