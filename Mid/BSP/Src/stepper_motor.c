#include "stepper_motor.h"
#include "stdio.h"
#include "math.h"

// 步进电机硬件资源的结构体数组
const StepperMotorHwRes stepper_hw_res[STEPPER_SUM] = {
    {ST1_EN_PORT, ST1_EN_PIN, ST1_DIR_PORT, ST1_DIR_PIN, 
    ST1_STEP_PORT, ST1_STEP_PIN, ST1_TIM, ST1_TIM_CHANNEL},
    {ST2_EN_PORT, ST2_EN_PIN, ST2_DIR_PORT, ST2_DIR_PIN, 
    ST2_STEP_PORT, ST2_STEP_PIN, ST2_TIM, ST2_TIM_CHANNEL},
    {ST3_EN_PORT, ST3_EN_PIN, ST3_DIR_PORT, ST3_DIR_PIN, 
    ST3_STEP_PORT, ST3_STEP_PIN, ST3_TIM, ST3_TIM_CHANNEL},
    {ST4_EN_PORT, ST4_EN_PIN, ST4_DIR_PORT, ST4_DIR_PIN, 
    ST4_STEP_PORT, ST4_STEP_PIN, ST4_TIM, ST4_TIM_CHANNEL}
};

// 步进电机的状态结构体数组
static StepperStatus stepper_status[STEPPER_SUM] ={
    {STEPPER_DISABLE, STEPPER_DIR_CW, 0, STEPPER_OK,0,0},
    {STEPPER_DISABLE, STEPPER_DIR_CW, 0, STEPPER_OK,0,0},
    {STEPPER_DISABLE, STEPPER_DIR_CW, 0, STEPPER_OK,0,0}, 
    {STEPPER_DISABLE, STEPPER_DIR_CW, 0, STEPPER_OK,0,0}
};

StepperErrorCode Stepper_Init(StepperID id){
    if(id >= STEPPER_SUM){
        return STEPPER_INVALID_ID;
    }

    // 初始化电机，硬件外设（GPIO等）的初始化工作已经在CubeMX生成的代码中完成

    // 初始化步进电机的默认状态
    stepper_status[id].enable = STEPPER_DISABLE;
    stepper_status[id].dir = STEPPER_DIR_CW;
    stepper_status[id].speed = 0;
    stepper_status[id].error = STEPPER_OK;

    // 使能步进电机
    Stepper_SetEnable(id, STEPPER_DISABLE);

    // 设置步进电机的旋转方向
    Stepper_SetDir(id, STEPPER_DIR_CW);

    // 设置步进电机的速度
    Stepper_SetSpeed(id, 0);

    return STEPPER_OK;
}

//步进电机控制
StepperErrorCode Stepper_SetSpeed(StepperID id, uint32_t speed){

    if(id >= STEPPER_SUM){
        return STEPPER_INVALID_ID;
    }
    if(speed == 0){
        Stepper_SetEnable(id, STEPPER_DISABLE);
    }
    else if(speed > STEPPER_MAX_RPM || speed < STEPPER_MIN_RPM){
        stepper_status[id].error = STEPPER_INVALID_PARAM;
        return STEPPER_INVALID_PARAM;        
    }

    TIM_HandleTypeDef *tim_handle = stepper_hw_res[id].htim;
    // 获取APB2总线时钟频率，因为此处用的是TIM8
    uint32_t apb2_freq = HAL_RCC_GetPCLK2Freq();
    // 得到预分频寄存器的值
    uint16_t psc = tim_handle->Init.Prescaler;
    // 计算TIM8的计数器的时钟频率cnt_freq
    uint32_t cnt_freq = apb2_freq / (psc + 1);

    // 根据速度speed的要求，来计算TIM8的重装载值
    uint32_t auto_reload = cnt_freq / (speed) - 1;

    HAL_TIM_PWM_Stop(tim_handle, stepper_hw_res[id].tim_channel);
    // 设置TIM8的重装载值
    __HAL_TIM_SET_AUTORELOAD(tim_handle, auto_reload);
    printf("cnt_freq: %d, auto_reload: %d\n", cnt_freq, auto_reload);
    // PWM的占空比设置为50%
    __HAL_TIM_SET_COMPARE(tim_handle, stepper_hw_res[id].tim_channel, (auto_reload + 1) / 2);

    HAL_TIM_GenerateEvent(tim_handle, TIM_EVENTSOURCE_UPDATE); //强制产生更新事件，使能PWM，因为有影子寄存器，所以需要强制产生更新事件，才能使能PWM

    HAL_TIM_PWM_Start(tim_handle, stepper_hw_res[id].tim_channel);

    // 记录id对应步进电机的速度
    stepper_status[id].speed = speed;

    return STEPPER_OK;
}

//步进电机控制旋转方向
StepperErrorCode Stepper_SetDir(StepperID id, StepperDir dir){
    if(id >= STEPPER_SUM){
        return STEPPER_INVALID_ID;
    }

    if(dir == STEPPER_DIR_CW){//顺时针转动，让板子输出高电平
        HAL_GPIO_WritePin(stepper_hw_res[id].dir_port, stepper_hw_res[id].dir_pin, GPIO_PIN_RESET); // 因为光耦的翻转作用，所以io置为reset，实际板子输出是高电平
    }else if (dir == STEPPER_DIR_CCW){//逆时针转动，让板子输出高电平
        HAL_GPIO_WritePin(stepper_hw_res[id].dir_port, stepper_hw_res[id].dir_pin, GPIO_PIN_SET); // 因为光耦的翻转作用，所以io置为set，实际板子输出是低电平
    }else{
        stepper_status[id].error = STEPPER_INVALID_PARAM;
        return STEPPER_INVALID_PARAM;
    }

    // 记录id对应步进电机的旋转方向
    stepper_status[id].dir = dir;
    return STEPPER_OK;
}

//步进电机控制使能
StepperErrorCode Stepper_SetEnable(StepperID id, StepperEnableState enable){
    if(id >= STEPPER_SUM){
        return STEPPER_INVALID_ID;
    }

    // 根据id，将对应步进电机的enable端口置为有效
    if(enable == STEPPER_DISABLE){
        HAL_GPIO_WritePin(stepper_hw_res[id].en_port, stepper_hw_res[id].en_pin, GPIO_PIN_SET); // 因为光耦的翻转作用，所以io置为set，实际板子输出是低电平
        TIM_HandleTypeDef *tim_handle = stepper_hw_res[id].htim;
        HAL_TIM_PWM_Stop(tim_handle, stepper_hw_res[id].tim_channel);
    }else if(enable == STEPPER_ENABLE){
        HAL_GPIO_WritePin(stepper_hw_res[id].en_port, stepper_hw_res[id].en_pin, GPIO_PIN_RESET); // 因为光耦的翻转作用，所以io置为reset，实际板子输出是高电平
    }else{
        stepper_status[id].error = STEPPER_INVALID_PARAM;
        return STEPPER_INVALID_PARAM;
    }

    // 记录id对应步进电机的使能状态
    stepper_status[id].enable = enable;
    return STEPPER_OK;
}

StepperErrorCode Stepper_GetStatus(StepperID id, StepperStatus *status){
    if(id >= STEPPER_SUM) {
        return STEPPER_INVALID_ID;
    }

    if(status == NULL){
        stepper_status[id].error = STEPPER_INVALID_PARAM;
        return STEPPER_INVALID_PARAM;
    }

    *status = stepper_status[id];
    return STEPPER_OK;
};

/**
 * @brief       开启步进电机
 * @param       motor_num: 步进电机接口序号
 * @param       dir      : 步进电机旋转方向
 * @retval      无
 */
void stepper_star(StepperID id, StepperDir dir)
{
    TIM_HandleTypeDef *tim_handle = stepper_hw_res[id].htim;
    HAL_TIM_Base_Start_IT(tim_handle); // 开启TIM8的中断
    switch(id)
    {
        case STEPPER_1 :
        {
            Stepper_SetDir(id, dir);
            // PWM的占空比设置为50%
            __HAL_TIM_SET_COMPARE(tim_handle, stepper_hw_res[id].tim_channel, STEPPER_PERIOD / 2);
            HAL_TIM_GenerateEvent(tim_handle, TIM_EVENTSOURCE_UPDATE); //强制产生更新事件，使能PWM，因为有影子寄存器，所以需要强制产生更新事件，才能使能PWM
            HAL_TIM_PWM_Start(tim_handle, stepper_hw_res[id].tim_channel);     /* 开启对应PWM通道 */
            break;
        }
        case STEPPER_2 :
        {
            Stepper_SetDir(id, dir);
            // PWM的占空比设置为50%
            __HAL_TIM_SET_COMPARE(tim_handle, stepper_hw_res[id].tim_channel, STEPPER_PERIOD / 2);
            HAL_TIM_GenerateEvent(tim_handle, TIM_EVENTSOURCE_UPDATE); //强制产生更新事件，使能PWM，因为有影子寄存器，所以需要强制产生更新事件，才能使能PWM
            HAL_TIM_PWM_Start(tim_handle, stepper_hw_res[id].tim_channel);     /* 开启对应PWM通道 */
            break;
        }
        case STEPPER_3 :
        {
            Stepper_SetDir(id, dir);
            // PWM的占空比设置为50%
            __HAL_TIM_SET_COMPARE(tim_handle, stepper_hw_res[id].tim_channel, STEPPER_PERIOD / 2);
            HAL_TIM_GenerateEvent(tim_handle, TIM_EVENTSOURCE_UPDATE); //强制产生更新事件，使能PWM，因为有影子寄存器，所以需要强制产生更新事件，才能使能PWM
            HAL_TIM_PWM_Start(tim_handle, stepper_hw_res[id].tim_channel);     /* 开启对应PWM通道 */
            break;  
        }
        case STEPPER_4 :
        {
            Stepper_SetDir(id, dir);
            // PWM的占空比设置为50%
            __HAL_TIM_SET_COMPARE(tim_handle, stepper_hw_res[id].tim_channel, STEPPER_PERIOD / 2);
            HAL_TIM_GenerateEvent(tim_handle, TIM_EVENTSOURCE_UPDATE); //强制产生更新事件，使能PWM，因为有影子寄存器，所以需要强制产生更新事件，才能使能PWM
            HAL_TIM_PWM_Start(tim_handle, stepper_hw_res[id].tim_channel);     /* 开启对应PWM通道 */
            break;
        }
        default : break;
    }
}

/**
 * @brief       将需要转动的角度转换成脉冲数
 * @param       angle    : 需要转动的角度值
 * @param       dir      : 旋转方向
 * @param       motor_num: 步进电机接口序号
 * @retval      无
 */
void stepper_set_angle(StepperID id, uint16_t angle)
{
    stepper_status[id].pulse_count = angle / MAX_STEP_ANGLE;
    if(stepper_status[id].pulse_count == 0) 
    {
        Stepper_SetEnable(id, STEPPER_DISABLE);
    }
    else 
    {
    // stepper_star(motor_num,dir);
    printf("angle: %d, pulse_count: %d\r\n", angle, stepper_status[id].pulse_count);
    Stepper_SetEnable(id, STEPPER_ENABLE);
    stepper_star(id, stepper_status[id].dir);
    }
}

uint8_t g_run_flag = 0;
/* 中断回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM8)
    {
        g_run_flag = 1;                             /* 标志位置一 */
        stepper_status[STEPPER_1].pulse_count--;                    /* 每一个完整的脉冲就-- */
        if(stepper_status[STEPPER_1].dir == STEPPER_DIR_CW)
        {
           stepper_status[STEPPER_1].add_pulse_count++;             /* 绝对位置++ */
        }else
        {
           stepper_status[STEPPER_1].add_pulse_count--;             /* 绝对位置-- */
        }

        if(stepper_status[STEPPER_1].pulse_count == 0)                /* 当脉冲数等于1的时候 代表需要发送的脉冲个数已完成，停止定时器输出 */
        {
            // TIM_HandleTypeDef *tim_handle = stepper_hw_res[STEPPER_1].htim;
            // HAL_TIM_PWM_Stop(tim_handle, stepper_hw_res[STEPPER_1].tim_channel);    
            Stepper_SetEnable(STEPPER_1, STEPPER_DISABLE);
            printf("all done, angle:%d, pulse_count:%d\r\n",(int)(stepper_status[STEPPER_1].add_pulse_count*MAX_STEP_ANGLE), 
                   stepper_status[STEPPER_1].add_pulse_count);  /* 打印累计转动了多少角度 */
            g_run_flag = 0;
        }
    }    
}


/********************************************梯形加减速***********************************************/
speedRampData g_srd               = {STOP,STEPPER_DIR_CW,0,0,0,0,0};  /* 加减速变量 */
__IO int32_t  g_step_position     = 0;                    /* 当前位置 */
__IO uint8_t  g_motion_sta        = 0;                    /* 是否在运动？0：停止，1：运动 */
__IO uint32_t g_add_pulse_count   = 0;                    /* 脉冲个数累计 */

/*
 * @brief       生成梯形运动控制参数
 * @param       step：移动的步数 (正数为顺时针，负数为逆时针).
 * @param       accel  加速度,实际值为accel*0.1*rad/sec^2  10倍并且2个脉冲算一个完整的周期
 * @param       decel  减速度,实际值为decel*0.1*rad/sec^2
 * @param       speed  最大速度,实际值为speed*0.1*rad/sec
 * @retval      无
 */
void create_t_ctrl_param(int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
    __IO uint16_t tim_count;        /* 达到最大速度时的步数*/
    __IO uint32_t max_s_lim;        /* 必须要开始减速的步数（如果加速没有达到最大速度）*/
    __IO uint32_t accel_lim;
    TIM_HandleTypeDef *tim_handle = stepper_hw_res[STEPPER_1].htim;  /* 获取定时器句柄 */
    if(g_motion_sta != STOP)        /* 只允许步进电机在停止的时候才继续*/
        return;
    if(step < 0)                    /* 步数为负数 */
    {   
        g_srd.dir = STEPPER_DIR_CCW;            /* 逆时针方向旋转 */
        Stepper_SetDir(STEPPER_1, STEPPER_DIR_CCW);
        step = -step;               /* 获取步数绝对值 */
    }
    else
    {
        g_srd.dir = STEPPER_DIR_CW;             /* 顺时针方向旋转 */
        Stepper_SetDir(STEPPER_1, STEPPER_DIR_CW);
    }

    if(step == 1)                   /* 步数为1 */
    {
        g_srd.accel_count = -1;     /* 只移动一步 */
        g_srd.run_state = DECEL;    /* 减速状态. */
        g_srd.step_delay = 1000;    /* 默认速度 */
    }
    else if(step != 0)              /* 如果目标运动步数不为0*/
    {
        /*设置最大速度极限, 计算得到min_delay用于定时器的计数器的值 min_delay = (alpha / t)/ w*/
        g_srd.min_delay = (int32_t)(A_T_x10 /speed); //匀速运行时的计数值

        /* 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2
         step_delay = 1/tt * sqrt(2*alpha/accel)
         step_delay = ( tfreq*0.69/10 )*10 * sqrt( (2*alpha*100000) / (accel*10) )/100 */
        
        g_srd.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel))/10); /* c0 */

        max_s_lim = (uint32_t)(speed*speed / (A_x200*accel/10));/* 计算多少步之后达到最大速度的限制 max_s_lim = speed^2 / (2*alpha*accel) */

        if(max_s_lim == 0)                                      /* 如果达到最大速度小于0.5步，我们将四舍五入为0,但实际我们必须移动至少一步才能达到想要的速度 */
        {
            max_s_lim = 1;
        }
        accel_lim = (uint32_t)(step*decel/(accel+decel));       /* 这里不限制最大速度 计算多少步之后我们必须开始减速 n1 = (n1+n2)decel / (accel + decel) */

        if(accel_lim == 0)                                      /* 不足一步 按一步处理*/
        {
            accel_lim = 1;
        }
        if(accel_lim <= max_s_lim)                              /* 加速阶段到不了最大速度就得减速。。。使用限制条件我们可以计算出减速阶段步数 */
        {
            g_srd.decel_val = accel_lim - step;                 /* 减速段的步数 */
        }
        else
        {
            g_srd.decel_val = -(max_s_lim*accel/decel);         /* 减速段的步数 */
        }
        if(g_srd.decel_val == 0)                                /* 不足一步 按一步处理 */
        {
            g_srd.decel_val = -1;
        }
        g_srd.decel_start = step + g_srd.decel_val;             /* 计算开始减速时的步数 */
        
        
        if(g_srd.step_delay <= g_srd.min_delay)                 /* 如果一开始c0的速度比匀速段速度还大，就不需要进行加速运动，直接进入匀速 */
        {
            g_srd.step_delay = g_srd.min_delay;
            g_srd.run_state = RUN;
        }
        else  
        {
            g_srd.run_state = ACCEL;
        }
        g_srd.accel_count = 0;                                  /* 复位加减速计数值 */
    }
    g_motion_sta = 1;                                           /* 电机为运动状态 */
    Stepper_SetEnable(STEPPER_1, STEPPER_ENABLE);
    // __HAL_TIM_SET_COUNTER(tim_handle, 0); 
    tim_count=__HAL_TIM_GET_COUNTER(tim_handle);
    printf("g_srd.run_state:%d,g_srd.dir:%d,g_srd.step_delay:%d,g_srd.decel_start:%d,,g_srd.decel_val:%d,g_srd.min_delay:%d,g_srd.accel_count:%d\n",
            g_srd.run_state,g_srd.dir,g_srd.step_delay,g_srd.decel_start,g_srd.decel_val,g_srd.min_delay,g_srd.accel_count);
    printf("first counter:%d\n",tim_count+g_srd.step_delay/2);
    __HAL_TIM_SET_COMPARE(tim_handle,stepper_hw_res[STEPPER_1].tim_channel,tim_count+g_srd.step_delay/2);  /* 设置定时器比较值 */
    HAL_TIM_OC_Start_IT(tim_handle,stepper_hw_res[STEPPER_1].tim_channel);                                 /* 使能定时器通道 */
}

/**
  * @brief  定时器比较中断
  * @param  htim：定时器句柄指针
  * @note   无
  * @retval 无
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
 
    __IO uint32_t tim_count = 0;
    __IO uint32_t tmp = 0;
    uint16_t new_step_delay = 0;                            /* 保存新（下）一个延时周期 */
    __IO static uint16_t last_accel_delay = 0;              /* 加速过程中最后一次延时（脉冲周期） */
    __IO static uint32_t step_count = 0;                    /* 总移动步数计数器*/
    __IO static int32_t rest = 0;                           /* 记录new_step_delay中的余数，提高下一步计算的精度 */
    __IO static uint8_t i = 0;                              /* 定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲 */

    TIM_HandleTypeDef *tim_handle = stepper_hw_res[STEPPER_1].htim;  /* 获取定时器句柄 */


    if(htim->Instance==TIM8)
    {
       
        tim_count = __HAL_TIM_GET_COUNTER(tim_handle);
        tmp = tim_count + g_srd.step_delay/2;               /* 整个C值里边是需要翻转两次的所以需要除以2 */
        // printf("%d\r\n",tmp);
        __HAL_TIM_SET_COMPARE(tim_handle,stepper_hw_res[STEPPER_1].tim_channel,tmp);
        i++;                                                /* 定时器中断次数计数值 */
        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
        {
            i = 0;                                          /* 清零定时器中断次数计数值 */
            switch(g_srd.run_state)                         /* 加减速曲线阶段 */
            {
            case STOP:
                step_count = 0;                             /* 清零步数计数器 */
                rest = 0;                                   /* 清零余值 */
                /* 关闭通道*/
                HAL_TIM_OC_Stop_IT(tim_handle,stepper_hw_res[STEPPER_1].tim_channel);
                Stepper_SetEnable(STEPPER_1, STEPPER_DISABLE);
                g_motion_sta = 0;                           /* 电机为停止状态  */
                break;

            case ACCEL:
                g_add_pulse_count++;                        /* 只用于记录相对位置转动了多少度 */
                step_count++;                               /* 步数加1*/
                if(g_srd.dir == STEPPER_DIR_CW)
                {
                    g_step_position++;                      /* 绝对位置加1  记录绝对位置转动多少度*/
                }
                else
                {
                    g_step_position--;                      /* 绝对位置减1*/
                }
                g_srd.accel_count++;                        /* 加速计数值加1*/
                new_step_delay = g_srd.step_delay - (((2 *g_srd.step_delay) + rest)/(4 * g_srd.accel_count + 1));/* 计算新(下)一步脉冲周期(时间间隔) */
                rest = ((2 * g_srd.step_delay)+rest)%(4 * g_srd.accel_count + 1);                                /* 计算余数，下次计算补上余数，减少误差 */
                if(step_count >= g_srd.decel_start)         /* 检查是否到了需要减速的步数 */
                {
                    g_srd.accel_count = g_srd.decel_val;    /* 加速计数值为减速阶段计数值的初始值 */
                    g_srd.run_state = DECEL;                /* 下个脉冲进入减速阶段 */
                }
                else if(new_step_delay <= g_srd.min_delay)  /* 检查是否到达期望的最大速度 计数值越小速度越快，当你的速度和最大速度相等或更快就进入匀速*/
                {
                    last_accel_delay = new_step_delay;      /* 保存加速过程中最后一次延时（脉冲周期）*/
                    new_step_delay = g_srd.min_delay;       /* 使用min_delay（对应最大速度speed）*/
                    rest = 0;                               /* 清零余值 */
                    g_srd.run_state = RUN;                  /* 设置为匀速运行状态 */
                }
                break;

            case RUN:
                g_add_pulse_count++;
                step_count++;                               /* 步数加1 */
                if(g_srd.dir == STEPPER_DIR_CW)
                {
                    g_step_position++;                      /* 绝对位置加1 */
                }
                else
                {
                    g_step_position--;                      /* 绝对位置减1*/
                }
                new_step_delay = g_srd.min_delay;           /* 使用min_delay（对应最大速度speed）*/
                if(step_count >= g_srd.decel_start)         /* 需要开始减速 */
                {
                    g_srd.accel_count = g_srd.decel_val;    /* 减速步数做为加速计数值 */
                    new_step_delay = last_accel_delay;      /* 加阶段最后的延时做为减速阶段的起始延时(脉冲周期) */
                    g_srd.run_state = DECEL;                /* 状态改变为减速 */
                }
                break;

            case DECEL:
                step_count++;                               /* 步数加1 */
                g_add_pulse_count++;
                if(g_srd.dir == STEPPER_DIR_CW)
                {
                    g_step_position++;                      /* 绝对位置加1 */
                }
                else
                {
                    g_step_position--;                      /* 绝对位置减1 */
                }
                g_srd.accel_count++;
                new_step_delay = g_srd.step_delay - (((2 * g_srd.step_delay) + rest)/(4 * g_srd.accel_count + 1));  /* 计算新(下)一步脉冲周期(时间间隔) */
                rest = ((2 * g_srd.step_delay)+rest)%(4 * g_srd.accel_count + 1);                                   /* 计算余数，下次计算补上余数，减少误差 */

                /* 检查是否为最后一步 */
                if(g_srd.accel_count >= 0)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
                {
                    g_srd.run_state = STOP;
                }
                break;
            }
            g_srd.step_delay = new_step_delay;              /* 为下个(新的)延时(脉冲周期)赋值 */
        }
    }
}
