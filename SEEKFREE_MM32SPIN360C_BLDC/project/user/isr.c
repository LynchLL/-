
#include "HAL_conf.h"
#include "gpio.h"
#include "adc.h"
#include "timer.h"

#include "pwm_input.h"
#include "opamp.h"
#include "comp.h"
#include "hall.h"
#include "pid.h"
#include "motor.h"
#include "move_filter.h"
#include "bldc_config.h"
#include "isr.h"




void TIM16_IRQHandler(void)
{
    TIM16->CNT = 0;
    if (TIM_GetITStatus(TIM16, TIM_IT_CC1) != RESET) 
    {
        if(GPIO_ReadInputDataBit(MOTOR_PWM_PORT, MOTOR_PWM_PIN))
        {
            //获取低电平持续时间并计算周期
            duty_low = TIM_GetCapture1(TIM16);
            period = duty_low + duty_high;
            duty_cycle = (float)duty_high/period;
        }
        else
        {
            //获取高电平持续时间与
            duty_high = TIM_GetCapture1(TIM16);
        }
        TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);
    }

    if (TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET) 
    {
        //当pwm占空比为0或者100%时 根据PWM引脚电平状态设置电机转速
        if(GPIO_ReadInputDataBit(MOTOR_PWM_PORT, MOTOR_PWM_PIN))
        {
            duty_high = period;
            duty_cycle = 1;//100%占空比
        }
        else
        {
            duty_high = 0;
            duty_cycle = 0;//0%占空比
        }
        TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
    }
}


void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{                           
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);     //清除中断标志位   
		
        //检查使能开关状态
        motor_en();
        
		//根据接收到的信号，去计算出需要设置的速度值
        if(!pwm_dir_get())
        {
            motor_control.set_speed = motor_control.max_speed * duty_cycle;
        }
        else
        {
            motor_control.set_speed = -motor_control.max_speed * duty_cycle;
        }
        
    #if BLDC_SPEED_CURVE_ENABLE==1
        //计算加速曲线 通过设置参数可以调节加速的响应速度
        motor_speed_curve();
    #else
        closed_loop.target_speed = motor_control.set_speed;
    #endif

        motor_set_dir();
        
    #if BLDC_CLOSE_LOOP_ENABLE==1
        //进行PI闭环计算
        closed_loop_pi_calc(speed_filter.data_average);
    #else
        closed_loop.out = (long)motor_control.set_speed*closed_loop.out_max/motor_control.max_speed;
    #endif

        //输出动力
        motor_power_out();
    }
}


void ADC1_IRQHandler(void)
{
	//清除标志位
    ADC_ClearFlag(ADC1, ADC_IT_EOC);
    
    //读取adc的值
    adc_read();
    
	//霍尔扫描
	scan_hall_status();

    if(1 > commutation_delay--)
    {//延时时间到 开始换相
        commutation_delay = 0;
        motor_commutation(next_hall_value);
    }
}


void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  	
	}
	
	if(TIM_GetITStatus(TIM1, TIM_IT_Break) != RESET) 
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Break);  
        //当刹车信号有效的时候会运行这里的代码
        //例如点亮一个LED灯来指示
        led_fault_control(LED_ON);
        
        
	}
}


