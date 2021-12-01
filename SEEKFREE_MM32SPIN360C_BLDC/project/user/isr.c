
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
            //��ȡ�͵�ƽ����ʱ�䲢��������
            duty_low = TIM_GetCapture1(TIM16);
            period = duty_low + duty_high;
            duty_cycle = (float)duty_high/period;
        }
        else
        {
            //��ȡ�ߵ�ƽ����ʱ����
            duty_high = TIM_GetCapture1(TIM16);
        }
        TIM_ClearITPendingBit(TIM16, TIM_IT_CC1);
    }

    if (TIM_GetITStatus(TIM16, TIM_IT_Update) != RESET) 
    {
        //��pwmռ�ձ�Ϊ0����100%ʱ ����PWM���ŵ�ƽ״̬���õ��ת��
        if(GPIO_ReadInputDataBit(MOTOR_PWM_PORT, MOTOR_PWM_PIN))
        {
            duty_high = period;
            duty_cycle = 1;//100%ռ�ձ�
        }
        else
        {
            duty_high = 0;
            duty_cycle = 0;//0%ռ�ձ�
        }
        TIM_ClearITPendingBit(TIM16, TIM_IT_Update);
    }
}


void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{                           
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);     //����жϱ�־λ   
		
        //���ʹ�ܿ���״̬
        motor_en();
        
		//���ݽ��յ����źţ�ȥ�������Ҫ���õ��ٶ�ֵ
        if(!pwm_dir_get())
        {
            motor_control.set_speed = motor_control.max_speed * duty_cycle;
        }
        else
        {
            motor_control.set_speed = -motor_control.max_speed * duty_cycle;
        }
        
    #if BLDC_SPEED_CURVE_ENABLE==1
        //����������� ͨ�����ò������Ե��ڼ��ٵ���Ӧ�ٶ�
        motor_speed_curve();
    #else
        closed_loop.target_speed = motor_control.set_speed;
    #endif

        motor_set_dir();
        
    #if BLDC_CLOSE_LOOP_ENABLE==1
        //����PI�ջ�����
        closed_loop_pi_calc(speed_filter.data_average);
    #else
        closed_loop.out = (long)motor_control.set_speed*closed_loop.out_max/motor_control.max_speed;
    #endif

        //�������
        motor_power_out();
    }
}


void ADC1_IRQHandler(void)
{
	//�����־λ
    ADC_ClearFlag(ADC1, ADC_IT_EOC);
    
    //��ȡadc��ֵ
    adc_read();
    
	//����ɨ��
	scan_hall_status();

    if(1 > commutation_delay--)
    {//��ʱʱ�䵽 ��ʼ����
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
        //��ɲ���ź���Ч��ʱ�����������Ĵ���
        //�������һ��LED����ָʾ
        led_fault_control(LED_ON);
        
        
	}
}


